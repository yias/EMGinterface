#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Int16.h"


#include "EMGinterface/additional_functions.h"


#include <math.h> 

#include <vector>
#include <string>
#include <stdio.h>
#include <iostream>

#include <algorithm>





int sRate=100;                                                   // set the sample rate (Hz)


// ----------------- variables for the robot base -----------------

bool _firstKukaBasePoseReceived=false;
double robotBaseStamp=0;

Eigen::VectorXf robot_base_position(3); 	// the position of the robot's base as received from the mocap system
Eigen::VectorXf robot_base_orienation(4);	// the orienation of the robot's base
Eigen::Matrix3f robot_base_rotMatrix;		// the rotation matrix of the orientation of the robot's base 



std::vector<double> handPosition(3,0);                          // vector for the position of the hand
std::vector<double> handVelocity(3,0);                          // vector for the velocity of the hand

std::vector< std::vector<double> > mocapHistoryPosition(3);      // vector for the position of the marker on hand
std::vector< std::vector<double> > mocapHistoryVelocity(3);      // vector for velocity of the marker on hand
std::vector<double> velocityNormHistory;                         // vector for the norm of the velocity of the hand



bool _firstHandPoseReceived=false;


// ----------------- callback function for the base listener -----------------

void robotBaseListener(const geometry_msgs::PoseStamped& msg){

	robot_base_position << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
  	robot_base_orienation << msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z;
	robot_base_rotMatrix = Utils::quaternionToRotationMatrix(robot_base_orienation);


	if(!_firstKukaBasePoseReceived)
	{
		ROS_INFO("Initial robot base pose received\n");
		_firstKukaBasePoseReceived = true;
		robotBaseStamp=msg.header.seq;
		
		
	}
}

// ----------------- callback function for hand -----------------


void mocapListener(const geometry_msgs::PoseStamped& mocapmsg){

    /*-- Callback function for subscriber of the mocap system --*/

    mocapPosition[0]=mocapmsg.pose.position.x;
    mocapPosition[1]=mocapmsg.pose.position.y;
    mocapPosition[2]=mocapmsg.pose.position.z;

    if(!_firstHandPoseReceived){
        _firstHandPoseReceived=true;
        ROS_INFO("Initial hand pose received\n");
    }


    mocapTime.push_back((ros::Time::now().toSec())-startTime);

    for(int i=0;i<3;i++){

        mocapHistoryPosition[i].push_back(mocapPosition[i]);

    }

    if(mocapCounter>0){
       std::vector<double> previousSample(3,0);
       for(int i=0;i<3;i++){
           previousSample[i]=mocapHistoryPosition[i][mocapCounter-1];
       }

       //std::cout<<"x: "<<previousSample[0]<<", y: "<<previousSample[1]<<",z: "<<previousSample[2]<<"\n";
       mocapVelocity=calcDtVelocity(mocapPosition,previousSample,(double)1/std::min((double)sRate,(double)mocapRate));

        for(int i=0;i<3;i++){
            mocapHistoryVelocity[i].push_back(mocapVelocity[i]);
        }
        velocityNormHistory.push_back(velocityNorm(mocapHistoryVelocity,(int)(lookBack*sRate)));
        //std::cout<<"\nvel: "<<velocityNormHistory.back()<<"\n";
        //checkVelocityHistory.push_back(check_velocity(velocityNormHistory.back(),velThreshold));
        //checkVelocityHistory.push_back(1);
       // std::cout<<"velocity: " << check_velocity(velocityNormHistory.back(),velThreshold) << "\n";// << velocityNormHistory.back() << " "



    }


    mocapCounter++;
    //ROS_INFO("I heard: [%d] messages from mocap\n", mocapCounter);
}







int main(int argc, char **argv)
{

    // set the message for publishing the graps type
    EMGinterface::mvOutput graspmsg;

    std_msgs::Int16 msgInt;



    // initialize the node
    ros::init(argc, argv, "emginterface");

    ros::NodeHandle n;


    // set a publisher for publishing the grasp type

    ros::Publisher graspType_pub=n.advertise<EMGinterface::mvOutput>("EMGinterface/grasp_type", 100);

    ros::Publisher graspType_pub2=n.advertise<std_msgs::Int16>("EMGinterfaceInt/grasp_type", 100);


    // set the subscribers to listen the classification outcome from the windows machine and the position of the hand

    ros::Subscriber daqSub = n.subscribe("win_pub", 2, daqListener);

    ros::Subscriber mocapSub=n.subscribe("hand/pose", 10, mocapListener);

    startTime=ros::Time::now().toSec();

    // set the loop rate
    ros::Rate loop_rate(sRate);

    while(!_firstHandPoseReceived){
    
        // std::cout<<"okook\n";
        ros::spinOnce();
        loop_rate.sleep();  
    }



    int count = 0;
    while (ros::ok())
    {
        // set the grasp type

        if(count>5){
            if(check_velocity(velocityNormHistory.back(),velThreshold)){
                graspmsg.vote=grasp_type;


                msgInt.data=grasp_type;
             }else{
                graspmsg.vote=0;


                msgInt.data=0;
            }
        }

        

        

        // publish the messages

        graspType_pub.publish(graspmsg);

        graspType_pub2.publish(msgInt);

        // if the key 't' is pressed, save the data and clear the data for the next trial

        if(getch_()=='t'){
            saveRecordings();

        }



        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}