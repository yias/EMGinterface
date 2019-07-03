#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"


#include "EMGinterface/additional_functions.h"


#include <math.h> 

#include <vector>
#include <string>
#include <stdio.h>
#include <iostream>

#include <algorithm>





int sRate=100;                                                   // set the sample rate (Hz)

int mocapCounter=0; 
double startTime;                                             // counter for messages from the mocap system


// ----------------- variables for the robot base -----------------

bool _firstKukaBasePoseReceived=false;
double robotBaseStamp=0;

std::vector<double> robot_base_position(3,0); 	// the position of the robot's base as received from the mocap system
std::vector<double> robot_base_orienation(4,0);	// the orienation of the robot's base
//Eigen::Matrix3f robot_base_rotMatrix;		// the rotation matrix of the orientation of the robot's base 



std::vector<double> handPosition(3,0);                          // vector for the position of the hand
std::vector<double> handVelocity(3,0);                          // vector for the velocity of the hand

std::vector< std::vector<double> > mocapHistoryPosition(3);      // vector for the position of the marker on hand
std::vector< std::vector<double> > mocapHistoryVelocity(3);      // vector for velocity of the marker on hand
std::vector<double> velocityNormHistory;                         // vector for the norm of the velocity of the hand



bool _firstHandPoseReceived=false;


// ----------------- callback function for the base listener -----------------

void robotBaseListener(const geometry_msgs::PoseStamped& msg){

	robot_base_position[0] = msg.pose.position.x;
    robot_base_position[1] = msg.pose.position.y;
    robot_base_position[2] = msg.pose.position.z;
  	robot_base_orienation[0]= msg.pose.orientation.w;
    robot_base_orienation[1] = msg.pose.orientation.x;
    robot_base_orienation[2] = msg.pose.orientation.y;
    robot_base_orienation[3] = msg.pose.orientation.z;
	//robot_base_rotMatrix = Utils::quaternionToRotationMatrix(robot_base_orienation);


	if(!_firstKukaBasePoseReceived)
	{
		ROS_INFO("Initial robot base pose received\n");
		_firstKukaBasePoseReceived = true;
		robotBaseStamp=msg.header.seq;
		
		
	}
}

// ----------------- callback function for hand -----------------


void handListener(const geometry_msgs::PoseStamped& mocapmsg){

    /*-- Callback function for subscriber of the mocap system --*/

    handPosition[0]=mocapmsg.pose.position.x;
    handPosition[1]=mocapmsg.pose.position.y;
    handPosition[2]=mocapmsg.pose.position.z;

    if(!_firstHandPoseReceived){
        _firstHandPoseReceived=true;
        ROS_INFO("Initial hand pose received\n");
    }


    //mocapTime.push_back((ros::Time::now().toSec())-startTime);

    // for(int i=0;i<3;i++){

    //     mocapHistoryPosition[i].push_back(mocapPosition[i]);

    // }

    // if(mocapCounter>0){
    //    std::vector<double> previousSample(3,0);
    //    for(int i=0;i<3;i++){
    //        previousSample[i]=mocapHistoryPosition[i][mocapCounter-1];
    //    }

    //    //std::cout<<"x: "<<previousSample[0]<<", y: "<<previousSample[1]<<",z: "<<previousSample[2]<<"\n";
    //    mocapVelocity=calcDtVelocity(mocapPosition,previousSample,(double)1/std::min((double)sRate,(double)mocapRate));

    //     for(int i=0;i<3;i++){
    //         mocapHistoryVelocity[i].push_back(mocapVelocity[i]);
    //     }
    //     velocityNormHistory.push_back(velocityNorm(mocapHistoryVelocity,(int)(lookBack*sRate)));
    //     //std::cout<<"\nvel: "<<velocityNormHistory.back()<<"\n";
    //     //checkVelocityHistory.push_back(check_velocity(velocityNormHistory.back(),velThreshold));
    //     //checkVelocityHistory.push_back(1);
    //    // std::cout<<"velocity: " << check_velocity(velocityNormHistory.back(),velThreshold) << "\n";// << velocityNormHistory.back() << " "



    // }


    mocapCounter++;
    //ROS_INFO("I heard: [%d] messages from mocap\n", mocapCounter);
}







int main(int argc, char **argv)
{

    std_msgs::Int16 msgInt;

    geometry_msgs::Twist avVelocity;



    // initialize the node
    ros::init(argc, argv, "velcalculator");

    ros::NodeHandle n;


    // set a publisher for publishing the grasp type

    ros::Publisher veldir_pub2=n.advertise<std_msgs::Int16>("EMGinterfaceInt/veldir", 100);

    ros::Publisher velocity_pub=n.advertise<geometry_msgs::Twist>("EMGinterfaceInt/avVel", 100);


    // set the subscribers to listen the classification outcome from the windows machine and the position of the hand

    ros::Subscriber robotBaseSub=n.subscribe("Robot_base/pose", 10, robotBaseListener);

    ros::Subscriber mocapSub=n.subscribe("hand/pose", 10, handListener);

    startTime=ros::Time::now().toSec();

    // set the loop rate
    ros::Rate loop_rate(sRate);

    while(!_firstHandPoseReceived){
    
        // std::cout<<"okook\n";
        ros::spinOnce();
        loop_rate.sleep();  
    }


    std::vector< std::vector<double> > mVel(3);
    std::vector<double> dtVel(3,0);
    std::vector<double> previouPos(3,0);
    std::vector<double> averageVel(3,0);

    double speed=0.0;


    int count = 0;
    while (ros::ok())
    {
        // set the grasp type

        if(count==0){
            // previouPos[0]=(robot_base_position[0]-handPosition[0]);
            // previouPos[1]=(robot_base_position[1]-handPosition[1]);
            // previouPos[2]=(robot_base_position[2]-handPosition[2]);            

            previouPos[0]=handPosition[0];
            previouPos[1]=handPosition[1];
            previouPos[2]=handPosition[2];            
            
        }else{

            // dtVel[0]=(robot_base_position[0]-handPosition[0]-previouPos[0])/(1/(double)sRate);
            // dtVel[1]=(robot_base_position[1]-handPosition[1]-previouPos[1])/(1/(double)sRate);
            // dtVel[2]=(robot_base_position[2]-handPosition[2]-previouPos[2])/(1/(double)sRate);

            // previouPos[0]=(robot_base_position[0]-handPosition[0]);
            // previouPos[1]=(robot_base_position[1]-handPosition[1]);
            // previouPos[2]=(robot_base_position[2]-handPosition[2]);

            dtVel[0]=(handPosition[0]-previouPos[0])/(1/(double)sRate);
            dtVel[1]=(handPosition[1]-previouPos[1])/(1/(double)sRate);
            dtVel[2]=(handPosition[2]-previouPos[2])/(1/(double)sRate);

            // std::cout<<dtVel[0]<<", "<<dtVel[1]<<", "<<dtVel[2]<<"\n";

            previouPos[0]=handPosition[0];
            previouPos[1]=handPosition[1];
            previouPos[2]=handPosition[2];

        }

        

        mVel[0].push_back(dtVel[0]);
        mVel[1].push_back(dtVel[1]);
        mVel[2].push_back(dtVel[2]);

        // std::cout<<dtVel[0]<<", "<<dtVel[1]<<", "<<dtVel[2]<<"\n";

        if(count>10){

            // std::cout<<"mVel: "  << mVel.size() << ", " << mVel[0].size() <<"\n";

            averageVel=calvAverageVelocity(mVel,10);

            // std::cout<<averageVel[0]<<", "<<averageVel[1]<<", "<<averageVel[2]<<"\n";

            avVelocity.linear.x=averageVel[0];
            avVelocity.linear.y=averageVel[1];
            avVelocity.linear.z=averageVel[2];

            velocity_pub.publish(avVelocity);

            speed=std::sqrt(std::pow(averageVel[0],2)+std::pow(averageVel[1],2)+std::pow(averageVel[2],2));

            std::cout<<"speed: " <<speed<<"\n";

            if(speed>=0.1){

                msgInt.data=1;
             
             }else{

                msgInt.data=0;
            }

             // publish the messages

            veldir_pub2.publish(msgInt);
        }

        

        

       

        // if the key 't' is pressed, save the data and clear the data for the next trial

        // if(getch_()=='t'){
        //     saveRecordings();

        // }



        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}