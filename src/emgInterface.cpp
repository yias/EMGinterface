#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include <EMGinterface/vtmsg.h>
#include "EMGinterface/additional_functions.h"

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math.h> 

#include <vector>
#include <string>
#include <stdio.h>
#include <iostream>



int sRate=300;                                                   // set the sample rate (Hz)
int trialCounter=0;                                              // counter for the trials
double startTime;


/*-- Variables related to the mocap system --*/

int mocapCounter=0;                                              // counter for messages from the mocap system
double lookBack=0.1;                                             // the timewindow to look back for the average velocity
double velThreshold=0.018;                                       // velocity threshold for destinguish the motion or no=motion of the hand
int mocapRate=250;                                               // the sample rate of the motion capture system

std::vector<double> mocapPosition(3,0);                          // vector for the position of the hand
std::vector<double> mocapVelocity(3,0);                          // vector for the velocity of the hand

std::vector< std::vector<double> > mocapHistoryPosition(3);      // vector for the position of the marker on hand
std::vector< std::vector<double> > mocapHistoryVelocity(3);      // vector for velocity of the marker on hand
std::vector<double> velocityNormHistory;                         // vector for the norm of the velocity of the hand

std::vector<double> mocapTime;                                   // timestamp for the mocap system
std::vector<double> checkVelocityHistory; // history of the velocity checking


/*-- Variables related to the windows system --*/

int daqCounter=0;                                                // counter for messages from the windows machine

int nbClasses=5;                                                 // number of different classes

int grasp_type=0;                                                // the outecome of the majority vote

double grasp_threshold=0.2;                                      // confidence threshold to change the grasp

std::vector<int> mVotes;                                         // a vector to store the classification outcome for each time window

std::vector<int> graspTypeHistory;                               // a vector with all the grasptypes

std::vector<double> graspTime;                                   // the time stamp of the listener of the windows machine


//-- Callback functions --

void daqListener(const decision_maker::vtmsg daqmsg){



    /*-- Callback function for subscriber of the windows machine --*/

    graspTime.push_back((ros::Time::now().toSec())-startTime);



    if(daqCounter>0){
    graspTime.push_back((ros::Time::now().toSec())-startTime);
    mVotes.push_back(daqmsg.vote);

//    mVotes.push_back(1);

    //checkVelocityHistory.push_back(check_velocity(velocityNormHistory.back(),velThreshold));

    //if(check_velocity(velocityNormHistory.back(),velThreshold)) {

        grasp_type=majority_vote(mVotes,nbClasses, grasp_threshold,grasp_type);

        std::cout<<"grasp type: "<<grasp_type<<"\n";
        graspTypeHistory.push_back(grasp_type);

    //}
    }

    daqCounter++;

    //ROS_INFO("I heard: [%d] messages from daq\n", daqCounter);
}



void mocapListener(const geometry_msgs::PoseStamped& mocapmsg){

    /*-- Callback function for subscriber of the mocap system --*/

    mocapPosition[0]=mocapmsg.pose.position.x;
    mocapPosition[1]=mocapmsg.pose.position.y;
    mocapPosition[2]=mocapmsg.pose.position.z;


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
    decision_maker::vtmsg graspmsg;


    // setting the velocity of the joints to maximum
    righHand_msg.velocity=joint_velocity;
    leftHand_msg.velocity=joint_velocity;


    // initialize the node
    ros::init(argc, argv, "decisionMaker");

    ros::NodeHandle n;


    // set a publisher for publishing the grasp type

    ros::Publisher graspType_pub=n.advertise<win_bridge::vtmsg>("EMGinterface/grasp_type", 100);



    // set the subscribers to listen the classification outcome from the windows machine and the position of the hand

    ros::Subscriber daqSub = n.subscribe("win_pub", 2, daqListener);

    ros::Subscriber mocapSub=n.subscribe("HAND/pose", 10, mocapListener);



    startTime=ros::Time::now().toSec();

    // set the loop rate
    ros::Rate loop_rate(sRate);




    int count = 0;
    while (ros::ok())
    {
        // set the grasp type

        graspmsg.vote=grasp_type;


        // publish the messages

        graspType_pub.publish(graspmsg);

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