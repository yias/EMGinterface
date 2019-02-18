#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Int16.h"
#include <EMGinterface/mvOutput.h>
#include "EMGinterface/additional_functions.h"
#include "win_bridge/vtmsg.h"

// #include <Eigen/Eigen>
// #include <Eigen/Geometry>
// #include <Eigen/Dense>
#include <math.h> 

#include <vector>
#include <string>
#include <stdio.h>
#include <iostream>

#include <algorithm>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <unistd.h>
#include <termios.h>



int sRate=300;                                                   // set the sample rate (Hz)
int trialCounter=0;                                              // counter for the trials
double startTime;


/*-- Variables related to the mocap system --*/

int mocapCounter=0;                                              // counter for messages from the mocap system
double lookBack=0.1;                                             // the timewindow to look back for the average velocity
double velThreshold=0.018;                                       // velocity threshold for destinguish the motion or no=motion of the hand
int mocapRate=200;                                               // the sample rate of the motion capture system

std::vector<double> mocapPosition(3,0);                          // vector for the position of the hand
std::vector<double> mocapVelocity(3,0);                          // vector for the velocity of the hand

std::vector< std::vector<double> > mocapHistoryPosition(3);      // vector for the position of the marker on hand
std::vector< std::vector<double> > mocapHistoryVelocity(3);      // vector for velocity of the marker on hand
std::vector<double> velocityNormHistory;                         // vector for the norm of the velocity of the hand

std::vector<double> mocapTime;                                   // timestamp for the mocap system
std::vector<double> checkVelocityHistory; // history of the velocity checking


/*-- Variables related to the windows system --*/

int daqCounter=0;                                                // counter for messages from the windows machine

int nbClasses=2;                                                 // number of different classes

int grasp_type=0;                                                // the outecome of the majority vote

double grasp_threshold=0.2;                                      // confidence threshold to change the grasp

std::vector<int> mVotes;                                         // a vector to store the classification outcome for each time window

std::vector<int> graspTypeHistory;                               // a vector with all the grasptypes

std::vector<double> graspTime;                                   // the time stamp of the listener of the windows machine

void saveRecordings();                                           // a function to save the data after each trial
int getch_();                                                    // a function to catch a key press asynchronously


bool _firstHandPoseReceived=false;

//-- Callback functions --

void daqListener(const win_bridge::vtmsg daqmsg){



    /*-- Callback function for subscriber of the windows machine --*/

    graspTime.push_back((ros::Time::now().toSec())-startTime);



    if(daqCounter>0){
        graspTime.push_back((ros::Time::now().toSec())-startTime);
        mVotes.push_back(daqmsg.vote);
        std::cout<<(int)daqmsg.vote<<"\n";

//    mVotes.push_back(1);

        checkVelocityHistory.push_back(check_velocity(velocityNormHistory.back(),velThreshold));

        if(check_velocity(velocityNormHistory.back(),velThreshold)) {

            grasp_type=majority_vote(mVotes,nbClasses, grasp_threshold,grasp_type);

            std::cout<<"grasp type: "<<grasp_type<<"\n";
            graspTypeHistory.push_back(grasp_type);

        }
    }
    

    daqCounter++;

    //ROS_INFO("I heard: [%d] messages from daq\n", daqCounter);
}



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


void saveRecordings(){




    std::cout<<"\nTrial " << trialCounter+1 <<" has been saved\n";

    // clear the data for the next trial

    for(int i=0;i<(int)mocapHistoryPosition.size();i++){
        mocapHistoryPosition[i].clear();
    }

    for(int i=0;i<(int)mocapHistoryVelocity.size();i++){
        mocapHistoryVelocity[i].clear();
    }

    mocapTime.clear();
    checkVelocityHistory.clear();
    graspTypeHistory.clear();
    mVotes.clear();
    velocityNormHistory.clear();

    std::cout<<"Data cleared\n";
    std::cout<<"Ready for the next trial\n";
    trialCounter++;
    grasp_type=0;
    mocapCounter=0;
    daqCounter=0;
    startTime=ros::Time::now().toSec();

}



int getch_(){
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);                  // save old settings

  newt = oldt;
  newt.c_lflag &= ~(ICANON);                        // disable buffering
  newt.c_cc[VMIN] = 0; newt.c_cc[VTIME] = 0;
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);         // apply new settings

  int c = getchar();                                // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);         // restore old settings
  return c;
}
