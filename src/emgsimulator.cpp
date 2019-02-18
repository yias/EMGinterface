#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Int16.h"
// #include <EMGinterface/mvOutput.h>
// #include "EMGinterface/additional_functions.h"
#include "win_bridge/vtmsg.h"

#include <math.h> 

#include <vector>
#include <string>
#include <stdio.h>
#include <iostream>


int main(int argc, char **argv)
{

	

	// initialize the node
    ros::init(argc, argv, "emgsimulator");

    ros::NodeHandle n;


    // set a publisher for publishing the grasp type

    ros::Publisher simulator_pub=n.advertise<win_bridge::vtmsg>("win_pub", 100);

    win_bridge::vtmsg daqmsg;


    std::vector<uint8_t> vOutput(25);

    uint8_t myRandom[]={1,1,1,2,2,1,1,1,2,1,1,2,1,2,1,2,2,1,1,1,1,1,2,1,1};

    vOutput.assign(myRandom,myRandom+25);

    for(int i=0;i<(int)vOutput.size();i++){
    	std::cout << vOutput[i] << " ";
    }

    std::cout << "\n";

    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        // set the grasp type

    	if(count>=(int)vOutput.size()){
    		daqmsg.vote=vOutput[(int)vOutput.size()-1];
    		// std::cout<<"ok1 "<<daqmsg.vote<< " <- " << vOutput[(int)vOutput.size()-1] <<"\n";
    	}else{
    		daqmsg.vote=vOutput[count];
    		// std::cout<<"ok2 "<<daqmsg.vote<< "<- " << vOutput[count] <<"\n";	
    	}
        

        simulator_pub.publish(daqmsg);


        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;


}