/**
 * \file velocity_mux.cpp
 * \brief
 * \author Zixuan XU
 * \version 0.1
 * \date 19/11/2023
 * 
 * \param[in] 
 * 
 * Subscribes to: <BR>
 * planning_pbvs_landing
 * local_planning
 * 
 * Publishes to: <BR>
 * px4_sender
 *
 * Description
 * subscribe velocity twist command from the landing node and the planning node, 
 * publish the command to px4 depending on different modes
 */


//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

#include <ros/ros.h>

#include <std_msgs/Int8.h>

#include "mission_utils.h"
#include "message_utils.h"

//ROS
// #include "ros/ros.h"

// Include here the ".h" files corresponding to the topic type you use.
// ...

// You may have a number of globals here.
prometheus_msgs::ControlCommand planningCmdTwist, landingCmdTwist, Command_Now;
std_msgs::Int8 flight_mode;  // 0 for planning, 1 for landing

// Callback functions...

void planningCmd_cb(const prometheus_msgs::ControlCommand::ConstPtr& msg){
    // ... Callback function code
    // if( msg->Command_ID  >  planningCmdTwist.Command_ID )
        planningCmdTwist = *msg;
    // else
        // ROS_INFO_STREAM_ONCE("Velocity_mux has not received planning command.");
}

void landingCmd_cb(const prometheus_msgs::ControlCommand::ConstPtr& msg)
{
    // CommandID必须递增才会被记录
    // if( msg->Command_ID  >  landingCmdTwist.Command_ID )
        landingCmdTwist = *msg;
    // else
        // ROS_INFO_STREAM_ONCE("Velocity_mux has not received landing command.");
}

void flightMode_cb(const std_msgs::Int8::ConstPtr& msg)
{
   flight_mode = *msg;
}

int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "velocity_mux");

    // Define your node handles: YOU NEED AT LEAST ONE !
    ros::NodeHandle nh("~");   // local

    // Read the node parameters if any
    // ...

    // Declare your node's subscriptions and service clients
    ros::Subscriber planningCmd_sub = nh.subscribe<prometheus_msgs::ControlCommand>("/prometheus/planning/control_command", 10, planningCmd_cb);
    ros::Subscriber landingCmd_sub = nh.subscribe<prometheus_msgs::ControlCommand>("/prometheus/landing/control_command", 10, landingCmd_cb);
    ros::Subscriber flightMode_sub = nh.subscribe<std_msgs::Int8>("/flight_mode_mux", 10, flightMode_cb);

    // Declare you publishers and service servers
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    ros::Rate rate(30);   // Or other rate.
    while (ros::ok()){
        ros::spinOnce();

        // Your node's code goes here.
        if(flight_mode.data==0){
            ROS_INFO("velocity_mux: planning mode");
            Command_Now = planningCmdTwist;
        } 
        else if(flight_mode.data==1){
            ROS_INFO("velocity_mux: landing mode");
            Command_Now = landingCmdTwist;
        }        

        command_pub.publish(Command_Now);

        rate.sleep();
    }
}
