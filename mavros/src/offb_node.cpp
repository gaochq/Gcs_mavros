/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include "std_msgs/Float64MultiArray.h"

mavros_msgs::State current_state;
double outputdata[4] = {0.0,0.0,0.0,0.0};

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void myCallback(const std_msgs::Float64MultiArray::ConstPtr& array)
{
    int i = 0;
        // print all the remaining numbers
        for(std::vector<double>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
        {
            outputdata[i] = *it;
            i++;
        }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber my_subscriber_object= nh.subscribe("chatter",1,myCallback);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = 0;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else 
        {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        if(outputdata[2]<=0)
            outputdata[2] = 0;
        pose.pose.position.x = outputdata[0];
        pose.pose.position.y = outputdata[1];
        pose.pose.position.z = outputdata[2];
		
		// if(outputdata[3]==1.0)
		// 	arm_cmd.request.value = true;
		// else if(outputdata[3]==0.0)
		// 	arm_cmd.request.value = false;
		// if( arming_client.call(arm_cmd) && arm_cmd.response.success)
		// {
		// 	ROS_INFO("Vehicle armed");
		// }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
