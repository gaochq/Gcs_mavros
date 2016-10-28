#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include "std_msgs/Float64MultiArray.h"

geometry_msgs::TwistStamped  fmu_controller_setvel;

void joyVelCallback(const geometry_msgs::Twist twist)
{
    ROS_INFO("keyboard received");
    fmu_controller_setvel.twist.linear.x = twist.linear.x;
    fmu_controller_setvel.twist.linear.y = twist.linear.y;
    fmu_controller_setvel.twist.linear.z = twist.linear.z;
    fmu_controller_setvel.twist.angular.x = twist.angular.x;
    fmu_controller_setvel.twist.angular.y = twist.angular.y;
    fmu_controller_setvel.twist.angular.z = twist.angular.z;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_vel");
    ros::NodeHandle nh;

    ros::Publisher mavros_control_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",1000); // here the number is the buffer
    // ros::Publisher mavros_control_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/mavros/setpoint_accel/accel",1000); // here the number is the buffer

    ros::Subscriber joy_vel   = nh.subscribe("/mavros/local_position/velocity",100,joyVelCallback);


    ros::Rate loop_rate(50);
    while(ros::ok())
    {
		if(fmu_controller_setvel.twist.angular.z > 0.8)
        	fmu_controller_setvel.twist.angular.z = 0.8;

        mavros_control_pub.publish(fmu_controller_setvel);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}



