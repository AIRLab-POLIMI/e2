/*
 * odometry_pub.cpp - AIRLab (Politecnico di Milano)
 * 
 * description
 *
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 15/feb/2014
 *
 */

#include "ros/ros.h"
#include "ros/package.h"
#include "odometry.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#define ROS_NODE_RATE	500
#define ROS_NODE_NAME	"odometry_pub"

float tanSpeed = 0.0;
float rotSpeed = 0.0;
Odometry *odom ;

void getWheelData(const geometry_msgs::TwistConstPtr &msg)
{
	ROS_INFO("[Odom]:: Velocity command");
	tanSpeed = msg->linear.x;
	rotSpeed = msg->angular.z;
}
void getInitialPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg )
{
	ROS_INFO("[Odom]:: Received new robot pose. Update");
	odom->~Odometry();

	odom = new Odometry(msg->pose.pose);

}

int main(int argc, char **argv)
{
	  ros::init(argc, argv, ROS_NODE_NAME);
	  ros::NodeHandle nh;

	  //Messages subscribers
	  ros::Subscriber initialpose = nh.subscribe("initialpose", 100, getInitialPose);
	  ros::Subscriber subWheelData = nh.subscribe("cmd_vel", 100, getWheelData);
	  ros::Publisher odom_pub =  nh.advertise<nav_msgs::Odometry>("odom", 1000);


	  ROS_INFO("[Odom]:: Node started");
	  ros::Rate r(ROS_NODE_RATE);

	  odom = new Odometry();

	  while(ros::ok())	//ROS LOOP
	  {

		  odom->ComputeOdometry(tanSpeed,rotSpeed);
		  odom_pub.publish(odom->getOdometryMsg());

		  ros::spinOnce();
		  r.sleep();
	  }

	  return 0;
}



