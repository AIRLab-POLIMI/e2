/*
 * neck.cpp - AIRLab (Politecnico di Milano)
 *
 *	This node receive messages on default topic and controls neck moviments for e2 robot
 *
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 28/feb/2014
 *
 */

#include "ros/ros.h"
#include "NeckInterface.h"

#define ROS_NODE_NAME	"e2_neck_controller"

using namespace std;

//=====================================
// Main Code
//=====================================
int main(int argc, char **argv)
{
	ros::init(argc, argv, ROS_NODE_NAME);

	ROS_INFO("["ROS_NODE_NAME"]:: Node Started");

	// Initialize main  class
	NeckInterface neck(ros::this_node::getName());

	ros::spin();
	return 0;
}




