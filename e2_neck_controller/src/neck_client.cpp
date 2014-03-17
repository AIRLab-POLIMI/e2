
/*
 * neck_client.cpp - AIRLab (Politecnico di Milano)
 * 
 * Client to test neck capabilities
 *
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 17/mar/2014
 *
 */

#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include "e2_neck_controller/NeckGoal.h"
#include "e2_neck_controller/NeckResult.h"
#include "e2_neck_controller/NeckAction.h"
#include "e2_neck_controller/NeckActionFeedback.h"

#define ROS_NODE_NAME	"neck_client"

typedef actionlib::SimpleActionClient<e2_neck_controller::NeckAction> NeckClient;
e2_neck_controller::NeckGoal goal;

NeckClient *ac_nc;

void nclientCallback(const e2_neck_controller::NeckGoalPtr& msg);

int main (int argc, char **argv)
{
	ros::init(argc, argv, ROS_NODE_NAME);

	ROS_INFO("["ROS_NODE_NAME"]:: Node Started");

	ros::NodeHandle n;

	ac_nc = new NeckClient("e2_neck_controller",true);

	while (!ac_nc->waitForServer(ros::Duration(1.0)))
		ROS_INFO("["ROS_NODE_NAME"]:: Waiting for the neck_controller action server to come up");

	//subscribe to the topic of interest
	ros::Subscriber sub = n.subscribe("neck_action", 1, nclientCallback);

	ros::spin();
	return 0;
}

//=========================================================
//	Callback function to send goal to neck controller
//=========================================================
void nclientCallback(const e2_neck_controller::NeckGoalPtr& msg)
{
     ROS_INFO("["ROS_NODE_NAME"]:: Request for sending goal [%d]  received", msg->action_id);

     goal.action_id = msg->action_id;
     ac_nc->sendGoal(goal);
}
