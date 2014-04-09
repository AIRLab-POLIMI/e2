/*
 * e2_voice_client.cpp
 * 
 * Client to test Voice Module
 *
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 09/apr/2014
 *
 */

#include "ros/ros.h"
#include "e2_voice/VoiceGoal.h"
#include "e2_voice/VoiceAction.h"
#include <actionlib/client/simple_action_client.h>

#define ROS_NODE_NAME	"e2_voice_client"

typedef actionlib::SimpleActionClient<e2_voice::VoiceAction> VoiceClient;

VoiceClient *ac_vc;
e2_voice::VoiceGoal goal;

//=========================================================
//	Callback function to send goal to neck controller
//=========================================================
void voiceCallback(const e2_voice::VoiceGoalConstPtr& msg)
{
     ROS_INFO("["ROS_NODE_NAME"]:: Request for sending goal [%d] received", msg->action_id);

     goal.action_id = msg->action_id;
     goal.text = msg->text;
     ac_vc->sendGoal(goal);
}

//=========================================================
//	Main Code
//=========================================================
int main (int argc, char **argv)
{
	ros::init(argc, argv, ROS_NODE_NAME);
	ROS_INFO("["ROS_NODE_NAME"]:: Node Started");

	ros::NodeHandle n;
	ac_vc = new VoiceClient("e2_voice_node",true);

	while (!ac_vc->waitForServer(ros::Duration(1.0)))
		ROS_INFO("["ROS_NODE_NAME"]:: Waiting for the voice action server to come up");

	//subscribe to the topic of interest
	ros::Subscriber sub = n.subscribe("voice_action", 1, voiceCallback);

	ros::spin();
	return 0;
}
