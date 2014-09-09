/*
 * e2_voice_node.cpp
 * 
 * Node to make the robot speak
 *
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 09/apr/2014
 *
 */
#include "ros/ros.h"
#include "ros/package.h"

#include <e2_voice/VoiceAction.h>
#include <e2_voice/VoiceGoal.h>
#include <e2_voice/VoiceResult.h>
#include <e2_voice/VoiceFeedback.h>
#include <e2_neck_controller/NeckAction.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#define ROS_NODE_RATE	5
#define ROS_NODE_NAME	"e2_voice_node"

using namespace std;
using namespace e2_neck_controller;

typedef actionlib::SimpleActionClient<NeckAction> NeckClient;

class Voice {

public:
	Voice(string name):
		 nh_("~"),
		 as_(nh_, name, boost::bind(&Voice::executeCB, this, _1), false)
	{
		goal_id_ = -99;
		as_.start();					//starting the actionlib server

		ac_nc = new NeckClient("e2_neck_controller",true);

		while (!ac_nc->waitForServer(ros::Duration(5.0)))
			ROS_INFO("["ROS_NODE_NAME"]:: Waiting for the neck_controller action server to come up");

		ROS_INFO("["ROS_NODE_NAME"]:: Node ready");
	}

	void executeCB(const e2_voice::VoiceGoalConstPtr& msg)
	{
		NeckGoal n_goal;
		// Check if current goal is still active
		if (as_.isPreemptRequested())
		{
			ROS_INFO("["ROS_NODE_NAME"]:: There's an active action. Abort");
			as_.setPreempted();
			return;
		}
		goal_id_ = msg->action_id ;

		//check if the name of the person has been provided for the add-face-images goal
		if (goal_id_ >= 2 || goal_id_ < 0 )
		{
			ROS_DEBUG("["ROS_NODE_NAME"]:: %d is not a valid action. Abort",goal_id_);
			as_.setPreempted();
			return;
		}

		ROS_INFO("["ROS_NODE_NAME"]:: Received action %d ",goal_id_);

		switch(goal_id_)
		{
			case 0:
				if(as_.isActive())
				{
					ROS_INFO("["ROS_NODE_NAME"]:: Abort current action");
					as_.setAborted();
					return;
				}
				break;
			case 1:
				// Add face expression
				if(msg->face_action != 0 )
				{
					n_goal.action=1;
					n_goal.sub_action=msg->face_action;
					ac_nc->sendGoal(n_goal);
					ac_nc->waitForResult();
				}
				// Add neck action
				if(msg->neck_action != 0)
				{
					n_goal.action=2;
					n_goal.sub_action=msg->neck_action;
					ac_nc->sendGoal(n_goal);
					ac_nc->waitForResult();
				}

				// Start Moving mouth
				n_goal.action=1;
				n_goal.sub_action=6;
				ac_nc->sendGoal(n_goal);

				// Make robot talk
				if(strcmp(msg->text.c_str(),"robin") == 0)
				{
					// So what ? Can't add a joke ?
					ROS_INFO("["ROS_NODE_NAME"]:: Urca Urca ");
					string command="mplayer "+ros::package::getPath("e2_config")+"/speak_config/robin.mp3";
					system(command.c_str());
				}
				else
				{
					ROS_INFO("["ROS_NODE_NAME"]:: %s ",msg->text.c_str());
					string command="pico2wave -l it-IT -w /tmp/e2.wav  '"+msg->text+"' && play /tmp/e2.wav  pitch -190 stretch 0.9 band 3000 500 treble 10 >/dev/null 2>&1";
					system(command.c_str());
				}
				// Back to normal face
				if(msg->face_action != 0 )
				{
					n_goal.action=1;
					n_goal.sub_action=1;
					ac_nc->sendGoal(n_goal);
					ac_nc->waitForResult();
				}
				// Back to straight neck position
				if(msg->neck_action != 0)
				{
					n_goal.action=2;
					n_goal.sub_action=1;
					ac_nc->sendGoal(n_goal);
					ac_nc->waitForResult();
				}
				// Stop Moving mouth
				n_goal.action=1;
				n_goal.sub_action=7;
				ac_nc->sendGoal(n_goal);

				break;
		}



		as_.setSucceeded();
		ROS_INFO("["ROS_NODE_NAME"]:: Action completed");
	}

private:
	int goal_id_;
	ros::NodeHandle nh_;
	e2_voice::VoiceResult result_;
	actionlib::SimpleActionServer<e2_voice::VoiceAction> as_;

	NeckClient *ac_nc;
};

//==========================================
//	Main Code
//==========================================
int main(int argc, char **argv)
{
	  ros::init(argc, argv, ROS_NODE_NAME);
	  ros::NodeHandle nh;

	  Voice voice(ros::this_node::getName());
	  ros::Rate r(ROS_NODE_RATE);

	  ros::spin();
	  return 0;
}
