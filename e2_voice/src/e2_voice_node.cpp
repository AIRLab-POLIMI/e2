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

#include <e2_voice/VoiceAction.h>
#include <e2_voice/VoiceGoal.h>
#include <e2_voice/VoiceResult.h>
#include <e2_voice/VoiceFeedback.h>
#include <actionlib/server/simple_action_server.h>

#define ROS_NODE_RATE	10
#define ROS_NODE_NAME	"e2_voice_node"

//#define SPEECH_COMMAND "espeak"
//#define SPEECH_PARAM " -s 150 -v it "
//#define SPEECH_OPT "--stdout | paplay" 		// Fix bug with alsa and pulseaudio

using namespace std;

class Voice {

public:
	Voice(string name):
		 nh_("~"),
		 as_(nh_, name, boost::bind(&Voice::executeCB, this, _1), false)
	{
		goal_id_ = -99;
		as_.start();					//starting the actionlib server

		ROS_INFO("["ROS_NODE_NAME"]:: Node ready");
	}

	void executeCB(const e2_voice::VoiceGoalConstPtr& msg)
	{
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
			ROS_INFO("["ROS_NODE_NAME"]:: %d is not a valid action. Abort",goal_id_);
			as_.setPreempted();
			return;
		}

		ROS_INFO("["ROS_NODE_NAME"]:: Received action %d ",goal_id_);

			switch(goal_id_)
			{
				case 0:
					ROS_INFO("["ROS_NODE_NAME"]:: Abort current action");
					as_.setAborted();
					break;
				case 1:
					ROS_INFO("["ROS_NODE_NAME"]:: Received text %s ",msg->text.c_str());
					//string command=SPEECH_COMMAND" "SPEECH_PARAM" '"+msg->text+"' "SPEECH_OPT ;
					string command="pico2wave -l it-IT -w /tmp/e2.wav  '"+msg->text+"' && play /tmp/e2.wav pitch -500 treble 10 4.0k  loudness 5 >/dev/null 2>&1";
					system(command.c_str());
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
