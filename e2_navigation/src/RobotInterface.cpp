/*
 * RobotInterface.cpp - AIRLab (Politecnico di Milano)
 * 
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 27/feb/2014
 *
 */

#include "RobotInterface.h"

using namespace std;

//=================================================================
// Class Constructor
//=================================================================
RobotInterface::RobotInterface(bool enable_neck,bool enable_voice,bool enable_train)
{
	recognized_user = "none";
	neck_enabled = enable_neck;
	voice_enabled = enable_voice;								// Voice enabled
	train_enabled = enable_train;								// Force train new face before start navigation task

	ac_mb = new MoveBaseClient("move_base", true);
	ac_fr = new FRClient("face_recognition", true);
	ac_nc = new NeckClient("e2_neck_controller",true);

	while (!ac_mb->waitForServer(ros::Duration(5.0)))
		ROS_INFO("[IRobot]:: Waiting for the move_base action server to come up");

	while (!ac_fr->waitForServer(ros::Duration(5.0)))
		ROS_INFO("[IRobot]:: Waiting for the face_recognition action server to come up");

	ros::Time init_time = ros::Time::now();
	while (!ac_nc->waitForServer(ros::Duration(1.0)) && neck_enabled)
	{
		ROS_INFO("[IRobot]:: Waiting for the neck_controller action server to come up");
		double elapsed = (ros::Time::now() - init_time).toSec();
		if(elapsed > 5.0)
		{
			ROS_INFO("[IRobot]:: Neck connection problem. Check if neck is connected. Disabled");
			neck_enabled=false;
		}
	}
	ROS_INFO("[IRobot]:: Base ready");
}

RobotInterface::~RobotInterface()
{
	ac_fr = NULL;
	ac_nc = NULL;
	ac_mb = NULL;

	ROS_INFO("[IRobot]:: Base disabled");
}

//=================================================================
// Define new base goal
//=================================================================
void RobotInterface::setGoal(MBGoal goal)
{
	ROS_INFO	("[IRobot]:: Received new goal: %f , %f , %f ",goal.target_pose.pose.position.x,goal.target_pose.pose.position.y,goal.target_pose.pose.position.z);
	ROS_INFO	("[IRobot]:: Orientation z, w : %f , %f",goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w);
	ac_mb->sendGoal(goal);
}

//=================================================================
// Delete all robot goal
//=================================================================
void RobotInterface::CancelAllGoals()
{
	ROS_DEBUG("[IRobot]:: All goal cancelled ");
	ac_mb->cancelAllGoals();
	ac_nc->cancelAllGoals();
	ac_fr->cancelAllGoals();
}

//=================================================================
// Get current goal status
//=================================================================
bool RobotInterface::getBaseGoalStatus()
{
	// Check if the robot succeded it's task
	if (ac_mb->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("[IRobot]:: Hooray, we reached the Goal ! ");
		return true;
	}
	return false;
}

//=================================================================
// Send messages to make the robot rotate on place. Allowed direction LEFT RIGHT
//=================================================================
void RobotInterface::RotateBase(char *direction,float angle)
{
	ROS_INFO("[IRobot]:: Rotating %s of %f rad ",direction,angle);

	double th = tf::getYaw(robot_pose.orientation);
	double th_new;

	if(strcmp(direction,"RIGHT") == 0)
		th_new = th - angle;
	else
		th_new = th + angle;

	current.target_pose.header.frame_id = "map";
	current.target_pose.pose = robot_pose;
	current.target_pose.pose.orientation.z = sin(th_new/2);
	current.target_pose.pose.orientation.w = cos(th_new/2);

	ac_mb->sendGoal(current);

}

//=================================================================
// Stop the robot
//=================================================================
void RobotInterface::StopBase()
{
	ROS_DEBUG("[IRobot::Base]:: Robot Stopped ");
	CancelAllGoals();
}

//=================================================================
//	Send an action to neck controller
//=================================================================
void RobotInterface::NeckAction(int action,int sub_action)
{
	if(neck_enabled)
	{
		ROS_INFO("[IRobot::Neck]:: Received Neck action %d - %d",action,sub_action);

		e2_neck_controller::NeckGoal n_goal;
		n_goal.action=action;
		n_goal.sub_action=sub_action;

		ac_nc->sendGoal(n_goal);
	}
	else
		ROS_INFO("[IRobot::Neck]:: Neck is not enabled. No action taken. ");
}

//=================================================================
// Return the current robot pose
//=================================================================
geometry_msgs::Pose RobotInterface::getRobotPose()
{
	ROS_DEBUG("[IRobot]:: Robot current pose x,y [%f,%f] ",robot_pose.position.x,robot_pose.position.y);
	return robot_pose;
}

//=================================================================
// Update the robot position using data from odom topic
//=================================================================
void RobotInterface::setRobotPose(geometry_msgs::Pose pose)
{
	robot_pose = pose;
	ROS_DEBUG("[IRobot]:: Update robot pose [x,y][%f,%f] - [z][%f]",robot_pose.position.x,robot_pose.position.y,robot_pose.orientation.z);
}

//=================================================================
// Make the robot talk
//=================================================================
void RobotInterface::Talk(string text)
{
	if(voice_enabled)
	{
		NeckAction(1,6); // Start Moving mouth

		string command=SPEECH_COMMAND" "SPEECH_PARAM" '"+text+"' "SPEECH_OPT ;	// TODO - Use Svox (pico) when male voice will be added - Better voice
		ROS_INFO("[IRobot]:: Robot say: %s",text.c_str());
		system(command.c_str());

		NeckAction(1,7); // stop Moving mouth
	}
	else
		ROS_INFO("[IRobot]:: Robot can't talk. Enable voice support");
}

//=================================================================
// Get information about battery status
//=================================================================
char *RobotInterface::getBatteryStatus()
{
	// TODO - Read Battery status motor board.
	return const_cast<char *>("GOOD");
}

//=================================================================
// Train user face to be used during backtrack
//=================================================================
bool RobotInterface::TrainUserFace(string user_name)
{
	face_recognition::FaceRecognitionGoal goal;

	ac_fr->waitForServer();

	// Clean old face data
	goal.order_id = 5;
	goal.order_argument = user_name;

	ac_fr->sendGoal(goal);
	ac_fr->waitForResult(ros::Duration(5.0));

	goal.order_id = 2;
	goal.order_argument = user_name;

	ac_fr->sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = ac_fr->waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
		ROS_INFO("[IRobot]:: New face saved.");

		goal.order_id = 3;
		goal.order_argument = user_name;

		ac_fr->sendGoal(goal);
		ac_fr->waitForResult(ros::Duration(10.0));

		ROS_INFO("[IRobot]:: Face Database Updated.");
		return true;
	}
	else
	{
		ROS_INFO("[IRobot]:: Problem saving new face...timeout occurred.");
		return false;
	}

	return false;

}

//=====================================
//	Send Request to facerecognition to check user
//=====================================
bool RobotInterface::CheckFace(string guest_user)
{
	ROS_INFO("[IRobot]:: Checking for known faces.");

	face_recognition::FaceRecognitionGoal goal;

    goal.order_id = 0;
    goal.order_argument = guest_user;

    ac_fr->sendGoal(goal, boost::bind(&RobotInterface::FaceRecognCb, this, _1, _2),FRClient::SimpleActiveCallback(), FRClient::SimpleFeedbackCallback());
    ac_fr->waitForResult(ros::Duration(2.0));

    if(strcmp(recognized_user.c_str(),guest_user.c_str())==0)
    	return true;

    return false;
}

//=====================================
// Face Recognition Callback
//=====================================
void RobotInterface::FaceRecognCb(const actionlib::SimpleClientGoalState& state, const face_recognition::FaceRecognitionResultConstPtr& result)
{
	recognized_user = "none";
	ROS_DEBUG("[IRobot]:: Goal [%i] Finished in state [%s]", result->order_id,state.toString().c_str());

	if(state.toString() != "SUCCEEDED") return;

	if( result->order_id==0)
	{
		ROS_INFO("[IRobot]:: Detected User: %s",result->names[0].c_str());
		recognized_user = result->names[0];
	}

	if( result->order_id==2)
		ROS_INFO("[IRobot]:: Pictures of %s were successfully added to the training images",result->names[0].c_str());

}
