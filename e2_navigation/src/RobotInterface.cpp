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

//=================================================================
// Class Constructor
//=================================================================
RobotInterface::RobotInterface(bool enable_neck,bool enable_voice,bool enable_train)
{
	last_speech_= ros::Time::now() - ros::Duration(SPEECH_DELAY);

	detected_user_.name = "";
	detected_user_.distance = 0;
	detected_user_.angle = 0;

	neck_enabled = enable_neck;
	voice_enabled = enable_voice;
	train_enabled = enable_train;

	kinectMotorFree_ = true;

	ac_mb = new MoveBaseClient("move_base", true);
	ac_fr = new FRClient("face_recognition", true);
	ac_nc = new NeckClient("e2_neck_controller",true);
	ac_vc = new VoiceClient("e2_voice_node",true);
	ac_kn = new KinectClient("kinect_motor", true);

	while (!ac_mb->waitForServer(ros::Duration(5.0)))
		ROS_INFO("[IRobot]:: Waiting for the move_base action server to come up");

	while (!ac_fr->waitForServer(ros::Duration(5.0)))
		ROS_INFO("[IRobot]:: Waiting for the face_recognition action server to come up");

	if(voice_enabled)
		while (!ac_vc->waitForServer(ros::Duration(5.0)))
			ROS_INFO("[IRobot]:: Waiting for the voice action server to come up");

	if(neck_enabled)
		while (!ac_nc->waitForServer(ros::Duration(5.0)))
			ROS_INFO("[IRobot]:: Waiting for the neck_controller action server to come up");

    while (!ac_kn->waitForServer(ros::Duration(5.0)))
                ROS_INFO("[IRobot]:: Waiting for the kinect action server to come up");

	ROS_INFO("[IRobot]:: Base ready");

}

RobotInterface::~RobotInterface()
{
 	cancell_all_goal();

	delete ac_mb;
	delete ac_fr;
	delete ac_nc;
	delete ac_vc;
	delete ac_kn;

	ROS_INFO("[IRobot]:: Base disabled");
}

//=================================================================
// Define new base goal
//=================================================================
void RobotInterface::base_setGoal(MBGoal goal)
{
	ROS_INFO("[IRobot]:: Received new goal: %f , %f , %f ",goal.target_pose.pose.position.x,goal.target_pose.pose.position.y,goal.target_pose.pose.position.z);
	ROS_INFO("[IRobot]:: Orientation z, w : %f , %f",goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w);
	ac_mb->sendGoal(goal);
}

//=================================================================
// Delete all robot goal
//=================================================================
void RobotInterface::cancell_all_goal()
{
	ac_mb->cancelAllGoals();
	ac_nc->cancelAllGoals();
	ac_fr->cancelAllGoals();
	ac_vc->cancelAllGoals();
	ac_kn->cancelAllGoals();

	ROS_DEBUG("[IRobot]:: All goal cancelled");
}

//=================================================================
// Get current goal status
//=================================================================
string RobotInterface::base_getStatus()
{
	// Check if the robot succeded it's task
	if (ac_mb->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("[IRobot::Base]:: Hooray, we reached the Goal ! ");
		return "SUCCEEDED";
	}
	else if(ac_mb->getState() == actionlib::SimpleClientGoalState::ABORTED)
	{
		ROS_INFO("[IRobot::Base]:: Move Base Aborted! ");
		return "ABORTED";
	}

	return "NULL";
}

//=================================================================
// Send messages to make the robot rotate on place. Allowed direction LEFT RIGHT
//=================================================================
void RobotInterface::base_rotate(char *direction,float angle)
{
	ROS_DEBUG("[IRobot::Base]:: Rotating %s of %f rad ",direction,angle);

	double th = tf::getYaw(robot_pose_.orientation);
	double th_new;

	if(strcmp(direction,"RIGHT") == 0)
		th_new = th - angle;
	else
		th_new = th + angle;

	current.target_pose.header.frame_id = "map";
	current.target_pose.pose = robot_pose_;
	current.target_pose.pose.orientation.z = sin(th_new/2);
	current.target_pose.pose.orientation.w = cos(th_new/2);

	ac_mb->sendGoal(current);

}

//=================================================================
// Stop the robot
//=================================================================
void RobotInterface::base_stop()
{
	ROS_DEBUG("[IRobot::Base]:: Robot Stopped ");
	cancell_all_goal();
}

//=================================================================
// Rotate kinect motor
//=================================================================
void RobotInterface::kinect_action(float angle)
{
	if(kinectMotorFree_)
	{
		kinectMotorFree_= false;
		ROS_INFO("[IRobot::Kinect]:: Motor rotate to %f deg",angle);
		kinect_motor::KinectGoal kinectGoal;

		kinectGoal.tilt = -100;

		ac_kn->sendGoal(kinectGoal, boost::bind(&RobotInterface::kinectDoneCallback, this, _1, _2));
		ac_kn->waitForResult(ros::Duration(5.0));

		kinectGoal.tilt = (int)angle;
		ac_kn->sendGoal(kinectGoal, boost::bind(&RobotInterface::kinectDoneCallback, this, _1, _2));

		ac_kn->waitForResult(ros::Duration(5.0));
	}
	else
	{
		ROS_INFO("[IRobot::Kinect]:: Device busy...");
	}

}

//=================================================================
//	Send an action to neck controller
//=================================================================
void RobotInterface::neck_action(int action,int sub_action)
{
	if(neck_enabled)
	{
		ROS_DEBUG("[IRobot::Neck]:: Received Neck action %d - %d",action,sub_action);

		NeckGoal n_goal;
		n_goal.action=action;
		n_goal.sub_action=sub_action;

		ac_nc->sendGoal(n_goal);
	}
	else
		ROS_INFO("[IRobot::Neck]:: Neck is not enabled. Action %d - %d Aborted",action,sub_action);
}

//=================================================================
// Return the current robot pose
//=================================================================
Pose RobotInterface::getRobotPose()
{
	ROS_DEBUG("[IRobot]:: Robot current pose x,y [%f,%f] ",robot_pose_.position.x,robot_pose_.position.y);
	return robot_pose_;
}

//=================================================================
// Update the robot position using data from odom topic
//=================================================================
void RobotInterface::setRobotPose(Pose pose)
{
	robot_pose_ = pose;
	ROS_DEBUG("[IRobot]:: Update robot pose [x,y][%f,%f] - [z,w][%f , %f]",robot_pose_.position.x,robot_pose_.position.y,robot_pose_.orientation.z,robot_pose_.orientation.w);
}

//=================================================================
// Make the robot talk
//=================================================================
void RobotInterface::robot_talk(string text, bool force)
{
	ros::Duration timeout(SPEECH_DELAY);

	if((ros::Time::now() - last_speech_ > timeout) || force )
	{

		if(force)
			ac_vc->waitForResult(timeout);

		if(voice_enabled)
		{
			last_speech_=ros::Time::now();
			neck_action(1,6); // Start Moving mouth

			VoiceGoal goal;
			goal.action_id = 1;
			goal.text = text;
			ac_vc->sendGoal(goal, boost::bind(&RobotInterface::voice_callback, this, _1, _2),VoiceClient::SimpleActiveCallback(), VoiceClient::SimpleFeedbackCallback());
		}
		else
			ROS_INFO("[IRobot]:: Robot can't talk. Enable voice support");

	}else
		ROS_DEBUG("[IRobot]:: Passed to few time");
}

//=====================================
// Voice Callback
//=====================================
void RobotInterface::voice_callback(const actionlib::SimpleClientGoalState& state, const VoiceResultConstPtr& result)
{
	ROS_DEBUG("[IRobot]:: Voice CallBack");
	neck_action(1,7); // stop Moving mouth
}

//=================================================================
// Get information about battery status
//=================================================================
char *RobotInterface::get_battery_status()
{
	// TODO - Read Battery status from motor board.
	return const_cast<char *>("GOOD");
}

//=================================================================
// Train user face to be used during backtrack
//=================================================================
bool RobotInterface::robot_train_user(string user_name)
{
	if(train_enabled)
	{
		//neck_action(2,6);
		kinect_action(10);// To correctly get user face

		FaceRecognitionGoal goal;

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

		//neck_action(2,1); // straight neck
		kinect_action(-5); //Back in orizontal position

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

	}else
		return true;	//train disabled

}

//=====================================
//	Send Request to facerecognition to check user
//=====================================
bool RobotInterface::robot_check_user(string user_name)
{
	ROS_INFO("[IRobot]:: Checking for known faces.");

	FaceRecognitionGoal goal;

    goal.order_id = 0;
    goal.order_argument = user_name;

    ac_fr->sendGoal(goal, boost::bind(&RobotInterface::facerecognition_callback, this, _1, _2),FRClient::SimpleActiveCallback(), FRClient::SimpleFeedbackCallback());
    ac_fr->waitForResult(ros::Duration(1.0));

    if(strcmp(detected_user_.name.c_str(),user_name.c_str())==0)
    	return true;

    return false;
}

//=====================================
// Face Recognition Callback
//=====================================
void RobotInterface::facerecognition_callback(const actionlib::SimpleClientGoalState& state, const FaceRecognitionResultConstPtr& result)
{
	clearDetectedUser();
	ROS_DEBUG("[IRobot]:: Goal [%i] Finished in state [%s]", result->order_id,state.toString().c_str());

	if(state.toString() != "SUCCEEDED")
	{
		ROS_DEBUG("[IRobot]:: Face Recognition  [%s]",state.toString().c_str());
		return;
	}

	if( result->order_id==0)
	{
		if(result->distance[0]/1000 < ERROR_DISTANCE)	//	If distance is greater than 15m there's something wrong
		{
			ROS_INFO("[IRobot]:: Detected User: %s at %f mm",result->names[0].c_str(),result->distance[0]);
			detected_user_.name = result->names[0];
			detected_user_.distance = result->distance[0]/1000; 				// convert mm from kinect to meters
			detected_user_.angle = result->angle[0];
		}else
			clearDetectedUser();
	}

	if( result->order_id==2)
		ROS_INFO("[IRobot]:: Pictures of %s were successfully added to the training images",result->names[0].c_str());

}

//=================================================================
// Getters and setters for detected user
//=================================================================
void RobotInterface::clearDetectedUser()
{
	detected_user_.angle=0.0;
	detected_user_.distance=0.0;
	detected_user_.name="";
}

t_user RobotInterface::getDetectedUser()
{
	return detected_user_;
}

void RobotInterface::setDetectedUser(t_user detectedUser)
{
	detected_user_ = detectedUser;
}

//======================================================================
//======================================================================

void RobotInterface::kinectDoneCallback(const actionlib::SimpleClientGoalState& state,const kinect_motor::KinectResultConstPtr& result)
{
	//ROS_INFO("[e2_brain]::Kinect motor set point reached [exit status %d]", (int)result->status);
	kinectMotorFree_ = true;
}

//======================================================================
//======================================================================
void RobotInterface::kinectActiveCallback(){}


//======================================================================
//======================================================================
void RobotInterface::kinectFeedbackCallback(const kinect_motor::KinectFeedbackConstPtr& feed){}

