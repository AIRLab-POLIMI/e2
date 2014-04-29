/*
 * NeckInterface.cpp - AIRLab (Politecnico di Milano)
 * 
 * This node control the movement of E-two robot neck
 *
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 10/mar/2014
 *
 */

#include "NeckInterface.h"

// ========================================================================
// Default constructor
// ========================================================================
NeckInterface::NeckInterface(string name) : nh_("~"),as_(nh_, name, boost::bind(&NeckInterface::executeCB, this, _1), false)
{
	goal_id_ = -99;
	as_.start();																//starting the actionlib server

	std::string device;
	nh_.param<std::string>("usb_device", device, "/dev/ttyACM0");
	pE2PololuInterface = new E2_Pololu_Interface(device);

	ROS_INFO("[INeck]:: Using %s device.",device.c_str());
	ROS_INFO("[INeck]:: Neck interface ready");

}

NeckInterface::~NeckInterface()
{
	delete pE2PololuInterface;
	ROS_INFO("[INeck]:: Neck interface ended");
}

// ========================================================================
//	Execute action using actionlib
// ========================================================================
void NeckInterface::executeCB(const e2_neck_controller::NeckGoalConstPtr &goal)
{
	// Check if current goal is still active
	if (as_.isPreemptRequested())
	{
		ROS_INFO("[INeck]:: There's an active action. Abort");
		as_.setPreempted();
		return;
	}
	goal_id_ = goal->action ;

	//check if the name of the person has been provided for the add-face-images goal
	if (goal_id_ > 10 || goal_id_ < 0 )
	{
		ROS_INFO("[INeck]:: %d is not a valid action. Abort",goal_id_);
		as_.setPreempted();
		return;
	}
	ROS_DEBUG("[INeck]:: Received action %d ",goal_id_);

	switch(goal_id_)
	{
		case 1:
			ROS_DEBUG("[INeck]:: Face Action ");
			face_actions(goal->sub_action);
			break;
		case 2:
			ROS_DEBUG("[INeck]:: Neck Action");
			neck_actions(goal->sub_action);
			break;
		case 3:
			ROS_DEBUG("[INeck]:: Composite Action");
			composite_actions(goal->sub_action);
			break;
		case 4:
			ROS_DEBUG("[INeck]:: Other Actions");
			other_actions(goal->sub_action);
			break;
	}

	as_.setSucceeded();
	ROS_INFO("[INeck]:: Action completed");

}

// ========================================================================
//	Face action chooser
// ========================================================================
void NeckInterface::face_actions(int action)
{
	switch(action)
	{
		case 1:
			ROS_INFO("[INeck::Face]:: Standard Face");
			pE2PololuInterface->standardFace();
			break;
		case 2:
			ROS_INFO("[INeck::Face]:: Happy Face");
			pE2PololuInterface->happyFace();
			break;
		case 3:
			ROS_INFO("[INeck::Face]:: Angry Face");
			pE2PololuInterface->angryFace();
			break;
		case 4:
			ROS_INFO("[INeck::Face]:: Interested Face");
			pE2PololuInterface->interestedFace();
			break;
		case 5:
			ROS_INFO("[INeck::Face]:: Invitation Face");
			pE2PololuInterface->invitationFace();
			break;
		case 6:
			ROS_INFO("[INeck::Face]:: Start Speaking");
			pE2PololuInterface->start_speakingFace();
			break;
		case 7:
			ROS_INFO("[INeck::Face]:: Stop Speaking");
			pE2PololuInterface->stop_speakingFace();
			break;
	}

}

// ========================================================================
//	Neck action chooser
// ========================================================================
void NeckInterface::neck_actions(int action)
{
	switch(action)
	{
		case 1:
			ROS_INFO("[INeck::Neck]:: Straight Neck");
			pE2PololuInterface->straightNeck();
			break;
		case 2:
			ROS_INFO("[INeck::Neck]:: Invitation Left");
			pE2PololuInterface->invitationLeft();
			break;
		case 3:
			ROS_INFO("[INeck::Neck]:: Invitation Right");
			pE2PololuInterface->invitationRight();
			break;
		case 4:
			ROS_INFO("[INeck::Neck]:: Give a Bow");
			pE2PololuInterface->give_a_bow();
			break;
		case 5:
			ROS_INFO("[INeck::Neck]:: Bend Forward");
			pE2PololuInterface->bendForward();
			break;
		case 6:
			ROS_INFO("[INeck::Neck]:: Bend Back");
			pE2PololuInterface->bendBack();
			break;
		case 7:
			ROS_INFO("[INeck::Neck]:: Bend Left");
			pE2PololuInterface->bendLeft();
			break;
		case 8:
			ROS_INFO("[INeck::Neck]:: Bend Right");
			pE2PololuInterface->bendRight();
			break;
	}

}

// ========================================================================
//	Composite action chooser
// ========================================================================
void NeckInterface::composite_actions(int action)
{
	switch(action)
	{
		case 1:
			ROS_INFO("[INeck::Composite]:: Express Surprise");
			pE2PololuInterface->expressSurprise();
			break;
		case 2:
			ROS_INFO("[INeck::Composite]:: Reach straight position");
			pE2PololuInterface->reachStraightPosition();
			break;
	}
}

// ========================================================================
//	Other action chooser
// ========================================================================
void NeckInterface::other_actions(int action)
{
	switch(action)
	{
		case 1:
			ROS_INFO("[INeck::Other]:: Chiudi comunicazione pololu....toglie tensione motori !");
			pE2PololuInterface->stopScriptAndGoHome();
			delete pE2PololuInterface;
			break;
	}
}
