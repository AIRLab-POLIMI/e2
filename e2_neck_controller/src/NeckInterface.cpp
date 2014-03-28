/*
 * NeckInterface.cpp - AIRLab (Politecnico di Milano)
 * 
 * This node control the movement for the E-two robot neck
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

	ROS_INFO("[INeck]:: Using %s device.",device.c_str());

	pE2PololuInterface = new E2_Pololu_Interface(device);

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
	goal_id_ = goal->action_id ;

	//check if the name of the person has been provided for the add-face-images goal
	if (goal_id_ > 10 || goal_id_ < 0 )
	{
		ROS_INFO("[INeck]:: %d is not a valid action. Abort",goal_id_);
		as_.setPreempted();
		return;
	}
	ROS_INFO("[INeck]:: Received action %d ",goal_id_);

	switch(goal_id_)
	{
		case '1':
			ROS_INFO("[INeck]:: Reach Straight Neck Position");
			pE2PololuInterface->reachStraightNeckPosition();
			break;
		case '2':
			ROS_INFO("[INeck]:: Invitation Left");
			pE2PololuInterface->invitationLeft();
			break;
		case '3':
			ROS_INFO("[INeck]:: Invitation Right");
			pE2PololuInterface->invitationRight();
			break;
		case '4':
			ROS_INFO("[INeck]:: Give a Bow");
			pE2PololuInterface->give_a_bow();
			break;
		case '5':
			ROS_INFO("[INeck]:: Surprise Expression");
			pE2PololuInterface->expressSurprise();
			break;
		case '6':
			ROS_INFO("[INeck]:: Bend Forward");
			pE2PololuInterface->bendForward();
			break;
		case '7':
			ROS_INFO("[INeck]:: Bend Back");
			pE2PololuInterface->bendBack();
			break;
		case '8':
			ROS_INFO("[INeck]:: Bend Left");
			pE2PololuInterface->bendLeft();
			break;
		case '9':
			ROS_INFO("[INeck]:: Bend Right");
			pE2PololuInterface->bendRight();
			break;
	}

	ROS_INFO("[INeck]:: Action completed");

	as_.setSucceeded();

}
