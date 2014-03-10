/*
 * NeckInterface.cpp - AIRLab (Politecnico di Milano)
 * 
 * description
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
NeckInterface::NeckInterface(string name) : nh_(),as_(nh_, name, boost::bind(&NeckInterface::executeCB, this, _1), false)
{
	goal_id_ = -99;
	as_.start();			//starting the actionlib server

	ROS_INFO("[INeck]:: Neck interface ready");

}

NeckInterface::~NeckInterface()
{
	ROS_INFO("[INeck]:: Neck interface ended");
}

// ========================================================================
//
// ========================================================================
void NeckInterface::executeCB(const e2_neck_controller::NeckGoalConstPtr &goal)
{
	// Check if current goal is still active
	if (as_.isPreemptRequested())
	{
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

	}

}
