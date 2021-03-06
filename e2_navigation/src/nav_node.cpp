/*
 * nav_node.cpp - AIRLab (Politecnico di Milano)
 *
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 26/mag/2014
 *
 */

#include "ros/ros.h"

#include "Navigation.h"

#include <e2_navigation/NavAction.h>
#include <e2_navigation/NavGoal.h>
#include <e2_navigation/NavResult.h>
#include <e2_navigation/NavFeedback.h>

#include <actionlib/server/simple_action_server.h>

#include <signal.h>
#include <ros/xmlrpc_manager.h>

#define ROS_NODE_RATE	5
#define ROS_NODE_NAME	"e2_nav"

using namespace std;

Navigation *nav;

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
	g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
	int num_params = 0;
	if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
		num_params = params.size();
	if (num_params > 1)
	{
		std::string reason = params[1];
		ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
		g_request_shutdown = 1; // Set flag
	}

	result = ros::xmlrpc::responseInt(1, "", 0);
}

class Nav {

public:
	Nav(string name):
		 nh(),
		 as_(nh, name, boost::bind(&Nav::executeCB, this, _1), false)
	{
		goal_id_ = -99;

		nav = new Navigation(ros::this_node::getName(),ROS_NODE_RATE);

		as_.start();					//starting the actionlib server

		ROS_INFO("["ROS_NODE_NAME"]:: Node ready");
	}

	void executeCB(const e2_navigation::NavGoalConstPtr& msg)
	{
		ROS_INFO("---------------------------------------");
		ROS_INFO("---------------------------------------");
		ROS_INFO("---------------------------------------");
		// Check if current goal is still active
		if (as_.isPreemptRequested())
		{
			ROS_INFO("["ROS_NODE_NAME"]:: There's an active action. Abort");
			as_.setPreempted();
			return;
		}
		goal_id_ = msg->action_id;

		ROS_DEBUG("["ROS_NODE_NAME"]:: Received action %d ",goal_id_);

		switch(goal_id_)
		{
			case 0:	// Abort
				ROS_DEBUG("["ROS_NODE_NAME"]:: Abort current action");
				nav->ActionReset();
				as_.setAborted();
				return;
				break;
			case 1:	// Looking for user
				ROS_DEBUG("["ROS_NODE_NAME"]:: Looking for users");
				nav->LookingUser();
				break;
			case 2:	// Aproach user
				ROS_DEBUG("["ROS_NODE_NAME"]:: Aproaching user");
				nav->ApproachUser();
				break;
			case 3: // Start Navigation for interested people
				ROS_DEBUG("["ROS_NODE_NAME"]:: Start navigation to target");
				nav->NavigateTarget();
				break;
		}

		ros::Rate rate(ROS_NODE_RATE);

		// Start Navigation Controller
		while(!g_request_shutdown && !nav->isActionCompleted() && !nav->isActionAborted())
		{
			nav->ActionController();
			ros::spinOnce();
			rate.sleep();
		}

		result_.action_id = goal_id_;

		if(nav->isActionCompleted())
			as_.setSucceeded(result_,"OK");
		else
			as_.setAborted(result_,"FAILED");

		nav->ActionReset();

	}

private:
	int goal_id_;
	ros::NodeHandle nh;

	e2_navigation::NavResult result_;
	actionlib::SimpleActionServer<e2_navigation::NavAction> as_;

};

//==========================================
//	Main Code
//==========================================
int main(int argc, char **argv)
{
	ros::init(argc, argv, ROS_NODE_NAME,ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;

	// Override SIGINT handler
	signal(SIGINT, mySigIntHandler);

	// Override XMLRPC shutdown
	ros::XMLRPCManager::instance()->unbind("shutdown");
	ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

	ros::Rate r(ROS_NODE_RATE);

	Nav navigation(ROS_NODE_NAME);

	// Start Navigation Controller
	while(!g_request_shutdown)
	{
		//if(nav->isActionCompleted())
		//	 nav->ActionReset();

		//nav->ActionController();

		ros::spinOnce();
		r.sleep();
	}

	// Do pre-shutdown tasks
	ROS_INFO("["ROS_NODE_NAME"]:: Node Stopped. Kill everything");
	ros::shutdown();
}



