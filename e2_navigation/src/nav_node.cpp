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
				as_.setAborted();
				break;
			case 1:	// Start Navigation for interested people
				ROS_DEBUG("["ROS_NODE_NAME"]:: Start Navigation Task");
				//nav->nav_clear();
				nav->nav_newTask();

				break;
			case 2:	// Aproach user
				ROS_DEBUG("["ROS_NODE_NAME"]:: Aproach user");
				//nav->nav_clear();
				nav->nav_goto(msg->distance,msg->angle);
				break;
			case 3: // Looking for user
				ROS_DEBUG("["ROS_NODE_NAME"]:: Looking for users");
				//nav->nav_clear();
				nav->nav_newLookingUser();
				break;
		}

		ros::Rate rate(ROS_NODE_RATE);

		// Start Navigation Controller
		while(!g_request_shutdown && !nav->nav_is_action_completed())
		{
			nav->controller();
			ros::spinOnce();
			rate.sleep();
		}
		nav->nav_clear();

		as_.setSucceeded();

		ROS_DEBUG("["ROS_NODE_NAME"]:: Action completed");
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

	// Start Navigation Controller
	while(!g_request_shutdown)
	{

		ros::spinOnce();
		r.sleep();
	}

	// Do pre-shutdown tasks
	ROS_INFO("["ROS_NODE_NAME"]:: Node Stopped. Kill everything");
	ros::shutdown();
}



