/*
 * navigator.cpp - AIRLab (Politecnico di Milano)
 * 
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 27/feb/2014
 *
 */
#include <signal.h>
#include "ros/ros.h"
#include "Navigation.h"
#include <ros/xmlrpc_manager.h>

#define ROS_NODE_RATE	30
#define ROS_NODE_NAME	"e2_navigation"

using namespace std;

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

//=====================================
// Main Code
//=====================================
int main(int argc, char **argv)
{
	ros::init(argc, argv, ROS_NODE_NAME,ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;
	ros::Rate r(ROS_NODE_RATE);

	// Override SIGINT handler
	signal(SIGINT, mySigIntHandler);

	// Override XMLRPC shutdown
	ros::XMLRPCManager::instance()->unbind("shutdown");
	ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

	ROS_INFO("["ROS_NODE_NAME"]:: Node Started");
	Navigation *nav = new Navigation(ros::this_node::getName(),ROS_NODE_RATE);

	//	Suscribers
	ros::Subscriber odom_sub= nh.subscribe("/odom", 10,&Navigation::odometry_callback,nav);

	// Enable Services
	ros::ServiceServer abort_service = nh.advertiseService(ROS_NODE_NAME"/abort",&Navigation::abort_callback,nav);
	ros::ServiceServer detect_service = nh.advertiseService(ROS_NODE_NAME"/detect",&Navigation::detect_callback,nav);
	ros::ServiceServer start_service = nh.advertiseService(ROS_NODE_NAME"/start",&Navigation::start_callback,nav);
	ros::ServiceServer goto_service = nh.advertiseService(ROS_NODE_NAME"/goto",&Navigation::goto_callback,nav);
	ros::ServiceServer neck_service = nh.advertiseService(ROS_NODE_NAME"/neckaction",&Navigation::neck_callback,nav);
	ros::ServiceServer talk_service = nh.advertiseService(ROS_NODE_NAME"/talk",&Navigation::talk_callback,nav);
	ros::ServiceServer train_service = nh.advertiseService(ROS_NODE_NAME"/train",&Navigation::train_callback,nav);
	ros::ServiceServer auto_service = nh.advertiseService(ROS_NODE_NAME"/auto",&Navigation::auto_engage_callback,nav);

	// Start Navigation Controller
	while(!g_request_shutdown)
	{
		nav->controller();
		ros::spinOnce();
		r.sleep();
	}

	// Do pre-shutdown tasks
	ROS_INFO("["ROS_NODE_NAME"]:: Node Stopped. Kill everything");
	delete nav;
	ros::shutdown();
}
