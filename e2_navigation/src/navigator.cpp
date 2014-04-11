/*
 * navigator.cpp - AIRLab (Politecnico di Milano)
 * 
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 27/feb/2014
 *
 */

#include "ros/ros.h"
#include "Navigation.h"


#define ROS_NODE_RATE	30
#define ROS_NODE_NAME	"e2_navigation"

using namespace std;

//=====================================
// Main Code
//=====================================
int main(int argc, char **argv)
{
	ros::init(argc, argv, ROS_NODE_NAME);
	ros::NodeHandle nh;
	ros::Rate r(ROS_NODE_RATE);

	ROS_INFO("["ROS_NODE_NAME"]:: Node Started");
	Navigation *nav = new Navigation(ros::this_node::getName(),ROS_NODE_RATE);

	//	Suscribers
	ros::Subscriber map_sub = nh.subscribe("/map", 1, &Navigation::map_callback, nav);
	ros::Subscriber odom_sub= nh.subscribe("/odom", 10,&Navigation::odometry_callback,nav);
	// Enable Services
	ros::ServiceServer abort_service = nh.advertiseService(ROS_NODE_NAME"/abort",&Navigation::abort_callback,nav);
	ros::ServiceServer detect_service = nh.advertiseService(ROS_NODE_NAME"/detect",&Navigation::detect_callback,nav);
	ros::ServiceServer start_service = nh.advertiseService(ROS_NODE_NAME"/start",&Navigation::start_callback,nav);
	ros::ServiceServer goto_service = nh.advertiseService(ROS_NODE_NAME"/goto",&Navigation::goto_callback,nav);
	ros::ServiceServer neck_service = nh.advertiseService(ROS_NODE_NAME"/neckaction",&Navigation::neck_callback,nav);
	ros::ServiceServer talk_service = nh.advertiseService(ROS_NODE_NAME"/talk",&Navigation::talk_callback,nav);
	ros::ServiceServer train_service = nh.advertiseService(ROS_NODE_NAME"/train",&Navigation::train_callback,nav);

	// Start Navigation Controller
	while(ros::ok)
	{
		nav->controller();
		ros::spinOnce();
		r.sleep();
	}

	// Do pre-shutdown tasks
	ROS_INFO("["ROS_NODE_NAME"]:: Node Stopped. Kill everything");
	delete nav;

}
