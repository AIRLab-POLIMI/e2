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
#include "e2_msgs/Goto.h"
#include "std_srvs/Empty.h"
#include "nav_msgs/Odometry.h"

#define ROS_NODE_RATE	1
#define ROS_NODE_NAME	"e2_navigation"

Navigation *navigation;

void OdometryCb(const nav_msgs::Odometry::ConstPtr& msg);
bool Abortcallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
bool Startcallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
bool Gotocallback(e2_msgs::Goto::Request& request, e2_msgs::Goto::Response& response);

using namespace std;

//=====================================
// Main Code
//=====================================
int main(int argc, char **argv)
{
	ros::init(argc, argv, ROS_NODE_NAME);
	ros::NodeHandle nh("~");

    string marker_config,speech_config;
    bool en_neck,en_voice,en_train;

	nh.param<bool>("en_neck", en_neck, true);
	nh.param<bool>("en_voice", en_voice, true);
	nh.param<bool>("en_train", en_train, true);

	nh.param("marker_config", marker_config, ros::package::getPath("e2_navigation")+"/config/marker_config.yaml");
	nh.param("speech_config", speech_config, ros::package::getPath("e2_navigation")+"/config/speech_config.yaml");

	ros::ServiceServer abort_service = nh.advertiseService("abort",Abortcallback);
	ros::ServiceServer start_service = nh.advertiseService("start",Startcallback);
	ros::ServiceServer goto_service = nh.advertiseService("goto",Gotocallback);

	// Suscribers && Publishers for input messages
    ros::Subscriber odom_sub= nh.subscribe("/odom", 10,OdometryCb);

	ROS_INFO("["ROS_NODE_NAME"]:: Node Started");

	navigation = new Navigation(&nh,marker_config,speech_config,ROS_NODE_RATE,en_neck,en_voice,en_train);
	navigation->Controller();

}

//=====================================
// Odometry callback for robot pose update
//=====================================
void OdometryCb(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_DEBUG("[Odometry]:: Odometry pose x,y,z: [%f,%f,%f]: ", msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
	navigation->UpdateRobotPose(msg->pose.pose);
}

//=====================================
// Kill everything and shutdown
//=====================================
bool Abortcallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	navigation->AbortTask();
	return true;
}

//=====================================
// Start new navigation task
//=====================================
bool Startcallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	navigation->NewTask();
	return true;
}

//=====================================
// Navigate to known location
//=====================================
bool Gotocallback(e2_msgs::Goto::Request& request, e2_msgs::Goto::Response& response)
{
	if(navigation->MarkerExist(request.location))
	{
		navigation->NavigateTo(request.location);
		response.result = true;
		return true;
	}
	response.result = false;
	return false;
}
