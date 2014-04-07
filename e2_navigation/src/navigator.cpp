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
#include "e2_msgs/Train.h"
#include "e2_msgs/Talk.h"
#include "e2_msgs/NeckAction.h"
#include "std_srvs/Empty.h"
#include "nav_msgs/Odometry.h"

#define ROS_NODE_RATE	30
#define ROS_NODE_NAME	"e2_navigation"

Navigation *navigation;

void OdometryCb(const nav_msgs::Odometry::ConstPtr& msg);

// Define services
bool Abortcallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
bool Detectcallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
bool Gotocallback(e2_msgs::Goto::Request& request, e2_msgs::Goto::Response& response);
bool Startcallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
bool Neckcallback(e2_msgs::NeckAction::Request& request, e2_msgs::NeckAction::Response& response);
bool Traincallback(e2_msgs::Train::Request& request, e2_msgs::Train::Response& response);
bool Talkcallback(e2_msgs::Talk::Request& request, e2_msgs::Talk::Response& response);

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
	ros::ServiceServer detect_service = nh.advertiseService("detect",Detectcallback);
	ros::ServiceServer start_service = nh.advertiseService("start",Startcallback);
	ros::ServiceServer goto_service = nh.advertiseService("goto",Gotocallback);
	ros::ServiceServer neck_service = nh.advertiseService("neckaction",Neckcallback);
	ros::ServiceServer talk_service = nh.advertiseService("talk",Traincallback);
	ros::ServiceServer train_service = nh.advertiseService("train",Traincallback);

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
// This service launch a detection face
//=====================================
bool Detectcallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	navigation->DetectUser();
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

//=====================================
// Service to test neck actions
//=====================================
bool Neckcallback(e2_msgs::NeckAction::Request& request, e2_msgs::NeckAction::Response& response)
{
	navigation->irobot.NeckAction(request.action,request.sub_action);
	return true;
}

//=====================================
// Service to test robot voice
//=====================================
bool Talkcallback(e2_msgs::Talk::Request& request, e2_msgs::Talk::Response& response)
{
	if(strcmp(request.text.c_str(), "") == 0)
	{
		ROS_INFO("["ROS_NODE_NAME"]:: Empty string. Cant test voice. Abort");
		return false;
	}
	navigation->irobot.Talk(request.text);
	return true;
}

//=====================================
// Train callback
//=====================================
bool Traincallback(e2_msgs::Train::Request& request, e2_msgs::Train::Response& response)
{
	if(strcmp(request.username.c_str(), "") == 0)
	{
		ROS_INFO("["ROS_NODE_NAME"]:: Can't train without a username. Abort action.");
		return false;
	}

	navigation->irobot.Talk(navigation->getSpeechById("train"));
	if(navigation->irobot.TrainUserFace(request.username.c_str()))
	{
		navigation->irobot.Talk(navigation->getSpeechById("train_success"));
		return true;
	}
	navigation->irobot.Talk(navigation->getSpeechById("train_failed"));
	return false;
}
