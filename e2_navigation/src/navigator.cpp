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
#include "nav_msgs/Odometry.h"

#define ROS_NODE_RATE	1
#define ROS_NODE_NAME	"navigator"

Navigation *navigation;

void OdometryCb(const nav_msgs::Odometry::ConstPtr& msg);

using namespace std;

//=====================================
// Main Code
//=====================================
int main(int argc, char **argv)
{
	ros::init(argc, argv, ROS_NODE_NAME);
	ros::NodeHandle nh("~");

    string neck_topic;
    string marker_config;
    string speech_config;

    nh.param<string>("neck_topic", neck_topic, "/e2/neck");
	nh.param("marker_config", marker_config, ros::package::getPath("e2_navigation")+"/config/marker_config.yaml");
	nh.param("speech_config", speech_config, ros::package::getPath("e2_navigation")+"/config/speech_config.yaml");

	// Suscribers && Publishers for input messages
    ros::Subscriber odom_sub= nh.subscribe("/odom", 10,OdometryCb);

	ROS_INFO("["ROS_NODE_NAME"]:: Node Started");

	navigation = new Navigation(&nh,marker_config,speech_config,ROS_NODE_RATE);
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
