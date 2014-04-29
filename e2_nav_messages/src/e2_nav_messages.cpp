/*
 * e2_nav_messages.cpp - AIRLab (Politecnico di Milano)
 *
 * Convert cmd_vel message for triskar base
 *
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 15/mar/2014
 *
 */
#define ROS_NODE_RATE	10
#define ROS_NODE_NAME	"e2_nav_messages"

#include "ros/ros.h"
#include "r2p/Velocity.h"
#include "geometry_msgs/Twist.h"

using namespace std;

void getVelocityCmd(const geometry_msgs::TwistConstPtr& msg );

ros::Subscriber sub_cmd_vel;
ros::Publisher pub_triskar_vel ;
r2p::Velocity triskar_msg;

int main(int argc, char **argv)
{
	  ros::init(argc, argv, ROS_NODE_NAME);
	  ros::NodeHandle nh("~");

	  string vel_topic,triskar_topic;

	  nh.param<string>("vel_topic", vel_topic, "/cmd_vel");
	  nh.param<string>("triskar_topic", triskar_topic, "/triskar/velocity");

	  sub_cmd_vel= nh.subscribe(vel_topic, 50, getVelocityCmd);
	  pub_triskar_vel =  nh.advertise<r2p::Velocity>(triskar_topic, 50);

	  ROS_INFO("["ROS_NODE_NAME"]:: Node started");
	  ROS_INFO("["ROS_NODE_NAME"]:: Vel topic : %s ", vel_topic.c_str());
	  ROS_INFO("["ROS_NODE_NAME"]:: Triskar vel topic : %s ", triskar_topic.c_str());

	  ros::Rate r(ROS_NODE_RATE);

	  while(ros::ok())	//ROS LOOP
	  {
		  ros::spinOnce();

		  pub_triskar_vel.publish(triskar_msg);

		  triskar_msg.x = 0.0;
		  triskar_msg.y = 0.0;
		  triskar_msg.w = 0.0;

		  r.sleep();
	  }
	  return 0;
}

//======================================================
//	Get Robot velocity data
//======================================================
void getVelocityCmd(const geometry_msgs::TwistConstPtr& msg )
{
	ROS_DEBUG("["ROS_NODE_NAME"]:: Received vel cmd");

	triskar_msg.x = msg->linear.x;
	triskar_msg.y = msg->linear.y;
	triskar_msg.w = msg->angular.z;
}




