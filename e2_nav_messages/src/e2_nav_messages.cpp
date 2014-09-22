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
#define ROS_NODE_RATE	15
#define ROS_NODE_NAME	"e2_nav_messages"

#define SMOOTH_MESSAGES 4

#include "ros/ros.h"
#include "r2p/Velocity.h"
#include "geometry_msgs/Twist.h"

using namespace std;

void getJoystickVel(const r2p::VelocityConstPtr& msg);
void getVelocity(const geometry_msgs::TwistConstPtr& msg );

ros::Subscriber sub_cmd_vel, sub_joystick_vel;
ros::Publisher pub_triskar_vel ;
r2p::Velocity triskar_curr_msg, triskar_joy_msg;

bool joy_message;

int main(int argc, char **argv)
{
	  ros::init(argc, argv, ROS_NODE_NAME);
	  ros::NodeHandle nh("~");

	  string vel_topic,triskar_topic,joystick_topic;

	  nh.param<string>("vel_topic", vel_topic, "/cmd_vel");
	  nh.param<string>("triskar_topic", triskar_topic, "/triskar/velocity");
	  nh.param<string>("joystick_topic", joystick_topic, "/velocity_joystick");

	  sub_joystick_vel = nh.subscribe(joystick_topic, 1000, getJoystickVel);
	  sub_cmd_vel= nh.subscribe(vel_topic, 1000, getVelocity);
	  pub_triskar_vel =  nh.advertise<r2p::Velocity>(triskar_topic, 1000);

	  ROS_INFO("["ROS_NODE_NAME"]:: Node started");
	  ROS_INFO("["ROS_NODE_NAME"]:: Vel topic : %s ", vel_topic.c_str());
	  ROS_INFO("["ROS_NODE_NAME"]:: Triskar vel topic : %s ", triskar_topic.c_str());

	  ros::Rate r(ROS_NODE_RATE);

	  while(ros::ok())	//ROS LOOP
	  {
		  
		  if(joy_message)
		  {
			  // Override normal messages
			  pub_triskar_vel.publish(triskar_joy_msg);
			  joy_message = false;
		  }
		  else
			  pub_triskar_vel.publish(triskar_curr_msg);

		  triskar_curr_msg.x = 0.0;
		  triskar_curr_msg.y = 0.0;
		  triskar_curr_msg.w = 0.0;
		
		  ros::spinOnce();
		  r.sleep();
	  }
	  return 0;
}

//======================================================
//	Get Robot velocity data
//======================================================
void getVelocity(const geometry_msgs::TwistConstPtr& msg )
{
	ROS_DEBUG("["ROS_NODE_NAME"]:: Received vel cmd");
	triskar_curr_msg.x = msg->linear.x;
	triskar_curr_msg.y = msg->linear.y;
	triskar_curr_msg.w = msg->angular.z;
}

void getJoystickVel(const r2p::VelocityConstPtr& msg)
{
	ROS_DEBUG("["ROS_NODE_NAME"]:: Received joystick cmd - Override normal messages");

	triskar_joy_msg.x = msg->x;
	triskar_joy_msg.y = msg->y;
	triskar_joy_msg.w = msg->w;

	joy_message = true;

}

