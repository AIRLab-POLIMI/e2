/*
 * odometry_pub.cpp - AIRLab (Politecnico di Milano)
 *
 *	 This node will print odometry information using encoders info from robot wheels
 *
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 15/feb/2014
 *
 */

#include "ros/ros.h"
#include "odometry.h"

#include "e2_msgs/Velocity.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#define ROS_NODE_RATE	50
#define ROS_NODE_NAME	"odometry_pub"

Odometry *odom ;

void getRobotVelocity(const e2_msgs::VelocityConstPtr& msg);
void getInitialPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg );

using namespace std;

int main(int argc, char **argv)
{
	  ros::init(argc, argv, ROS_NODE_NAME);
	  ros::NodeHandle nh("~");

	  string enc_1,enc_2,enc_3;
	  string odometry_type,vel_topic;

	  nh.param<string>("enc_1", enc_1, "Encoder1");
	  nh.param<string>("enc_2", enc_2, "Encoder2");
	  nh.param<string>("enc_3", enc_3, "Encoder3");

	  nh.param<string>("vel_topic", vel_topic, "/cmd_vel");
	  nh.param<string>("odom_type", odometry_type, "velocity");

	  bool encoder = false;

	  //Messages subscribers
	  tf::TransformBroadcaster broadcaster;

	  ros::Subscriber sub_enc_1;
	  ros::Subscriber sub_enc_2;
	  ros::Subscriber sub_enc_3;
	  ros::Subscriber sub_cmd_vel;
	  ros::Subscriber initialpose = nh.subscribe("initialpose", 10, getInitialPose);

	  if(strcmp(odometry_type.c_str(),string("encoder").c_str()) == 0)
	  {
		  // Encoders
		  //sub_enc_1 = nh.subscribe(enc_1, 100, getWheelData);
		  //sub_enc_2 = nh.subscribe(enc_2, 100, getWheelData);
		  //sub_enc_3 = nh.subscribe(enc_3, 100, getWheelData);
		  encoder=true;
	  }
	  else
		  sub_cmd_vel= nh.subscribe(vel_topic, 10, getRobotVelocity);

	  ros::Publisher odom_pub =  nh.advertise<nav_msgs::Odometry>("/odom", 1000);

	  ROS_INFO("[Odometry]:: Node started");
	  ROS_INFO("[Odometry]:: Odometry updated by : %s ", odometry_type.c_str());
	  ROS_INFO("[Odometry]:: Velocity Topic  : %s ", vel_topic.c_str());

	  if(encoder)
	  {
		  ROS_INFO("[Odometry]:: Encoder1 Topic : %s", enc_1.c_str());
		  ROS_INFO("[Odometry]:: Encoder2 Topic : %s", enc_2.c_str());
		  ROS_INFO("[Odometry]:: Encoder3 Topic : %s", enc_3.c_str());
	  }


	  ros::Rate r(ROS_NODE_RATE);

	  odom = new Odometry();

	  while(ros::ok())	//ROS LOOP
	  {

		  // Update Odometry information
		  if(encoder)
			  odom->UpdateOdometryEncoder();
		  else
			  odom->UpdateOdometryVelocity();

		  // Publish odom information
		  tf::Quaternion quaternion;
		  quaternion.setX(odom->odom_quat.x);
		  quaternion.setY(odom->odom_quat.y);
		  quaternion.setZ(odom->odom_quat.z);
		  quaternion.setW(odom->odom_quat.w);

		  // Broadcasting transformations between odom frame and robot base
		  broadcaster.sendTransform(tf::StampedTransform(tf::Transform(quaternion, tf::Vector3(odom->x, odom->y, 0.0)),ros::Time::now(), "odom","base_footprint"));

		  nav_msgs::Odometry msg;

		  msg.header.stamp = odom->current_time;
		  msg.header.frame_id = "odom";

		  //set the position
		  msg.pose.pose.position.x = odom->x;
		  msg.pose.pose.position.y = odom->y;
		  msg.pose.pose.position.z = 0.0;
		  msg.pose.pose.orientation.x=quaternion.getX();
		  msg.pose.pose.orientation.y=quaternion.getY();
		  msg.pose.pose.orientation.z=quaternion.getZ();
		  msg.pose.pose.orientation.w=quaternion.getW();

		  //set the velocity
		  msg.child_frame_id = "base_footprint";
		  msg.twist.twist.linear.x = odom->vx;
		  msg.twist.twist.linear.y = 0;
		  msg.twist.twist.angular.z = odom->vr;

		  odom_pub.publish(msg);

		  ros::spinOnce();
		  r.sleep();
	  }

	  return 0;
}

//======================================================
//	Get odometry with initial pose
//======================================================
void getInitialPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg )
{
	ROS_INFO("[Odom]:: Received new robot pose. Update");
	odom->~Odometry();

	odom = new Odometry(msg->pose.pose);
}

//======================================================
//	Get Robot velocity data
//======================================================
void getRobotVelocity(const e2_msgs::VelocityConstPtr& msg)
{
	ROS_INFO("[Odom]:: Received new robot velocity.");
	odom->vx = msg->x;
	odom->vy = msg->y;
	odom->vr = msg->w;
}
