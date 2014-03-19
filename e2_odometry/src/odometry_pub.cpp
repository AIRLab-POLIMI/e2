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
#include "e2_msgs/EncoderStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#define ROS_NODE_RATE	100
#define ROS_NODE_NAME	"odometry_pub"

#define VEL_X_MOTOR_START 0.12
#define VEL_Y_MOTOR_START 0.12
#define VEL_W_MOTOR_START 0.35

Odometry *odom ;
bool encoder = false;

void getWheelEnc1(const e2_msgs::EncoderStampedConstPtr& msg);
void getWheelEnc2(const e2_msgs::EncoderStampedConstPtr& msg);
void getWheelEnc3(const e2_msgs::EncoderStampedConstPtr& msg);

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

	  //Messages subscribers
	  tf::TransformBroadcaster broadcaster;

	  ros::Subscriber sub_enc_1 = nh.subscribe(enc_1, 10, getWheelEnc1);
	  ros::Subscriber sub_enc_2 = nh.subscribe(enc_2, 10, getWheelEnc2);
	  ros::Subscriber sub_enc_3 = nh.subscribe(enc_3, 10, getWheelEnc3);

	  ros::Subscriber initialpose = nh.subscribe("/initialpose", 10, getInitialPose);
	  ros::Subscriber sub_cmd_vel= nh.subscribe(vel_topic, 10, getRobotVelocity);

	  if(strcmp(odometry_type.c_str(),string("encoder").c_str()) == 0)
		  encoder=true;

	  ros::Publisher odom_pub =  nh.advertise<nav_msgs::Odometry>("/odom", 1);

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
		  msg.twist.twist.linear.y = odom->vy;
		  msg.twist.twist.angular.z = odom->vr;

		  odom_pub.publish(msg);

		  // Flush odom data
		  odom->flush();

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
	ROS_DEBUG("[Odom]:: Received new robot pose. Update");
	odom->~Odometry();

	odom = new Odometry(msg->pose.pose);
}

//======================================================
//	Get Robot velocity data
//======================================================
void getRobotVelocity(const e2_msgs::VelocityConstPtr& msg)
{
	ROS_DEBUG("[Odom]:: Received new robot velocity.");

	if(abs(msg->x)>=VEL_X_MOTOR_START)
			odom->vx = msg->x;
	if(abs(msg->y)>=VEL_Y_MOTOR_START)
			odom->vy = msg->y;
	if(abs(msg->w)>=VEL_W_MOTOR_START)
			odom->vr = msg->w;

	// Update Odometry information
	if(encoder && odom->enc1 && odom->enc2)
	{
		odom->UpdateOdometryEncoder();
		odom->enc1 = false;
		odom->enc2 = false;
		odom->enc3 = false;
	}else
		odom->UpdateOdometryVelocity();

}

void getWheelEnc1(const e2_msgs::EncoderStampedConstPtr& msg)
{
	odom->enc1 = true;
	odom->enc1_vel=msg->encoder.delta;
}

void getWheelEnc2(const e2_msgs::EncoderStampedConstPtr& msg)
{
	odom->enc2 = true;
	odom->enc2_vel=msg->encoder.delta;
}

void getWheelEnc3(const e2_msgs::EncoderStampedConstPtr& msg)
{
	odom->enc3 = true;
	odom->enc3_vel=msg->encoder.delta;
}
