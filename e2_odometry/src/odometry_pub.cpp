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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "r2p/Velocity.h"
#include "r2p/EncoderStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#define ROS_NODE_RATE	30
#define ROS_NODE_NAME	"odometry_pub"

#define VEL_X_MOTOR_START 0.12
#define VEL_Y_MOTOR_START 0.12
#define VEL_W_MOTOR_START 0.35

Odometry *odom ;

void getRobotVelocity(const r2p::VelocityConstPtr& msg);
void getInitialPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg );
void encoderCallback(const r2p::EncoderStampedConstPtr& enc1,const r2p::EncoderStampedConstPtr& enc2,const r2p::EncoderStampedConstPtr& enc3);

using namespace std;
using namespace r2p;
using namespace message_filters;

int main(int argc, char **argv)
{
	  ros::init(argc, argv, ROS_NODE_NAME);
	  ros::NodeHandle nh("~");

	  string enc_1,enc_2,enc_3;
	  string odometry_type,vel_topic;

	  nh.param<string>("enc_1", enc_1, "/triskar/encoder1");
	  nh.param<string>("enc_2", enc_2, "/triskar/encoder2");
	  nh.param<string>("enc_3", enc_3, "/triskar/encoder3");

	  nh.param<string>("vel_topic", vel_topic, "/triskar/velocity");
	  nh.param<string>("odom_type", odometry_type, "encoder");

	  //Messages subscribers
	  tf::TransformBroadcaster broadcaster;

	  message_filters::Subscriber<EncoderStamped> sub_enc_1(nh, enc_1, 1);
	  message_filters::Subscriber<EncoderStamped> sub_enc_2(nh, enc_2, 1);
	  message_filters::Subscriber<EncoderStamped> sub_enc_3(nh, enc_3, 1);

	  TimeSynchronizer<EncoderStamped, EncoderStamped,EncoderStamped> sync(sub_enc_1, sub_enc_2,sub_enc_3, 10);

	  sync.registerCallback(boost::bind(&encoderCallback, _1, _2,_3));

	  ros::Subscriber initialpose = nh.subscribe("/initialpose", 10, getInitialPose);
	  ros::Subscriber sub_cmd_vel= nh.subscribe(vel_topic, 10, getRobotVelocity);

	  ros::Publisher odom_pub =  nh.advertise<nav_msgs::Odometry>("/odom", 1);

	  odom = new Odometry();

	  if(strcmp(odometry_type.c_str(),string("encoder").c_str()) == 0)
		  odom->encoder_enabled=true;
	  else
		  odom->encoder_enabled=false;

	  ROS_INFO("[Odometry]:: Node started");
	  ROS_INFO("[Odometry]:: Odometry updated by : %s ", odometry_type.c_str());
	  ROS_INFO("[Odometry]:: Velocity Topic  : %s ", vel_topic.c_str());

	  if(odom->encoder_enabled)
	  {
		  ROS_INFO("[Odometry]:: Encoder1 Topic : %s", enc_1.c_str());
		  ROS_INFO("[Odometry]:: Encoder2 Topic : %s", enc_2.c_str());
		  ROS_INFO("[Odometry]:: Encoder3 Topic : %s", enc_3.c_str());
	  }

	  ros::Rate r(ROS_NODE_RATE);

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
void getRobotVelocity(const r2p::VelocityConstPtr& msg)
{
	ROS_DEBUG("[Odom]:: Received new robot velocity.");

	odom->vx = 0.0;
	odom->vy = 0.0;
	odom->vr = 0.0;

	if(fabs(msg->x)>=VEL_X_MOTOR_START)
			odom->vx = msg->x;
	if(fabs(msg->y)>=VEL_Y_MOTOR_START)
			odom->vy = msg->y;
	if(fabs(msg->w)>=VEL_W_MOTOR_START)
			odom->vr = msg->w;

	if(!odom->encoder_enabled)
		odom->UpdateOdometryVelocity();

}

//======================================================
//	Get Encoders data
//======================================================
void encoderCallback(const r2p::EncoderStampedConstPtr& enc1,const r2p::EncoderStampedConstPtr& enc2,const r2p::EncoderStampedConstPtr& enc3)
{
	odom->enc1_vel=enc1->encoder.delta;
	odom->enc2_vel=enc2->encoder.delta;
	odom->enc3_vel=enc3->encoder.delta;

	if(odom->encoder_enabled)
		odom->UpdateOdometryEncoder();

}
