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
#include "ros/package.h"
#include "odometry.h"

//#include "e2_odometry/Encoder.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#define ROS_NODE_RATE	50
#define ROS_NODE_NAME	"odometry_pub"

float d_left = 0.0;
float d_right = 0.0;
Odometry *odom ;

/*
void getWheelData(const odometry::Encoder::ConstPtr& msg)
{
	d_left = msg->d_left;
	d_right = msg->d_right;
}*/
void getInitialPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg )
{
	ROS_INFO("[Odom]:: Received new robot pose. Update");
	odom->~Odometry();

	odom = new Odometry(msg->pose.pose);

}

int main(int argc, char **argv)
{
	  ros::init(argc, argv, ROS_NODE_NAME);
	  ros::NodeHandle nh;

	  //Messages subscribers
	  tf::TransformBroadcaster broadcaster;
	  ros::Subscriber initialpose = nh.subscribe("initialpose", 100, getInitialPose);
	  //ros::Subscriber subWheelData = nh.subscribe("enc", 100, getWheelData);
	  ros::Publisher odom_pub =  nh.advertise<nav_msgs::Odometry>("odom", 1000);

	  ROS_INFO("[Odom]:: Node started");
	  ros::Rate r(ROS_NODE_RATE);

	  odom = new Odometry();

	  while(ros::ok())	//ROS LOOP
	  {

		  // Update Odometry information
		  odom->ComputeOdometry(d_left,d_right);
		  d_left = 0.0;
		  d_right = 0.0;

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
