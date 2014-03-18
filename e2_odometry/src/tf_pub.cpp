/*
 * tf_pub.cpp - AIRLab (Politecnico di Milano)
 * 
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 15/feb/2014
 *
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#define ROS_NODE_NAME	"tf_publisher"
#define ROS_NODE_RATE 100

int main(int argc, char** argv)
{
	ros::init(argc, argv, ROS_NODE_NAME);
	ros::NodeHandle n;

	ros::Rate r(ROS_NODE_RATE);

	tf::TransformBroadcaster broadcaster;

	while(n.ok())
	{
		broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.25)),ros::Time::now(),"base_footprint", "base_link"));
		broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 1.2)),ros::Time::now(),"base_link", "kinect_visionSensor"));
		broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 1.18)),ros::Time::now(),"base_link", "laser_scan"));
		broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.25, -0.25, -0.25)),ros::Time::now(),"base_link", "rightWheel"));
		broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.25, 0.25, -0.25)),ros::Time::now(),"base_link", "leftWheel"));
		broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.25, 0.0, -0.25)),ros::Time::now(),"base_link", "rearWheel"));
		r.sleep();
	}
}


