/*
 * e2_fake_encoder.cpp - AIRLab (Politecnico di Milano)
 * 
 * Publish data from wheel encoder, using odom data from vrep simulator
 *
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 25/feb/2014
 *
 */

#include <ros/ros.h>
#include <e2_simulator/Encoder.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#define BASE_WIDTH 0.5
#define ROS_NODE_RATE	50
#define ROS_NODE_NAME	"encoder_pub"

ros::Time current_time, last_time,initial_time;

nav_msgs::Odometry e2_curr_odom,e2_prev_odom;

float distance = 0.0 ;
float l_distance = 0.0 ;
float r_distance = 0.0 ;

//===================================================
// Get fake odom messages and update timestamp
//===================================================
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	e2_prev_odom=e2_curr_odom;
	e2_curr_odom.header.stamp=msg->header.stamp;
	e2_curr_odom.pose.pose=msg->pose.pose;
	e2_curr_odom.twist.twist=msg->twist.twist;

	last_time = e2_prev_odom.header.stamp;
	current_time = e2_curr_odom.header.stamp;

	ROS_DEBUG("Pose x,y,z: [%f,%f,%f]", msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
}

//===================================================
// Update encoder values
//===================================================
void updateEncoder()
{
	double elapsed = (current_time - last_time).toSec();

	double vx = e2_curr_odom.twist.twist.linear.x  ;
	double vr = e2_curr_odom.twist.twist.angular.z ;

	double d = vx * elapsed ;
	double th = vr * elapsed ;

	double d_left = ((d * 2) - ( th * BASE_WIDTH )) / 2 ;
	double d_right = (d * 2) - d_left ;

	l_distance = d_left;
	r_distance = d_right;

	ROS_DEBUG("["ROS_NODE_NAME"]:: Elapsed: %f s", elapsed);
	ROS_DEBUG("["ROS_NODE_NAME"]:: Linear Velocity: %f m/s", vx);
	ROS_DEBUG("["ROS_NODE_NAME"]:: Angular Velocity: %f rad/s", vr);
	ROS_DEBUG("["ROS_NODE_NAME"]:: Total Distance: %f m", d);
	ROS_DEBUG("["ROS_NODE_NAME"]:: Left Wheel Distance: %f m", d_left);
	ROS_DEBUG("["ROS_NODE_NAME"]:: Right Wheel Distance: %f m", d_right);

	ROS_DEBUG("["ROS_NODE_NAME"]:: Final count: Travelled %f m in % f seconds", distance,(current_time-initial_time).toSec());
}

//===================================================
// Main Code
//===================================================
int main(int argc, char** argv){

	ros::init(argc, argv, ROS_NODE_NAME);

	ros::NodeHandle n;

	ros::Publisher enc_pub = n.advertise<e2_simulator::Encoder>("enc", 50); // TODO
	ros::Subscriber fake_odom_sub = n.subscribe("/e2/fake_odom", 1000, odomCallback);

	last_time = ros::Time::now() - ros::Duration(1);
	current_time = ros::Time::now();
	initial_time = ros::Time::now();

	ros::Rate r(ROS_NODE_RATE);

	ROS_INFO("["ROS_NODE_NAME"]:: Node Started");

	while(n.ok())
	{
		ROS_DEBUG("**************************************");
		// check for incoming messages
		ros::spinOnce();
		updateEncoder();

		e2_simulator::Encoder msg;
		msg.d_left = l_distance;
		msg.d_right = r_distance;

		// Publish the message
		enc_pub.publish(msg);

		ROS_DEBUG("**************************************");
		r.sleep();

	}

}
