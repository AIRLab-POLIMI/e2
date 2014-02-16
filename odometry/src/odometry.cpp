/*
 * odometry.cpp - AIRLab (Politecnico di Milano)
 * 
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 15/feb/2014
 *
 */

#include "odometry.h"

//==================================================================
//		Constructor
//==================================================================

Odometry::Odometry()
{
	// Initialize Odometry position
	x = 0.0;
	y = 0.0;
	th = 0.0;

	// Initial time
	last_time=ros::Time::now();
	current_time=ros::Time::now();
};

Odometry::Odometry(geometry_msgs::Pose pose)
{
	// Initialize Odometry position
	x = pose.position.x;
	y = pose.position.y;
	th = tf::getYaw(pose.orientation);

	// Initial time
	last_time=ros::Time::now();
	current_time=ros::Time::now();
};
//==================================================================
//		Distructor
//==================================================================

Odometry::~Odometry()
{
	vx = 0.0;
	vy = 0.0;
	vth = 0.0;
};

//==================================================================
//		Calculate odometry data based on robot velocity
//==================================================================

void Odometry::ComputeOdometry(float tanSpeed, float rotSpeed)
{
	current_time=ros::Time::now();

	vx = tanSpeed;
	vy = tanSpeed;
	vth = rotSpeed;

    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

	last_time=current_time;

};

//==================================================================
//		Generate a ros message with odometry data
//==================================================================

nav_msgs::Odometry Odometry::getOdometryMsg()
{

	nav_msgs::Odometry msg;

	msg.header.stamp = current_time;
	msg.header.frame_id = "odom";

    //set the position
	msg.pose.pose.position.x = x;
	msg.pose.pose.position.y = y;
	msg.pose.pose.position.z = 0.0;
	msg.pose.pose.orientation = odom_quat;

    //set the velocity
	msg.child_frame_id = "base_footprint";
	msg.twist.twist.linear.x = vx;
    msg.twist.twist.linear.y = vy;
    msg.twist.twist.angular.z = vth;

	return msg;
}
