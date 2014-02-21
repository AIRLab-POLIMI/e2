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

	odom_quat.x=0;
	odom_quat.y=0;
	odom_quat.z=0;
	odom_quat.w=1;

	// Initial time
	last_time=ros::Time::now();
	current_time=ros::Time::now();
};

Odometry::Odometry(geometry_msgs::Pose pose)
{
	// Initialize Odometry position
	x = pose.position.x;
	y = pose.position.y;

	odom_quat = pose.orientation;
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
	x = 0.0;
	y = 0.0;
	th = 0.0;

	vx = 0.0;
	vr = 0.0;
};

//==================================================================
//		Calculate odometry data based on robot velocity
//==================================================================

void Odometry::ComputeOdometry(float tanSpeed, float rotSpeed)
{

	current_time=ros::Time::now();
	double elapsed = (current_time - last_time).toSec();
	last_time=current_time;

	double d_left = 0.0;
	double d_right = 0.0;

	// To Do... Use encoder data - just for simulation test
	double LeftMotorSpeed;
	double RightMotorSpeed;
	LeftMotorSpeed = tanSpeed - rotSpeed * 0.5/2;
	RightMotorSpeed = tanSpeed + rotSpeed * 0.5/2;

	//vx = tanSpeed;
	//vth = rotSpeed;

	// Calculate motor velocity
	d_left = LeftMotorSpeed * elapsed;
	d_right = RightMotorSpeed * elapsed;

	// Distance travelled
	double d = (d_left + d_right) / 2;
	double d_th = (d_right - d_left) / ROBOT_WIDTH;

	// Set velocity from calculated data
	vx = d / elapsed ;
	vr =d_th / elapsed ;
	double d_x,d_y;

	if( d != 0 )
	{
		d_x = cos(d_th) * d ;
		d_y = -sin(d_th) * d ;

		x = x + (cos(th) * d_x - sin(th) * d_y ) ;
		y = y + (sin(th) * d_x + cos(th) * d_y ) ;
	}
	if(d_th != 0)
		th = th + d_th ;

	odom_quat.x = 0;
	odom_quat.y = 0;
	odom_quat.z = sin(th/2);
	odom_quat.w = cos(th/2);

	ROS_INFO("-----------------------------------------------------------------------");
	ROS_INFO("[Odom]:: ----- Received lin %f - rot %f",tanSpeed,rotSpeed);
	ROS_INFO("[Odom]:: ----- Time passed from last calculation %f",elapsed);
	ROS_INFO("[Odom]:: ----- Distance Travelled right %f",d_right);
	ROS_INFO("[Odom]:: ----- Distance Travelled left %f",d_left);
	ROS_INFO("[Odom]:: ----- Distance Travelled %f",d);
	ROS_INFO("[Odom]:: ----- Linear Velocity %f m/s",vx);
	ROS_INFO("[Odom]:: ----- Angular Velocity %f rad/s",vr);
	ROS_INFO("[Odom]:: ----- Delta X %f",d_x);
	ROS_INFO("[Odom]:: ----- Delta Y %f",d_y);
	ROS_INFO("[Odom]:: ----- Delta TH %f",d_th);
	ROS_INFO("-----------------------------------------------------------------------");
};
