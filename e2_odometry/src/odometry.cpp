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

	vx = 0.0;
	vy = 0.0;
	vr = 0.0;

	distance = 0.0;
	angle = 0.0;

	odom_quat.x=0;
	odom_quat.y=0;
	odom_quat.z=0;
	odom_quat.w=1;

	// Initial time
	last_time=ros::Time::now();
	current_time=ros::Time::now();
};

//==================================================================
//		Constructor to set initial robot pose
//==================================================================
Odometry::Odometry(geometry_msgs::Pose pose)
{
	// Initialize Odometry position
	x = pose.position.x;
	y = pose.position.y;

	vx = 0.0;
	vy = 0.0;
	vr = 0.0;

	distance = 0.0;
	angle = 0.0;

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
	vy = 0.0;
	vr = 0.0;

	distance = 0.0;
	angle = 0.0;

};

//==================================================================
//		Calculate odometry data based on robot wheel velocity from encoders (Triskar base)
//==================================================================
void Odometry::UpdateOdometryEncoder()
{
	current_time=ros::Time::now();
	double elapsed = (current_time - last_time).toSec();

	// TODO
	double phi_1,phi_2,phi_3;

	//Add calc for phi
	phi_1 = enc1_vel * elapsed ;
	phi_2 = enc2_vel * elapsed ;
	phi_3 = enc3_vel * elapsed ;

	//Calculate delta theta
	double delta_th = ( WHEEL_RADIUS / ( 3 * L_DISTANCE) ) * ( phi_1 + phi_2 + phi_3 ) ;

	// Calculate new position
	double delta_x = WHEEL_RADIUS / ( 3 * sin( M_PI / 3 ) )  *
										(
												( cos( M_PI / 3 +  delta_th  ) - cos( M_PI / 3 -  delta_th ) ) * phi_1 +
												( -cos(delta_th) - cos( M_PI / 3 +  delta_th  ) ) * phi_2 +
												( cos(delta_th) + cos( M_PI / 3 -  delta_th ) ) * phi_3
										) ;

	double delta_y = WHEEL_RADIUS / ( 3 * sin( M_PI / 3 ) )  *
										(
												( sin( M_PI / 3 -  delta_th ) + sin( M_PI / 3 +  delta_th  ) ) * phi_1 +
												( -sin(delta_th) - sin( M_PI / 3 +  delta_th  ) ) * phi_2 +
												( sin(delta_th) - sin( M_PI / 3 -  delta_th ) ) * phi_3
										);

	x += delta_x;
	y += delta_y;
	th += delta_th;

	// Update odom quaternion
	odom_quat.x = 0;
	odom_quat.y = 0;
	odom_quat.z = sin(th/2);
	odom_quat.w = cos(th/2);

	distance += sqrt(delta_x * delta_x + delta_y * delta_y);
	angle += fabs(delta_th);

	last_time=current_time;

	ROS_INFO("------------------------------------------------");
	ROS_INFO("[Odom]:: Elapsed  :  %f", elapsed);
	ROS_INFO("[Odom]:: Angle 	  :  %f", angle);
	ROS_INFO("[Odom]:: Distance :  %f", distance);
	ROS_INFO("[Odom]:: RAD SEC ENC_1 :  %f", enc1_vel);
	ROS_INFO("[Odom]:: RAD SEC ENC_2 :  %f", enc2_vel);
	ROS_INFO("[Odom]:: RAD SEC ENC_3 :  %f", enc3_vel);
	ROS_INFO("[Odom]:: Vx  :  %f ", vx);
	ROS_INFO("[Odom]:: Vy  :  %f ", vy);
	ROS_INFO("[Odom]:: Vr  :  %f ", vr);

	ROS_INFO("[Odom]:: delta_x  :  %f ", delta_x);
	ROS_INFO("[Odom]:: delta_y  :  %f ", delta_y);
	ROS_INFO("[Odom]:: delta_th  :  %f ", delta_th);

	ROS_INFO("------------------------------------------------");

	enc1_vel = 0.0;
	enc2_vel = 0.0;
	enc3_vel = 0.0;

};


//==================================================================
//		Calculate odometry data based on robot velocity
//==================================================================
void Odometry::UpdateOdometryVelocity()
{

	current_time=ros::Time::now();
	double elapsed = (current_time - last_time).toSec();

	vx*=SCALE_VELOCITY_COST;
	vy*=SCALE_VELOCITY_COST;
	vr*=SCALE_VELOCITY_COST;

	// Vy inverted - Hw problem
	vy=-vy;

	//compute odometry in a typical way given the velocities of the robot
    double delta_x = (vx * cos(th) - vy * sin(th)) * elapsed;
    double delta_y = (vx * sin(th) + vy * cos(th)) * elapsed;
    double delta_th = vr * elapsed ;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    // Update odom quaternion
	odom_quat.x = 0;
	odom_quat.y = 0;
	odom_quat.z = sin(th/2);
	odom_quat.w = cos(th/2);

	distance += sqrt(delta_x * delta_x + delta_y * delta_y);
	angle += fabs(delta_th);

	last_time=current_time;

	ROS_DEBUG("------------------------------------------------");
	ROS_DEBUG("[Odom]:: Elapsed  :  %f", elapsed);
	ROS_DEBUG("[Odom]:: Angle 	  :  %f", angle);
	ROS_DEBUG("[Odom]:: Distance :  %f", distance);
	ROS_DEBUG("------------------------------------------------");

};

//==================================================================
//	Clear old odometry data
//==================================================================
void Odometry::flush()
{
	vx = 0.0;
	vy = 0.0;
	vr = 0.0;

	d_left  = 0.0 ;
	d_right = 0.0;

}


//==================================================================
//	Publish odometry data on console (DEBUG INFO)
//==================================================================
void Odometry::getOdometryInfo()
{
	ROS_INFO("-----------------------------------------------------------------------");
	//ROS_INFO("[Odom]:: ----- Time passed from last calculation %f",elapsed);
	//ROS_INFO("[Odom]:: ----- Distance Travelled right %f",d_right);
	//ROS_INFO("[Odom]:: ----- Distance Travelled left %f",d_left);
	//ROS_INFO("[Odom]:: ----- Distance Travelled %f",d);
	//ROS_INFO("[Odom]:: ----- Linear Velocity %f m/s",vx);
	//ROS_INFO("[Odom]:: ----- Angular Velocity %f rad/s",vr);
	//ROS_INFO("[Odom]:: ----- Delta X %f",d_x);
	//ROS_INFO("[Odom]:: ----- Delta Y %f",d_y);
	//ROS_INFO("[Odom]:: ----- Delta TH %f",d_th);
	ROS_INFO("-----------------------------------------------------------------------");
}
