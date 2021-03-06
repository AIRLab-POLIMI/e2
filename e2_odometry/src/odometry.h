/*
 * odometry.h - AIRLab (Politecnico di Milano)
 * 
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 15/feb/2014
 *
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <math.h>
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>

#define L_DISTANCE	0.3 								// Wheel distance from center of mass (m)
#define WHEEL_RADIUS  0.052					// Wheel radius
#define SCALE_VELOCITY_COST 0.645 		// Correction for velocity only

class Odometry
{
	private:

		double elapsed;
		double distance, angle;
		double delta_x,  delta_y, delta_th;

		tf::TransformBroadcaster odom_broadcaster;

	public:

		double x ;	// X position of the robot
		double y ;	// Y position of the robot
		double th ; // Angular orientation of the robot

		double vx; // linear velocity along x
		double vy; // linear velocity along y
		double vr ; // angular velocity

		bool encoder_enabled,debug;
		double enc1_vel,enc2_vel,enc3_vel;

		ros::Time current_time, last_time;
		geometry_msgs::Quaternion odom_quat;

		// Constructor
		Odometry();
		Odometry(geometry_msgs::Pose pose);
		~Odometry();

		/*
		 * Print odometry data on console
		 */
		void getOdometryInfo();

		/*
		 * Compute odometry based on robot encoder
		 * Direct Method
		 */
		void UpdateOdometryEncoder();

		/*
		 * Calc delta xy based ond encoders velocity
		 */
		void calc_delta_xy(double phi_1,double phi_2,double phi_3);

		/*
		 * Calc delta th (rotation based on encoders vel)
		 */
		void calc_delta_th(double phi_1,double phi_2,double phi_3);

		/*
		 * Compute odometry based on robot velocity
		 */
		void UpdateOdometryVelocity();

		/*
		 * Clear old odometry data
		 */
		void clear();
};

#endif /* ODOMETRY_H_ */
