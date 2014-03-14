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

#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>

#define ROBOT_WIDTH 0.6
//#define ROBOT_WHEEL ?

class Odometry
{
	private:

		float d_left,d_right;
		tf::TransformBroadcaster odom_broadcaster;

	public:

		double x ;	// X position of the robot
		double y ;	// Y position of the robot
		double th ; // Angular orientation of the robot

		double vx; // linear velocity along x
		double vy; // linear velocity along y (Default 0.0 -not used)

		double vr ; // angular velocity

		double linear_velocity;
		double angular_velocity;

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
		 */
		void UpdateOdometryEncoder();

		/*
		 * Compute odometry based on robot velocity
		 */
		void UpdateOdometryVelocity();

		/*
		 * Compute odometry based on robot movement
		 *  @param d_left	 left wheel movement
		 *  @param d_right right wheel movement
		 */
		void UpdateOdometryMovement(float d_left,float d_right);


		/*
		 * Clear old odometry data
		 */
		void flush();
};

#endif /* ODOMETRY_H_ */
