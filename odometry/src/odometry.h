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

#define ROBOT_WIDTH 0.5

class Odometry
{
	private:

		float tanSpeed;
		float rotSpeed;
		tf::TransformBroadcaster odom_broadcaster;



	public:

		double x ;
		double y ;
		double th ;

		double vx;
		double vr ;

		geometry_msgs::Quaternion odom_quat;
		ros::Time current_time, last_time;

		// Constructor
		Odometry();
		Odometry(geometry_msgs::Pose pose);
		~Odometry();

		/*
		 * Compute odometry based on robot speeds
		 *  @param tanSpeed	linear speed of robot
		 *  @param rotSpeed rotational speed
		 */
		void ComputeOdometry(float tanSpeed,float rotSpeed);
};

#endif /* ODOMETRY_H_ */
