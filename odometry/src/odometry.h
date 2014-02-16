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

class Odometry
{
	private:

		float tanSpeed;
		float rotSpeed;
		ros::Time current_time, last_time;
		geometry_msgs::Quaternion odom_quat;
		tf::TransformBroadcaster odom_broadcaster;

		double x ;
		double y ;
		double th ;
		double vx ;
		double vy ;
		double vth ;

	public:
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

		/*
		 * Return a ros Odometry message
		 */
		nav_msgs::Odometry getOdometryMsg();

};


#endif /* ODOMETRY_H_ */
