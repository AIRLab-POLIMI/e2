/*************************************************************
 * Get messages from vrep and generate fake odom messages,
 * used to correctly simulate robot navigation system.
 *
 * by Lorenzo Ripani - AIRLab (Politecnico di Milano)
 * email: ripani.lorenzo@gmail.com
 *
 * version 0.1 -  01/2014
 *************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#define RATE	5000

geometry_msgs::PoseStamped e2_pose;
geometry_msgs::TwistStamped e2_twist;

/**
 * Get pose messages from vrep.
 */
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	e2_pose.pose=msg->pose;
	ROS_DEBUG("Pose x,y,z: [%f,%f,%f]", msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
	ROS_DEBUG("E2_Pose x,y,z: [%f,%f,%f]", e2_pose.pose.position.x,e2_pose.pose.position.y,e2_pose.pose.position.z);
}

/*
 * Get orientation messages from vrep
 */
void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
	ROS_DEBUG("Twist-linear: [%f,%f,%f]", msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
	e2_twist.twist=msg->twist;
}

//===================================================
// Main Code
//===================================================
int main(int argc, char** argv){

	ros::init(argc, argv, "odometry_publisher");

	ros::NodeHandle n;
	//ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/e2/fake_odom", 50);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
	ros::Subscriber pose_sub = n.subscribe("/e2/pose", 1000, poseCallback);
	ros::Subscriber twist_sub = n.subscribe("/e2/twist", 1000, twistCallback);

	tf::TransformBroadcaster odom_broadcaster;

	ros::Time current_time, last_time;
	last_time = ros::Time::now();
	current_time = ros::Time::now();
	ros::Rate r(5.0);

	ROS_INFO("Fake odom Up and Running......");

	while(n.ok()){

		 // check for incoming messages
		ros::spinOnce();

		current_time = ros::Time::now();

		//all odometry is 6DOF use quaternion
		//geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(e2_pose.pose.);
		double yaw = asin(1);
		geometry_msgs::Quaternion odom_quat = e2_pose.pose.orientation;

		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_footprint";

		// Add position
		odom_trans.transform.translation.x = e2_pose.pose.position.x;
		odom_trans.transform.translation.y = e2_pose.pose.position.y;
		odom_trans.transform.translation.z = 0.0;
		// Add orientation
		odom_trans.transform.rotation = odom_quat;

		// Send the transform
		odom_broadcaster.sendTransform(odom_trans);

		// Publishing odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		// Set the position
		odom.pose.pose= e2_pose.pose;
		odom.pose.pose.position.z = 0.0;


		// Set the velocity
		odom.child_frame_id = "base_footprint";
		odom.twist.twist=e2_twist.twist;

		// Publish the message
		odom_pub.publish(odom);

		last_time = current_time;

		usleep(RATE);

	}
	ROS_INFO("Fake odom ended......");
}
