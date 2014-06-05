/*
 * e2_nav_messages.cpp - AIRLab (Politecnico di Milano)
 *
 * Convert cmd_vel message for triskar base
 *
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 15/mar/2014
 *
 */
#define ROS_NODE_RATE	10
#define ROS_NODE_NAME	"e2_nav_messages"

#define SMOOTH_MESSAGES 4

#include "ros/ros.h"
#include "r2p/Velocity.h"
#include "geometry_msgs/Twist.h"

using namespace std;

void getVelocityCmd(const geometry_msgs::TwistConstPtr& msg );

ros::Subscriber sub_cmd_vel;
ros::Publisher pub_triskar_vel ;
r2p::Velocity triskar_curr_msg;
r2p::Velocity triskar_prev_msg;

int main(int argc, char **argv)
{
	  ros::init(argc, argv, ROS_NODE_NAME);
	  ros::NodeHandle nh("~");

	  string vel_topic,triskar_topic;

	  nh.param<string>("vel_topic", vel_topic, "/cmd_vel");
	  nh.param<string>("triskar_topic", triskar_topic, "/triskar/velocity");

	  sub_cmd_vel= nh.subscribe(vel_topic, 1000, getVelocityCmd);
	  pub_triskar_vel =  nh.advertise<r2p::Velocity>(triskar_topic, 1000);

	  ROS_INFO("["ROS_NODE_NAME"]:: Node started");
	  ROS_INFO("["ROS_NODE_NAME"]:: Vel topic : %s ", vel_topic.c_str());
	  ROS_INFO("["ROS_NODE_NAME"]:: Triskar vel topic : %s ", triskar_topic.c_str());

	  ros::Rate r(ROS_NODE_RATE);

	  while(ros::ok())	//ROS LOOP
	  {
		  

		  pub_triskar_vel.publish(triskar_curr_msg);

		  triskar_curr_msg.x = 0.0;
		  triskar_curr_msg.y = 0.0;
		  triskar_curr_msg.w = 0.0;
		
		  ros::spinOnce();
		  r.sleep();
	  }
	  return 0;
}

//======================================================
//	Get Robot velocity data
//======================================================
void getVelocityCmd(const geometry_msgs::TwistConstPtr& msg )
{
	ROS_DEBUG("["ROS_NODE_NAME"]:: Received vel cmd");

	triskar_prev_msg = triskar_curr_msg;

	triskar_curr_msg.x = msg->linear.x;
	triskar_curr_msg.y = msg->linear.y;
	triskar_curr_msg.w = msg->angular.z;

	r2p::Velocity triskar_smooth_msg;
/*
	if(triskar_curr_msg.x > triskar_prev_msg.x  && triskar_curr_msg.w > triskar_prev_msg.w)
	{
		float delta_x = triskar_curr_msg.x - triskar_prev_msg.x;
		float delta_w = triskar_curr_msg.w - triskar_prev_msg.w;

		float acc_x = delta_x / SMOOTH_MESSAGES ;
		float acc_w = delta_w / SMOOTH_MESSAGES ;

		triskar_smooth_msg.x = triskar_prev_msg.x + acc_x;
		triskar_smooth_msg.w = triskar_prev_msg.w + acc_w;
		triskar_smooth_msg.y = 0 ;

		pub_triskar_vel.publish(triskar_smooth_msg);

		triskar_smooth_msg.x += acc_x;
		triskar_smooth_msg.w += acc_w;
		pub_triskar_vel.publish(triskar_smooth_msg);

		triskar_smooth_msg.x += acc_x;
		triskar_smooth_msg.w += acc_w;
		pub_triskar_vel.publish(triskar_smooth_msg);

		triskar_smooth_msg.x += acc_x;
		triskar_smooth_msg.w += acc_w;
		pub_triskar_vel.publish(triskar_smooth_msg);

	}
	else if(triskar_curr_msg.x < triskar_prev_msg.x  && triskar_curr_msg.w < triskar_prev_msg.w)
	{
		float delta_x = triskar_prev_msg.x - triskar_curr_msg.x;
		float delta_w = triskar_prev_msg.w - triskar_curr_msg.w;

		float acc_x = delta_x / SMOOTH_MESSAGES ;
		float acc_w = delta_w / SMOOTH_MESSAGES ;

		triskar_smooth_msg.x = triskar_prev_msg.x - acc_x;
		triskar_smooth_msg.w = triskar_prev_msg.w - acc_w;
		triskar_smooth_msg.y = 0 ;

		pub_triskar_vel.publish(triskar_smooth_msg);

		triskar_smooth_msg.x -= acc_x;
		triskar_smooth_msg.w -= acc_w;
		pub_triskar_vel.publish(triskar_smooth_msg);

		triskar_smooth_msg.x -= acc_x;
		triskar_smooth_msg.w -= acc_w;
		pub_triskar_vel.publish(triskar_smooth_msg);

		triskar_smooth_msg.x -= acc_x;
		triskar_smooth_msg.w -= acc_w;
		pub_triskar_vel.publish(triskar_smooth_msg);
	}
	else if(triskar_curr_msg.w > triskar_prev_msg.w)
	{
		float delta_w = triskar_curr_msg.w - triskar_prev_msg.w;
		float acc_w = delta_w / SMOOTH_MESSAGES ;

		triskar_smooth_msg.x = triskar_curr_msg.x;
		triskar_smooth_msg.w = triskar_prev_msg.w + acc_w;
		triskar_smooth_msg.y = 0 ;

		pub_triskar_vel.publish(triskar_smooth_msg);

		triskar_smooth_msg.w += acc_w;
		pub_triskar_vel.publish(triskar_smooth_msg);

		triskar_smooth_msg.w += acc_w;
		pub_triskar_vel.publish(triskar_smooth_msg);

		triskar_smooth_msg.w += acc_w;
		pub_triskar_vel.publish(triskar_smooth_msg);
	}
	else if(triskar_curr_msg.w < triskar_prev_msg.w)
	{
		float delta_w = triskar_prev_msg.w - triskar_curr_msg.w;
		float acc_w = delta_w / SMOOTH_MESSAGES ;

		triskar_smooth_msg.x = triskar_curr_msg.x;
		triskar_smooth_msg.w = triskar_prev_msg.w - acc_w;
		triskar_smooth_msg.y = 0 ;

		pub_triskar_vel.publish(triskar_smooth_msg);

		triskar_smooth_msg.w -= acc_w;
		pub_triskar_vel.publish(triskar_smooth_msg);

		triskar_smooth_msg.w -= acc_w;
		pub_triskar_vel.publish(triskar_smooth_msg);

		triskar_smooth_msg.w -= acc_w;
		pub_triskar_vel.publish(triskar_smooth_msg);
	}
	*/

}



