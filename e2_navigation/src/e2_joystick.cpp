/*
 * e2_joystick.cpp - AIRLab (Politecnico di Milano)
 * 
 * Simple keyboard interface to navigate e2 robot
 *
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 16/feb/2014
 */

#include "ros/ros.h"
#include "ros/package.h"

// Teleop
#include <signal.h>
#include <termios.h>
#include <geometry_msgs/Twist.h>

#define ROS_NODE_RATE	50
#define ROS_NODE_NAME	"e2_joystick"

#define VEL_INCREMENT 0.2		// Velocity increment

#define KEYCODE_J 0x6A				// Right
#define KEYCODE_L 0x6c				// Left
#define KEYCODE_I 0x69				// Up
#define KEYCODE_K 0x6B				// Back
#define KEYCODE_PLUS 0x2B		// PLUS
#define KEYCODE_MINUS 0x2D	// MINUS
#define KEYCODE_SPACE 0x20	// SPACE


int main(int argc, char **argv)
{

	ros::init(argc, argv, ROS_NODE_NAME);
	ros::NodeHandle nh;
	geometry_msgs::Twist msg;

	//Message publisher
	ros::Publisher cmd_vel_pub =  nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	ROS_INFO("[e2_joystick]:: Node started");
	ros::Rate r(ROS_NODE_RATE);


	// KEYBOARD
	// get the console in raw mode
	int kfd = 0;
	struct termios cooked, raw;

	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("[e2_joystick]::Reading from keyboard");
	puts("-----------------------------------------------------");
	puts("UP - Straitforward		DOWN - BACK	  ");
	puts("LEFT - rotate left		RIGHT-right		  ");
	puts("+/- Increase velocity Default 0.5m/s (0.2)");
	puts("SPACE Stop Motors									");

	char c;
	float standardTanVelocity=0.5;
	float standardRotVelocity=0.5;

	float tanSpeed=0;
	float rotSpeed=0;

	while(ros::ok())
	{
		// get the next event from the keyboard
		if(read(kfd, &c, 1) < 0)
		{
			perror("read():");
			exit(-1);
		}

		ROS_DEBUG("value: 0x%02X\n", c);

		switch(c)
		{
		      case KEYCODE_J:
		    	  ROS_INFO("[e2_joystick]::LEFT");
		    	  tanSpeed=standardTanVelocity;
		    	  rotSpeed=standardRotVelocity;
		    	  break;
		      case KEYCODE_L:
		    	  ROS_INFO("[e2_joystick]::RIGHT");
		    	  tanSpeed=standardTanVelocity;
		    	  rotSpeed=-standardRotVelocity;
		    	  break;
		      case KEYCODE_I:
		    	  ROS_INFO("[e2_joystick]::UP");
		    	  tanSpeed=standardTanVelocity;
		    	  rotSpeed=0;
		    	  break;
		      case KEYCODE_K:
		    	  ROS_INFO("[e2_joystick]::DOWN");
		    	  tanSpeed=-standardTanVelocity;
		    	  rotSpeed=0;
		    	  break;
		      case KEYCODE_PLUS:
		    	  ROS_INFO("[e2_joystick]::INCREASE VELOCITY");
		    	  standardRotVelocity += VEL_INCREMENT;
		    	  standardTanVelocity += VEL_INCREMENT;
		    	  break;
		      case KEYCODE_MINUS:
		    	  ROS_INFO("[e2_joystick]::DECREASE VELOCITY");
		    	  standardRotVelocity -= VEL_INCREMENT;
		    	  standardTanVelocity -= VEL_INCREMENT;
		    	  break;
		      case KEYCODE_SPACE:
		    	  ROS_INFO("[e2_joystick]::STOP MOTOR");
		    	  tanSpeed = 0.0;
		    	  rotSpeed = 0.0;
		    	  break;
		}

		msg.linear.x =tanSpeed;
		msg.angular.z = rotSpeed;

		cmd_vel_pub.publish(msg);

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}



