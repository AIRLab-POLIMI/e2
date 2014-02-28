#include <stdio.h>
#include <string.h>

//Serial Communication libraries
#include "MotorExpert.h"

//Ros Libraries
#include "ros/ros.h"
#include "ros/package.h"

//Ros Messages
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

#define RATE 1

using namespace std;

//Variables
//==============================================================================
//==============================================================================
float rotSpeed;;
float tanSpeed;

//Control Variables


//Protoypes
//==============================================================================
//==============================================================================
//Get data from User_Tracker module
void getWheelData(const geometry_msgs::TwistConstPtr &msg);

//Functions
//==============================================================================
//==============================================================================
int main(int argc, char **argv)
{
	
	//Initializations
	tanSpeed = 0;
	rotSpeed = 0;
	
  ros::init(argc, argv, "motor");
  ros::NodeHandle nh;
    
  //Messages subscribers
  //ros::Subscriber subWheelData = nh.subscribe("wheel_data", 100, getWheelData);
  ros::Subscriber subWheelData = nh.subscribe("cmd_vel", 100, getWheelData);
  //ros::Publisher pubKinectTilt = nh.advertise<motor::Tilt>("tilt", 1);
  
  //SerialCommunication
  MotorExpert* me = new MotorExpert();
  ROS_INFO("[MOTOR]::Opening Serial communication");
  ros::Rate r(RATE);
	while(ros::ok())	//ROS LOOP
  {
		ROS_INFO("*********************************************************************");
		ROS_INFO("[MOTOR]::Sending data to motor  TanSpeed:%f  -- RotSpeed:%f",  tanSpeed, rotSpeed);

		me->actuations((float)tanSpeed, (float)rotSpeed);		
    //me->actuations(0, 0);
		//me->getOdometryData();
		
		ROS_INFO("*********************************************************************");
		ROS_INFO("*********************************************************************");
		//Send messages
		ros::spinOnce();
		r.sleep();
	}    
}


//==============================================================================
//==============================================================================
//Get data from User_Tracker module
void getWheelData(const geometry_msgs::TwistConstPtr &msg)
{
	//Save wheel data
  //rotSpeed = (int)wheelDataMSG.rotSpeed.data;
  //tanSpeed = (int)wheelDataMSG.tanSpeed.data;


  rotSpeed = (int)msg->angular.z;
  tanSpeed = (int)msg->linear.x;

  ROS_INFO("[MOTOR]:: Get velocity command [lin,rot] : %f , %f",tanSpeed,rotSpeed);

/*
  //Check RotSpeed data
  if(rotSpeed >= -10 && rotSpeed <= 10)
		rotSpeed = 0;
  else if(rotSpeed >= 20)
		rotSpeed = 20;
	else if(rotSpeed <= -20)
		rotSpeed = -20;
	
	//Check TanSpeed data
  if(tanSpeed >= -9 && tanSpeed <= 9)
		tanSpeed = 0;
  else if(tanSpeed >= 30)
		tanSpeed = 30;
	else if(tanSpeed <= -30)
		tanSpeed = -30;	
*/
}
