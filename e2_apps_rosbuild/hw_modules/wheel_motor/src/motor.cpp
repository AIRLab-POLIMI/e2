#include <stdio.h>
#include <string.h>

//Serial Communication libraries
#include "MotorExpert.h"

//Ros Libraries
#include "ros/ros.h"
#include "ros/package.h"

//Ros Messages
#include "robot_brain/WheelData.h"
#include "std_msgs/String.h"
#include "motor/Tilt.h"


using namespace std;

//Variables
//==============================================================================
//==============================================================================
int rotSpeed;;
int tanSpeed;

motor::Tilt kinectTiltMSG;


//Control Variables


//Protoypes
//==============================================================================
//==============================================================================
//Get data from User_Tracker module
void getWheelData(const robot_brain::WheelData wheelDataMSG);

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
  ros::Subscriber subWheelData = nh.subscribe("wheel_data", 100, getWheelData);
  ros::Publisher pubKinectTilt = nh.advertise<motor::Tilt>("tilt", 1);
  
  //SerialCommunication
  MotorExpert* me = new MotorExpert();
  ROS_INFO("[MOTOR]::Opening Serial communication");
  ros::Rate r(500);
	while(ros::ok())	//ROS LOOP
  {
		
		ROS_ERROR("[MOTOR]::Sending data to motor  TanSpeed:%d  -- RotSpeed:%d",  tanSpeed, rotSpeed);
		me->actuations((float)tanSpeed, (float)rotSpeed);
    //me->actuations(0, 0);

		//me->getOdometryData();
		
		//Send messages
		//kinectTiltMSG.kinectPoint.data = 0.0;
		//i++;
		//if (i > 29.0) i = -30.0;
		//pubKinectTilt.publish(kinectTiltMSG);
		ros::spinOnce();
		r.sleep();
	}    
}


//==============================================================================
//==============================================================================
//Get data from User_Tracker module
void getWheelData(const robot_brain::WheelData wheelDataMSG)
{
	//Save wheel data
  rotSpeed = (int)wheelDataMSG.rotSpeed.data;
  tanSpeed = (int)wheelDataMSG.tanSpeed.data;
    
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
}
