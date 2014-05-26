/*======================================================================
Authors:	cristianmandelli@gmail.com 
	      	deborahzamponi@gmail.com
 Data: 28/01/2012
 Description: Handle kinect motor
//====================================================================*/
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
//ROS
#include "ros/ros.h"
#include "ros/package.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
//ActionLib
#include <kinect_motor/KinectAction.h>
#include <actionlib/server/simple_action_server.h>
//Other libs
#include "libfreenect.h"

using namespace std;

//Action server definition
typedef actionlib::SimpleActionServer<kinect_motor::KinectAction> Server;

//Global Scope
//======================================================================
//======================================================================
//Action results definition
kinect_motor::KinectResult result;

//Freekinect init
freenect_context *f_ctx;
freenect_device *f_dev;
//ros::Publisher imu_publisher;

int currentTiltPosition;

//Prototypes
//======================================================================
//======================================================================
//Server execution funcion 
void execute(const kinect_motor::KinectGoalConstPtr& goal, Server* as);

//Main
//======================================================================
//======================================================================
int main(int argc, char **argv)
{
	//Ros initialization
	ros::init(argc, argv, "kinect_motor");
	ros::NodeHandle n;
	
	//Variables
	int ledDefaultState = LED_BLINK_GREEN;
	int user_device_number = 0;
	double  imuDefaultDuration = 1.0;
	double  tiltDefaultPosition = 0.0;
	currentTiltPosition = 0;
	
	//Freekinect initializations
  if (freenect_init (&f_ctx, NULL) < 0)
	{
		ROS_ERROR("[kinect_motor]::Freenect_init() failed\n");
    return 1;
	}
	freenect_set_log_level (f_ctx, FREENECT_LOG_INFO);
	 
	//Scan for kinect devices
  int nr_devices = freenect_num_devices (f_ctx);
  // Get the device number 
  if (argc > 1) user_device_number = atoi (argv[1]);
  if (nr_devices < 1) return 1;
    
  // Calculate the kinect node base name
  std::stringstream kinect_node_base;
  kinect_node_base << "/kinect_motor/" << user_device_number << "/";
  
  // Open the base portion of the Kinect
  if (freenect_open_device (f_ctx, &f_dev, user_device_number) < 0) 
  {
		ROS_ERROR("[kinect_motor]::Could not open device\n");
    return 1;
	}
	
	// Get the defaults 
	n.getParam(kinect_node_base.str() + "tilt", tiltDefaultPosition);
  n.getParam(kinect_node_base.str() + "led", ledDefaultState);
  n.getParam(kinect_node_base.str() + "imuDuration", imuDefaultDuration);
  
  // Set the default kinect state
  freenect_set_tilt_degs(f_dev, tiltDefaultPosition);
  freenect_set_led(f_dev, (freenect_led_options) ledDefaultState);
  
	//imu_publisher = n.advertise<sensor_msgs::Imu>(kinect_node_base.str() + "imu", 1000);
  //ros::Timer imu_publish_timer = n.createTimer(ros::Duration(imuDefaultDuration), imu_publish_data);
   
	//Server definition
	Server server(n, "kinect_motor", boost::bind(&execute, _1, &server), false);
	//Start server
	server.start();
	
	ros::spin();
	return 0;
}



//Functions
//======================================================================
//======================================================================
void execute(const kinect_motor::KinectGoalConstPtr& goal, Server* as)
{
	ROS_INFO("[kinect_motor]::Executing...");
	
	int delta = (int)goal->tilt;
	if(delta == -100) //Reset command
		{
			currentTiltPosition = 0;
		}
	else
	{
		//Calculate new kinect position and check its values
		currentTiltPosition = currentTiltPosition + delta;
		if( currentTiltPosition > 30) {currentTiltPosition = 30;}
		else if (currentTiltPosition < -30) {currentTiltPosition = -30;}
	}
	
	//Send data to kinect
	freenect_set_tilt_degs (f_dev, (double)currentTiltPosition);
		
	//Send end messages
	result.status = currentTiltPosition;
	as->setSucceeded(result);
}
