/*======================================================================
Authors:	cristianmandelli@gmail.com 
	      	deborahzamponi@gmail.com
 Data: 13/02/2012
 Description: Handle of wheel motor
//====================================================================*/
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
//ROS
#include "ros/ros.h"
#include "ros/package.h"
//ActionLib
#include <wheel_motor/WheelAction.h>
#include <actionlib/server/simple_action_server.h>
//Other libs
#include "MotorExpert.h"

#define MAX_TAN_SPEED 60
#define MIN_TAN_SPEED -60
#define MAX_ROT_SPEED 60
#define MIN_ROT_SPEED -60

using namespace std;

//Action server definition
typedef actionlib::SimpleActionServer<wheel_motor::WheelAction> Server;

//Global Scope
//======================================================================
//======================================================================
//Action results definition
wheel_motor::WheelResult result;

//Motor expert from mrt
MotorExpert* me;

//Wheel speed values
int rotSpeed = 0;
int tanSpeed = 0;

//Prototypes
//======================================================================
//======================================================================
//Server execution funcion 
void execute(const wheel_motor::WheelGoalConstPtr& goal, Server* as);

//Main
//======================================================================
//======================================================================
int main(int argc, char **argv)
{
	//Ros initialization
	ros::init(argc, argv, "wheel_motor");
	ros::NodeHandle nh;
		
	//Server definition
	Server server(nh, "wheel_motor", boost::bind(&execute, _1, &server), false);
	//Start server
	server.start();
	
	ros::spin();
	return 0;
}



//Functions
//======================================================================
//======================================================================
void execute(const wheel_motor::WheelGoalConstPtr& goal, Server* as)
{
	
	ROS_INFO("[wheel_motor]::Executing...");
	MotorExpert* me = new MotorExpert();

	//Save previous speed values
	int prevRotSpeed = rotSpeed;
	int prevTanSpeed = tanSpeed;
	
	//Get new speed values
	rotSpeed = (int)goal->rotSpeed;
	tanSpeed = (int)goal->tanSpeed;
	
	//Corrections
	if( rotSpeed > MAX_ROT_SPEED)  {rotSpeed = MAX_ROT_SPEED;}
	else if( rotSpeed < MIN_ROT_SPEED)  {rotSpeed = MIN_ROT_SPEED;}
	
	if( tanSpeed > MAX_TAN_SPEED)  {tanSpeed = MAX_TAN_SPEED;}
	else if( tanSpeed < MIN_TAN_SPEED)  {tanSpeed = MIN_TAN_SPEED;}
	
	if(prevRotSpeed == rotSpeed && prevTanSpeed == tanSpeed)
	{
		ROS_INFO("[wheel_motor]::Skip speed commands - Motor already on setpoint");
		result.status = 0;
	}
	else if(prevTanSpeed < tanSpeed) //SpeedUp
	{
		int temp = tanSpeed - prevTanSpeed;
		if(temp > 3)
		{
			ROS_INFO("Sono qui TanSpeed %d-- rotSpeed %d", prevTanSpeed+5, rotSpeed);
			me->actuations(prevTanSpeed+3, rotSpeed);
			result.status = 1;
			
			tanSpeed = prevTanSpeed + 3;
		}
		else
		{
			ROS_INFO("Sono qui TanSpeed %d-- rotSpeed %d", tanSpeed, rotSpeed);
			me->actuations(tanSpeed, rotSpeed);
			result.status = 1;
		}
	}
	else if(prevTanSpeed > tanSpeed) //SpeedDown
	{
		int temp = prevTanSpeed - tanSpeed;
		if(temp > 3)
		{
			ROS_INFO("Sono qui TanSpeed %d-- rotSpeed %d", prevTanSpeed-5, rotSpeed);
			me->actuations(prevTanSpeed-3, rotSpeed);
			result.status = 1;
			
			tanSpeed = prevTanSpeed - 3;
		}
		else
		{
			ROS_INFO("Sono qui TanSpeed %d-- rotSpeed %d", tanSpeed, rotSpeed);
			me->actuations(tanSpeed, rotSpeed);
			result.status = 1;
		}
	}	
	else
	{
		ROS_INFO("Sono qui TanSpeed %d-- rotSpeed %d", tanSpeed, rotSpeed);
		//Apply here some timing....
		
		me->actuations(tanSpeed, rotSpeed);
		result.status = 1;
	}
	
	ros::Duration(0.3).sleep();	
	as->setSucceeded(result);
}
