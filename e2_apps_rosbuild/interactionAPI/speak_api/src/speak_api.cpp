/*======================================================================
Authors:	cristianmandelli@gmail.com 
	      	deborahzamponi@gmail.com
 Data: 28/01/2012
 Description: Handle of spoken interaction
//====================================================================*/
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
//ROS
#include "ros/ros.h"
#include "ros/package.h"
//ActionLib
#include <speak_api/SpeakAction.h>
#include <actionlib/server/simple_action_server.h>

using namespace std;

//Action server definition
typedef actionlib::SimpleActionServer<speak_api::SpeakAction> Server;

//Global Scope
//======================================================================
//======================================================================
//Action results definition
speak_api::SpeakResult result;

//Speak command
string command = "espeak -s ";

//Prototypes
//======================================================================
//======================================================================
//Server execution funcion 
void execute(const speak_api::SpeakGoalConstPtr& goal, Server* as);

//Main
//======================================================================
//======================================================================
int main(int argc, char **argv)
{
	//Ros initialization
	ros::init(argc, argv, "speak_api");
	ros::NodeHandle nh;
	
	//Server definition
	Server server(nh, "speak_api", boost::bind(&execute, _1, &server), false);
	//Start server
	server.start();
	
	ros::spin();
	return 0;
}



//Functions
//======================================================================
//======================================================================
void execute(const speak_api::SpeakGoalConstPtr& goal, Server* as)
{
	ROS_INFO("[speal_api]::Executing...");
	
	//Read goal data
	string speed = goal->speed;
	string phrase = goal->phrase;
	string language = goal->language;
	
	//Creating command
	command.append(speed);
	command.append(" -v ");
	command.append(language);
	command.append(" '");
	command.append(phrase);
	command.append("'");
	
	//Command execution
	//espeak -s 160 -v it 'ciao.'
	system(command.c_str());

	//reset command
	command = "espeak -s ";
	
	//Send end messages
	result.status = 1;
	as->setSucceeded(result);
	
}
