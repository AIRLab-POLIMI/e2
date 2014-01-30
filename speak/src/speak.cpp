/*************************************************************
 * Make the robot talk
 *
 * by Lorenzo Ripani - AIRLab (Politecnico di Milano)
 * email: ripani.lorenzo@gmail.com
 *
 * version 0.1 - 01/2014
 *************************************************************/

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>

//Ros Libraries
#include "ros/ros.h"
#include "ros/package.h"
#include "speak/SpeakMessage.h"

#define LOOP_RATE 30

using namespace std;

//Variables
//==============================================================================
//==============================================================================
vector<string> phrases;

//Protoypes
//==============================================================================
//==============================================================================
void loadConfigFile(string file);
void executeCB(const speak::SpeakMessageConstPtr& msg);

const string command_speak = "espeak";
const string command_param = " -s 150 -v it ";
char *command;


//Functions
//==============================================================================
//==============================================================================
int main(int argc, char **argv)
{
	//Initializations
    ros::init(argc, argv, "speak");
    ros::NodeHandle nh;
    
    //load configuration file
    string configFile = ros::package::getPath("speak")+"/config/speak.txt";
	loadConfigFile(configFile);
    
    //Messages subscribers
    ros::Subscriber subHLData = nh.subscribe("speak", 100, executeCB);

	ros::Rate r(LOOP_RATE);
	while(ros::ok())	//ROS LOOP
    {   
		
        ros::spinOnce();
		r.sleep();
    }    
}

void executeCB(const speak::SpeakMessageConstPtr& msg){

	command = new char[500];
	string temp;

	switch (msg->order_id) {

		case 0:

			break;
		case 1:
			temp=command_speak+" "+command_param+"'"+msg->message+"'";
			strcpy(command,temp.c_str());
			system(command);
			ROS_INFO("Talking: %s",msg->message.c_str());
			break;
		case 2:

			temp="killall "+command_speak;
			strcpy(command,temp.c_str());
			system(command);
			ROS_INFO("Stop talking: %s",command);
			break;

		default:
			break;
	}
}

//==============================================================================
//==============================================================================
void loadConfigFile(string file)
{
	string temp;
	
	ifstream File;
	File.open(file.c_str(), ios::in);
	
	while(!File.eof())
	{
		getline(File, temp);
		if (temp.length() > 0)
		{
			phrases.push_back(temp);
		}
	}
	
	File.close();
}
