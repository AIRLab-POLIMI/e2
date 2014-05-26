/*======================================================================
 * Authors:	cristianmandelli@gmail.com 
 * 		     	deborahzamponi@gmail.com
 * Data: 13/02/2012
 * Description: Handle of head expressions
 * NOTE:
 * To add a new expression, define a new function with its code and add 
 * corresponding enum new item. Than you must modify execution function
 * to handle the new api.
 * ===================================================================*/
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>
//ROS
#include "ros/ros.h"
#include "ros/package.h"
//ActionLib
#include <head_api/HeadAction.h>
#include <actionlib/server/simple_action_server.h>

using namespace std;

//Action server definition
typedef actionlib::SimpleActionServer<head_api::HeadAction> Server;

//Global Scope
//======================================================================
//======================================================================
//Primitives enumeration
enum {STANDARD, INVITATION, INTERESTED, 
			ANGRY, ANNOYED, HAPPY, SAD} primitiveTypes;

//Action results definition
head_api::HeadResult result;

vector<double> params;

//Prototypes
//======================================================================
//======================================================================
//Server execution funcion 
void execute(const head_api::HeadGoalConstPtr& goal, Server* as);
//Expression primitives
void head_standard();  		//code 0
void head_invitation();		//code 1
void head_interested();		//code 2
void head_angry();				//code 3
void head_annoyed();			//code 4
void head_happy();				//code 5
void head_sad();					//code 6
//Other function;
void paramsParser(string src);
//Main
//======================================================================
//======================================================================
int main(int argc, char **argv)
{
	//Ros initialization
	ros::init(argc, argv, "head_api");
	ros::NodeHandle nh;
	
	//Server definition
	Server server(nh, "head_api", boost::bind(&execute, _1, &server), false);
	//Start server
	server.start();
	
	ros::spin();
	return 0;
}


//Functions
//======================================================================
//======================================================================
void execute(const head_api::HeadGoalConstPtr& goal, Server* as)
{
	ROS_INFO("[head_api]::Executing...");
	
	//Reading passed params
	int apiCode = (int)goal->actionCode;
	string par = goal->params;
	paramsParser(par);

	
	if(apiCode == STANDARD)
	{
		head_standard(); 
		result.status = 1;
		params.clear();
	}
	else if(apiCode == INVITATION)
	{
		head_invitation(); 
		result.status = 1;
		params.clear();
	}
	else if(apiCode == INTERESTED)
	{
		head_interested();
		result.status = 1;
		params.clear();
	}
	else if(apiCode == ANGRY)
	{
		head_angry();
		result.status = 1;
		params.clear();
	}
	else if(apiCode == ANNOYED)
	{
		head_annoyed();
		result.status = 1;
		params.clear();
	}
	else if(apiCode == HAPPY)
	{
		head_happy();
		result.status = 1;
		params.clear();
	}
	else if(apiCode == SAD)
	{
		head_sad();
		result.status = 1;
		params.clear();
	}
	else
	{
		ROS_ERROR("[head_api]::Primitive code %d not found", apiCode);
		result.status = -1;
		params.clear();
	}
	
	//Send end message
	as->setSucceeded(result);
}


//======================================================================
//======================================================================
void head_standard()  		//code 0
{
	ROS_INFO("[head_api]::Enacting standar head position...");
}


//======================================================================
//======================================================================
void head_invitation()		//code 1
{
	ROS_INFO("[head_api]::Enacting invitation head position...");
}


//======================================================================
//======================================================================
void head_interested()		//code 2
{
	ROS_INFO("[head_api]::Enacting interested head position...");
}


//======================================================================
//======================================================================
void head_angry()					//code 3
{
	ROS_INFO("[head_api]::Enacting angry head position...");
}


//======================================================================
//======================================================================
void head_annoyed()				//code 4
{
	ROS_INFO("[head_api]::Enacting annoyed head position...");
}


//======================================================================
//======================================================================
void head_happy()					//code 5
{
	ROS_INFO("[head_api]::Enacting happy head position...");
}


//======================================================================
//======================================================================
void head_sad()						//code 6
{
	ROS_INFO("[head_api]::Enacting sad head position...");
}


//======================================================================
//======================================================================
void paramsParser(string src)
{
	size_t found;
	string par;
	
	for(size_t i = 0; i < src.length(); i++)
	{
		if(src.at(0) == ':' || src.length() == 0)
			break;
		else
		{
			//Getting each single parameter from src string
			found = src.find_first_of(",");
			if(found < 0 && src.length() > 0)
			{
				stringstream ss(src);
				double temp;
				ss >> temp;
				params.push_back(temp);
			}
			else if( found > 0 && src.length() > 1)
			{
				par = src.substr(0, found);
				src.erase(0,found+1);
				stringstream ss(par);
				double temp;
				ss >> temp;
				params.push_back(temp);
			}
		}
	}
}
