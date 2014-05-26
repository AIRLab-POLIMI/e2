/*======================================================================
 * Authors:	cristianmandelli@gmail.com 
 * 		     	deborahzamponi@gmail.com
 * Data: 13/02/2012
 * Description: Handle of neck moves
 * NOTE:
 * To add a new move, define a new function with its code and add 
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
#include <neck_api/NeckAction.h>
#include <actionlib/server/simple_action_server.h>

using namespace std;

//Action server definition
typedef actionlib::SimpleActionServer<neck_api::NeckAction> Server;

//Global Scope
//======================================================================
//======================================================================
//Primitives enumeration
enum {RESET, TILT_LEFT_OR_RIGHT, TILT_HAEAD_OR_BACK, BOW} primitiveTypes;

//Action results definition
neck_api::NeckResult result;

vector<double> params;

//Prototypes
//======================================================================
//======================================================================
//Server execution funcion 
void execute(const neck_api::NeckGoalConstPtr& goal, Server* as);
//Expression primitives
void neck_reset();	  					//code 0
void neck_tilt_lr(int delta);		//code 1
void neck_tilt_hb(int delta);		//code 2
void neck_bow();								//code 3
//Other function;
void paramsParser(string src);
//Main
//======================================================================
//======================================================================
int main(int argc, char **argv)
{
	//Ros initialization
	ros::init(argc, argv, "neck_api");
	ros::NodeHandle nh;
	
	//Server definition
	Server server(nh, "neck_api", boost::bind(&execute, _1, &server), false);
	//Start server
	server.start();
	
	ros::spin();
	return 0;
}


//Functions
//======================================================================
//======================================================================
void execute(const neck_api::NeckGoalConstPtr& goal, Server* as)
{
	ROS_INFO("[neck_api]::Executing...");
	
	//Reading passed params
	int apiCode = (int)goal->actionCode;
	string par = goal->params;
	paramsParser(par);
	
	if(apiCode == RESET)
	{
		neck_reset(); 
		result.status = 1;
		params.clear();
	}
	else if(apiCode == TILT_LEFT_OR_RIGHT)
	{
		if(params.size() == 1)
		{
			neck_tilt_lr((int)params[0]);
			result.status = 1;
		}
		else
		{
			ROS_ERROR("[neck_api]::API parameter error");
			result.status = -1;
		}
			params.clear();
	}
	else if(apiCode == TILT_HAEAD_OR_BACK)
	{
		if(params.size() == 1)
		{
			neck_tilt_hb((int)params[0]);
			result.status = 1;
		}
		else
		{
			ROS_ERROR("[neck_api]::API parameter error");
			result.status = -1;
		}
			params.clear();
	}
	else if(apiCode == BOW)
	{
		neck_bow();
		result.status = 1;
		params.clear();
	}
	else
	{
		ROS_ERROR("[neck_api]::Primitive code %d not found", apiCode);
		result.status = -1;
		params.clear();
	}
	
	//Send end message
	as->setSucceeded(result);
}


//======================================================================
//======================================================================
void neck_reset()			//code 0
{
	ROS_INFO("[neck_api]::Enacting reset neck position...");
}


//======================================================================
//======================================================================
void neck_tilt_lr(int delta)		//code 1
{
	ROS_INFO("[neck_api]::Enacting tilt left/right neck position...");
}


//======================================================================
//======================================================================
void neck_tilt_hb(int delta)		//code 2
{
	ROS_INFO("[neck_api]::Enacting tilt haed/back neck position...");
}


//======================================================================
//======================================================================
void neck_bow()			//code 3
{
	ROS_INFO("[neck_api]::Enacting bow neck position...");
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
