/*======================================================================
 Authors:	cristianmandelli@gmail.com 
	      	deborahzamponi@gmail.com
 Data: 27/01/2012
 Description: Provide a list of high level API for user-robot 
			  interaction
======================================================================*/
#include <stdio.h>
#include <string.h>
#include <sstream>

//Ros Libraries
#include "ros/ros.h"
#include "ros/package.h"
#include "robot_brain/HighLevelData.h"
#include "highLevel_Interaction/SpeakData.h"

using namespace std;

//Variables
//==============================================================================
//==============================================================================
bool highLevelIntDataReady;

string primitiveStr;
string paramsStr;

vector<double> params;

//Messages published
highLevel_Interaction::SpeakData speakDataMSG;

//Protoypes
//==============================================================================
//==============================================================================
//Primitives
void face_happy();		//0
void face_sad();		//1
void face_standard();	//2
void face_angry();		//3
void speak_hello();		//4
void speak_goodbye();	//5
void speak_explain();	//6
void neck_forward();	//7
void neck_backwards();	//8
void neck_bow();		//9

//Callbacks
void getPrimitiveDataActivation(const robot_brain::HighLevelData hld);

//Parser
void primitiveMSGParser();

//Other
void callAPINum(unsigned int i);

//Functions
//==============================================================================
//==============================================================================
int main(int argc, char **argv)
{
	//Initializations
	highLevelIntDataReady = false;
	
    ros::init(argc, argv, "robot_brain");
    ros::NodeHandle nh;
    
    //Messages subscribers
    ros::Subscriber subHLData = nh.subscribe("highLevelData", 100, getPrimitiveDataActivation);
    
    //Messages Published
	ros::Publisher pubSpeakData = nh.advertise<highLevel_Interaction::SpeakData>("speak_data", 100);
    
	ros::Rate r(30);
	while(ros::ok())	//ROS LOOP
    {   
		if(highLevelIntDataReady)
		{
			primitiveMSGParser();
		}
		
        ros::spinOnce();
		r.sleep();
    }    
}


//==============================================================================
//==============================================================================
void face_happy()
{
	ROS_INFO("Face Happy");
}

//==============================================================================
//==============================================================================
void face_sad()
{
	ROS_INFO("Face Sad");
}


//==============================================================================
//==============================================================================
void face_standard()
{
	ROS_INFO("Face Standard");
}


//==============================================================================
//==============================================================================
void face_angry()
{
	ROS_INFO("Face Angry");
}


//==============================================================================
//==============================================================================
void speak_hello()
{
	ROS_INFO("Say hello");
	speakDataMSG.phraseNum.data = (int) 3;
}


//==============================================================================
//==============================================================================
void speak_goodbye()
{
	ROS_INFO("Say goodbye");
}


//==============================================================================
//==============================================================================
void speak_explain()
{
	ROS_INFO("Say Explain");
}


//==============================================================================
//==============================================================================
void neck_forward()
{
	ROS_INFO("Put neck forward");
}


//==============================================================================
//==============================================================================
void neck_backwards()
{
	ROS_INFO("Put neck backwards");
}


//==============================================================================
//==============================================================================
void neck_bow()
{
	ROS_INFO("Make a bow");
}


//==============================================================================
//==============================================================================
void callAPINum(unsigned int i)
{
	if (i == 0) {face_happy();}
	if (i == 1) {face_sad();}
	if (i == 2) {face_standard();}
	if (i == 3) {face_angry();}
	if (i == 4) {speak_hello();}
	if (i == 5) {speak_goodbye();}
	if (i == 6) {speak_explain();}
	if (i == 7) {neck_forward();}
	if (i == 8) {neck_backwards();}
	if (i == 9) {neck_bow();}
}


//==============================================================================
//==============================================================================
void primitiveMSGParser()
{
	unsigned int it = 0;
	size_t found;
	string functionParams, functionPar;
	
	for(size_t i = 0; i < primitiveStr.length(); i++)
	{
		if(primitiveStr.at(i) == '1')
		{
			//find first occurence of ":"
			found = paramsStr.find_first_of(":");	
			//save the function params into a substring		
			functionParams=paramsStr.substr(0, found);
			//erase the substring from starting string
			paramsStr.erase(0, found+1);
						
			while(found != string::npos)
			{
				//separe each single parameters from substring
				found = functionParams.find_first_of(",");
				functionPar = functionParams.substr(0, found);
				functionParams.erase(0, found+1);
				
				//ROS_ERROR("PIPPO %s", functionPar.c_str());
				
				//save params into vector
				if (functionPar.length() > 0)
				{
					stringstream ss(functionPar);
					double temp;
					ss >> temp;
				
					params.push_back(temp);
				}
			}
			
			callAPINum(it);
			params.clear();
		}
		it++;
	}
}


//==============================================================================
//==============================================================================
void getPrimitiveDataActivation(const robot_brain::HighLevelData hld)
{
	//Save data from message
	primitiveStr = hld.primitives.data;
	paramsStr = hld.parameters.data;
	
	
    ROS_INFO("Primitive: %s, Params: %s", primitiveStr.c_str(), paramsStr.c_str());
}
