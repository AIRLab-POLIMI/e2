//======================================================================
// Authors:	cristianmandelli@gmail.com 
//	      			deborahzamponi@gmail.com
// 				ripani.lorenzo@gmail.com
// Modified: 26/05/2014
// Data: 11/02/2012
// Description: This is e2 brain. This module manages data from vision
// modules, combining them to controll robot actions. This module uses
// other module named "modulename_api" that are some server that provides
// interaction primitives for speak, head expression, neck moves etc called
// by interaction process.
// The interaction process is managed by mean of state machines. 
//======================================================================
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>
//ROS
#include "ros/ros.h"
#include "ros/package.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Float64.h"
//Actionlib
#include <e2_voice/VoiceAction.h>
#include <e2_navigation/NavAction.h>
#include <e2_neck_controller/NeckAction.h>
#include <actionlib/client/simple_action_client.h>
//Modules
#include <user_tracker/Com.h>
#include <head_analyzer/MoveDataMSG.h>
#include <head_analyzer/HeadDataMSG.h>
#include <head_analyzer/EyesDataMSG.h>

//OtherLibs
#include "StateMachine.h"

//Primitives API
#define PRIMITIVE_API 10

//User params
#define USER_Y_MIN_POSITION 150
#define USER_Y_MAX_POSITION 50
//Interaction params
#define INTERACTION_SAMPLES 60

#define ROS_NODE_RATE	1

using namespace std;

//Action clients definition
typedef actionlib::SimpleActionClient<e2_voice::VoiceAction> VoiceClient;
typedef actionlib::SimpleActionClient<e2_navigation::NavAction> NavClient;
typedef actionlib::SimpleActionClient<e2_neck_controller::NeckAction> NeckClient;
//typedef actionlib::SimpleActionClient<kinect_motor::KinectAction> KinectClient;

//Enum
enum transaction { INTERESTED, NOT_INTERESTED };
enum APIcodeDefinition { SPEAK = 0, HEAD = 2, NECK = 4};

struct apiCode
{
	int apiCode[PRIMITIVE_API];
	string apiPar[PRIMITIVE_API];
};

struct phraseData
{
	unsigned int id;
	string speed;
	string language;
	string data;
};

struct headStruct
{
	float ratio;
	int pitch;
	int roll;
};

struct eyesStruct
{
	float leftRatio;
	float rightRatio;
};

struct userStruct
{
	int distance;
	headStruct headData;
	eyesStruct eyesData;
	
	char movement;
};

struct vectorStruct
{
	vector<int> userDistanceVect;
	vector<float> userHeadRatioVect;
	vector<int> userHeadPitchVect;
	vector<int> userHeadRollVect;
	vector<float> userEyeLRatioVect;
	vector<float> userEyeRRatioVect;
	vector<char> userMovesVect;
};

//Global Scope
//======================================================================
//======================================================================
int err;
int userDistance;
int userDistanceThreshold;
int userDistanceY;
int userHeadPitch;
int userHeadRoll;
int interestedCounter = 0;
int not_interestedCounter = 0;
int sampleCounter = 0;

float userEyeLRatio;
float userEyeRRatio;
float userHeadRatio;

char userMoveClass;

bool userPositionDataReady;
bool userEyesDataReady;
bool userMoveDataReady;
bool userHeadRatioDataReady;
bool userHeadPitchDataReady;
bool userHeadRollDataReady;
bool kinectMotorFree;
bool speakHandlerFree;
bool neckHandlerFree;
bool navHandlerFree;
bool visionDataCapture;
bool visionDataAnalyze;
bool firstInteraction;
bool user_interested;
bool start;

vector<phraseData> phrases;
userStruct userData;
apiCode activationAPI;

//Prototypes
//======================================================================
//======================================================================
//Server action done callbacks
void voiceDoneCallback(const actionlib::SimpleClientGoalState& state, const e2_voice::VoiceResultConstPtr& result);
void navDoneCallback(const actionlib::SimpleClientGoalState& state,  const e2_navigation::NavResultConstPtr& result);
void neckDoneCallback(const actionlib::SimpleClientGoalState& state,  const e2_neck_controller::NeckResultConstPtr& result);
//void kinectDoneCallback(const actionlib::SimpleClientGoalState& state, const kinect_motor::KinectResultConstPtr& result);

//Server action activated callbacks
void voiceActiveCallback();
void navActiveCallback();
void neckActiveCallback();
//void kinectActiveCallback();

//Server action feedBack callback
void voiceFeedbackCallback(const e2_voice::VoiceFeedbackConstPtr& feed);
void navFeedbackCallback(const e2_navigation::NavFeedbackConstPtr& feed);
void neckFeedbackCallback(const e2_neck_controller::NeckFeedbackConstPtr& feed);
//void kinectFeedbackCallback(const kinect_motor::KinectFeedbackConstPtr& feed);

//Messages callbacks
void getUserPositionData(const user_tracker::Com com);
void getUserEyesData(const head_analyzer::EyesDataMSG eyes);
void getUserHeadData(const head_analyzer::HeadDataMSG head);
void getUserMoveData(const head_analyzer::MoveDataMSG move);
//Behaviour functions
int getUserDistanceBehaviour(vector<int>* vector);
int getUserHeadRollBehaviour(vector<int>* roll, vector<float>* ratio);
int getUserHeadPitchBehaviour(vector<int>* vector);
int getUserLeftEyeRatioBehaviour(vector<float>* vector);
int getUserRightEyeRatioBehaviour(vector<float>* vector);
char detectMeaningfulMove(vector<char>* vector);
//Other functions
int getVectorOutputValue(vector<int>* vector);
float getVectorOutputValue(vector<float>* vector);
void loadSpeakConfigFile(string file);
void setStatusValue(int interested, int not_interested);
int setMaxValueBt(int a, int b);
void primitiveMSGParser(string primitiveStr, string paramsStr);

bool start_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

//Main
//======================================================================
//======================================================================
int main(int argc, char **argv)
{
	//Ros initialization
	ros::init(argc, argv, "e2_brain");
	ros::NodeHandle nh;
	
	//Controls variables initializations
	userPositionDataReady = false;
	userEyesDataReady = false;
	userMoveDataReady = false;
	userHeadRatioDataReady = false;
	userHeadPitchDataReady = false;
	userHeadRollDataReady = false;
	kinectMotorFree = false;
	speakHandlerFree = true;
	neckHandlerFree = true;
  	navHandlerFree = true;
	visionDataCapture = false;
	visionDataAnalyze = false;
	firstInteraction = true;
	user_interested = false;
	start = false;

	//Variables
	vectorStruct dataVectors;	//Values from vision modules
	Node* node;
	
	//Messages subscribtions
	ros::Subscriber subUserDistance = nh.subscribe("com", 10, getUserPositionData);
	ros::Subscriber subUserHeadData = nh.subscribe("headData", 10, getUserHeadData);
	ros::Subscriber subUserEyesData = nh.subscribe("eyesData", 10, getUserEyesData);
	ros::Subscriber subUserMoveData = nh.subscribe("moveData", 10, getUserMoveData);

	ros::Publisher kinect_pub_ = nh.advertise<std_msgs::Float64>("tilt_angle", 1000);

	ros::ServiceServer start_service = nh.advertiseService("e2_brain/start",start_callback);
	
	//Load state machines that controls the interaction process
	StateMachine *sm = new StateMachine();
	string stateMachineConfigFile = ros::package::getPath("e2_brain")+"/config/state_machine_config/state_machine.txt";
	err = sm->loadFromFile(stateMachineConfigFile);
	sm->printMap();
	if(err < 0)
		ROS_ERROR("[e2_brain]::Error loading state machine from configuration file");
	else
		ROS_INFO("[e2_brain]::State machine from configuration file ... [OK]");
	
	//Load speak configuration file
	string speakConfigFile = ros::package::getPath("e2_brain")+"/config/speak_config/speak.txt";
	loadSpeakConfigFile(speakConfigFile);
	
	//Clients definition
	VoiceClient voiceClient("e2_voice_node", true);
	NeckClient neckClient("e2_neck_controller", true);
	NavClient navClient("e2_nav",true);
	//KinectClient kinectClient("kinect_motor", true);
	
	//Servers goal definitions
	e2_voice::VoiceGoal voiceGoal;
	e2_navigation::NavGoal navGoal;
	e2_neck_controller::NeckGoal neckGoal;
	std_msgs::Float64 kinectGoal;
	
	//Wait for servers
	voiceClient.waitForServer();
	navClient.waitForServer();
	neckClient.waitForServer();
	//kinectClient.waitForServer();
	ROS_INFO("[e2_brain]::Servers connected ... [OK]");
	
	//KinectMotor initializations

	//kinectGoal.tilt = -100;
	//kinectClient.sendGoal(kinectGoal, &kinectDoneCallback, &kinectActiveCallback, &kinectFeedbackCallback);
	//kinectMotorFree = false;

	//RosLoop
	ros::Rate r(ROS_NODE_RATE);
	while(ros::ok())
	{
		if(start)
		{
			ROS_INFO("IN THE LOOP");
			/*==================================================================
													Kinect_motor management
			==================================================================*/
			/*
			if(userPositionDataReady && userDistance < 1500 && navHandlerFree)
			{
				ROS_INFO("[e2_brain]:: kinect move");

				if(userDistanceY > USER_Y_MIN_POSITION && kinectMotorFree)
				{
					kinectMotorFree = false;
					kinectGoal.data = -2;
					//kinectClient.sendGoal(kinectGoal, &kinectDoneCallback, &kinectActiveCallback, &kinectFeedbackCallback);
					kinect_pub_.publish(kinectGoal);
				}
				else if(userDistanceY < USER_Y_MAX_POSITION && kinectMotorFree)
				{
					kinectMotorFree = false;
					kinectGoal.data = 2;
					//kinectClient.sendGoal(kinectGoal, &kinectDoneCallback, &kinectActiveCallback, &kinectFeedbackCallback);
					kinect_pub_.publish(kinectGoal);
				}
			}

			*/
			/*==================================================================
													Locate user
			==================================================================*/
			if(!userPositionDataReady && navHandlerFree)
			{
				ROS_INFO("[e2_brain]:: Looking for user");
				//Looking for user
				navGoal.action_id = 3;	// Start navigation task full optional
				navClient.sendGoal(navGoal, &navDoneCallback, &navActiveCallback, &navFeedbackCallback);
				navHandlerFree = false;
			}

			/*==================================================================
													Vision data management
			==================================================================*/
			if(userPositionDataReady && userDistance < 1000 && navHandlerFree)
				visionDataCapture = true;
			else
			{
				//ROS_INFO("[e2_brain]:: Aproach user, %d",userDistance);
				//Aproach user
				//navGoal.action_id = 2;
				//navGoal.angle=0;
				//navGoal.distance= userDistance - 900;
				//navClient.sendGoal(navGoal, &navDoneCallback, &navActiveCallback, &navFeedbackCallback);
				//navHandlerFree = false;
			}

			if(visionDataCapture && !visionDataAnalyze && navHandlerFree)
			{
				if(firstInteraction)
				{
					sampleCounter = 59;
				}
				//ROS_INFO("VISION DATA %d", sampleCounter);
				//Get user position data
				if(userPositionDataReady)
				{
					dataVectors.userDistanceVect.push_back(userDistanceThreshold);
					userPositionDataReady = false;
					sampleCounter++;
				}
				//Get user head data
				if(userHeadPitchDataReady)
				{
					dataVectors.userHeadPitchVect.push_back(userHeadPitch);
					userHeadPitchDataReady = false;
				}
				if(userHeadRatioDataReady)
				{
					dataVectors.userHeadRatioVect.push_back(userHeadRatio);
					userHeadRatioDataReady = false;
				}
				if(userHeadRollDataReady)
				{
					dataVectors.userHeadRollVect.push_back(userHeadRoll);
					userHeadRollDataReady = false;
				}
				//Get user eyes data
				if(userEyesDataReady)
				{
					dataVectors.userEyeLRatioVect.push_back(userEyeLRatio);
					dataVectors.userEyeRRatioVect.push_back(userEyeRRatio);
					userEyesDataReady = false;
				}
				//Get user move data
				if(userMoveDataReady)
				{
					dataVectors.userMovesVect.push_back(userMoveClass);
					userMoveDataReady = false;
				}
				
				//Stop acquisition and start analysis when all data are acquired
				if((sampleCounter == INTERACTION_SAMPLES) && (speakHandlerFree))
				{
					//ROS_INFO("Interaction Sample acquired");
					//ROS_INFO(" Samples details:");
					//ROS_INFO("      UserTracker:  %d", dataVectors.userDistanceVect.size());
					//ROS_INFO("      HeadAnalyzer: %d", dataVectors.userHeadRatioVect.size());
					visionDataCapture = false;
					visionDataAnalyze = true;
				}
			}
			
			/*==================================================================
													Vision data analysis
			==================================================================*/
			if(visionDataAnalyze && navHandlerFree)
			{
				int t = 1;
				if(!firstInteraction)
				{
					//User Distance analysis
					userData.distance = getUserDistanceBehaviour(&dataVectors.userDistanceVect);
					ROS_ERROR("userDistanceVector -> [OK]");
					if (userData.distance > 0) {setStatusValue(-1, 1);}
					else if (userData.distance == 0) {setStatusValue(1, 0);}
					else if (userData.distance < 0) {setStatusValue(2, -2);}
					
					//User Ratio and Roll analysis
					userData.headData.roll = getUserHeadRollBehaviour(&dataVectors.userHeadRollVect, &dataVectors.userHeadRatioVect);
					ROS_ERROR("userHeadRollVector -> [OK]");
					userData.headData.ratio = userData.headData.roll;
					ROS_ERROR("userHeadRatioVector -> [OK]");
					if (userData.headData.roll != 0) {setStatusValue(2, -2);}
					else if (userData.headData.roll == 0) {setStatusValue(1, 0);}
					
					//User Pitch analysis
					userData.headData.pitch = getUserHeadPitchBehaviour(&dataVectors.userHeadPitchVect);
					ROS_ERROR("userHeadPitchVector -> [OK]");
					if (userData.headData.pitch > 0) {setStatusValue(-2, 2); }
					else if (userData.headData.pitch == 0) {setStatusValue(1, 1); }
					else if (userData.headData.pitch < 0) {setStatusValue(2, -2); }

					//User left eye ratio analysis
					userData.eyesData.leftRatio = getUserLeftEyeRatioBehaviour(&dataVectors.userEyeLRatioVect);
					ROS_ERROR("userleftEyeRatioVector -> [OK]");
					if (userData.eyesData.leftRatio > 0) {setStatusValue(1, -1); }
					else if (userData.eyesData.leftRatio == 0) {setStatusValue(0, 0); }
					else if (userData.eyesData.leftRatio < 0) {setStatusValue(-1, 1); }
					
					//User right eye ratio analysis
					userData.eyesData.rightRatio = getUserRightEyeRatioBehaviour(&dataVectors.userEyeRRatioVect);
					ROS_ERROR("userRightEyeRatioVector -> [OK]");
					if (userData.eyesData.rightRatio > 0) {setStatusValue(1, -1); }
					else if (userData.eyesData.rightRatio == 0) {setStatusValue(0, 0); }
					else if (userData.eyesData.rightRatio < 0) {setStatusValue(-1, 1); }

					//User move analysis
					userData.movement = detectMeaningfulMove(&dataVectors.userMovesVect);
					ROS_INFO("Detected moves: %c", userData.movement);
					if (userData.movement == 'N') {setStatusValue(3, -2); }
					else if (userData.movement == 'D') {setStatusValue(-2, 3); }
					else if (userData.movement == 'L') {setStatusValue(-1, 2); }
					else if (userData.movement == 'R') {setStatusValue(-1, 2); }

					ROS_ERROR("Counters value INTERESTED=%d NOT_INTERESTED=%d",
					interestedCounter, not_interestedCounter);

					//Select interaction action
					t = setMaxValueBt(interestedCounter, not_interestedCounter);

					if(interestedCounter > not_interestedCounter )
						user_interested = true;
					else
						user_interested = false;
				}
				//Clear vector
				dataVectors.userDistanceVect.clear();
				dataVectors.userHeadRatioVect.clear();
				dataVectors.userHeadPitchVect.clear();
				dataVectors.userHeadRollVect.clear();
				dataVectors.userEyeLRatioVect.clear();
				dataVectors.userEyeRRatioVect.clear();
				dataVectors.userMovesVect.clear();
				interestedCounter = 0;
				not_interestedCounter = 0;

				firstInteraction = false;

				//run one step into interaction procesess
				if (t >= 0)
				{
					ROS_ERROR("Run Machine with %d", t);
					node = sm->run(t);
					if(node != NULL)
					{
						//Print node info
						node->displayNode();

						//Check state machines output and send goals to servers
						primitiveMSGParser(node->getOutput(), node->getParams());

						/*for(int i=0; i < 10;  i++)
						{
							ROS_ERROR("code %d", activationAPI.apiCode[i]);
							ROS_ERROR("param %s", activationAPI.apiPar[i].c_str());

						}*/


						//Enable capturing fase
						visionDataCapture = true;
						visionDataAnalyze = false;
						sampleCounter = 0;
					}
					else
					{
						visionDataCapture = false;
						visionDataAnalyze = false;
						ROS_ERROR(" ####### INTERAZIONE COMPLETATA ####### ");

						if (user_interested)
						{
							ROS_INFO("------------------Navigation----------------------");

							// Define Goal to navigation
							navGoal.action_id = 1;	// Start navigation task full optional
							navClient.sendGoal(navGoal, &navDoneCallback, &navActiveCallback, &navFeedbackCallback);
						}

					}
				}
			}
			
			/*==================================================================
												Send Data to server (Actionlib)
			==================================================================*/
			//Speaking
			if(activationAPI.apiCode[SPEAK] == 1)
			{
				activationAPI.apiCode[SPEAK] = -1;
				speakHandlerFree = false;
				
				//Get phrase number
				stringstream ss(activationAPI.apiPar[SPEAK]);
				int temp;
				ss >> temp;

				//Define goal and send it to the server side
				voiceGoal.action_id = 1;
				voiceGoal.text = phrases[temp].data;
				voiceClient.sendGoal(voiceGoal, &voiceDoneCallback, &voiceActiveCallback, &voiceFeedbackCallback);
			}

			//Face move
			if(activationAPI.apiCode[NECK] == 1)
			{
				activationAPI.apiCode[NECK] = -1;

				neckHandlerFree = false;
				int temp;
				//Get head action code (first value of params)
				size_t found = 0;
				found = activationAPI.apiPar[NECK].find_first_of(",");
				if(found > 0)
				{
					string activationCode = activationAPI.apiPar[NECK].substr(0, found);
					stringstream ss(activationCode);
					ss >>temp;
					//Remove fiurst params to string
					activationAPI.apiPar[NECK].erase(0, found + 1);

					//Set goal gode
					neckGoal.action = 1 ;
					neckGoal.sub_action = temp;
				}
				else
				{
					stringstream ss(activationAPI.apiPar[NECK]);
					ss >> temp;
					//Set goal gode
					neckGoal.action = 1 ;
					neckGoal.sub_action = temp;
				}

				//Define goal and send it to the server side
				neckClient.sendGoal(neckGoal, &neckDoneCallback, &neckActiveCallback, &neckFeedbackCallback);
			}




			/*==================================================================
												Ending i-th iteration
			==================================================================*/
		}
		ros::spinOnce();
		r.sleep();
	}
}
//Functions
//======================================================================
//======================================================================
void primitiveMSGParser(string primitiveStr, string paramsStr)
{
	
	unsigned int it = 0;
	size_t found;
	string functionParams, functionPar;
	
	for(size_t i = 0; i < primitiveStr.length(); i++)
	{
		if(primitiveStr.at(i) == '1')
		{
			//Set apiCode activation on true value
			activationAPI.apiCode[i] = 1;
			//find first occurence of ":"
			found = paramsStr.find_first_of(":");	
			//save the function params into activation api struct		
			activationAPI.apiPar[i] = paramsStr.substr(0, found);
			//erase the substring from starting string
			paramsStr.erase(0, found+1);
		}
		else
		{
			activationAPI.apiCode[i] = -1;
			activationAPI.apiPar[i] = " ";
		}
		it++;
	}
}

//======================================================================
//======================================================================
void getUserEyesData(const head_analyzer::EyesDataMSG eyesData)
{
	userEyeLRatio = (float)eyesData.eyeLRatio.data;
	userEyeRRatio = (float)eyesData.eyeRRatio.data;
	
	if(userEyeLRatio > 0.0 && userEyeRRatio > 0.0)
		userEyesDataReady = true;
	else
		userEyesDataReady = false;
	
	//ROS_INFO("[e2_brain]::userEyesData received = LeftRatio %f  --  RightRatio %f ", userEyeLRatio, userEyeRRatio);
}


//======================================================================
//======================================================================
void getUserHeadData(const head_analyzer::HeadDataMSG headData)
{
	userHeadRatio = (float)headData.headRatio.data;
	if( userHeadRatio > 0.0 )
		userHeadRatioDataReady = true;
	else
		userHeadRatioDataReady = false;
	
	userHeadPitch = (int)headData.headPitch.data;
	if( userHeadPitch > 0 )
		userHeadPitchDataReady = true;
	else
		userHeadPitchDataReady = false;
		
	userHeadRoll = (int)headData.headRoll.data;
	if( userHeadRoll != -1 )
		userHeadRollDataReady = true;
	else
		userHeadRollDataReady = false;
		
	//ROS_INFO("[e2_brain]::userHeadData received = Ratio %f  --  Pitch %d  --  Roll %d", userHeadRatio, userHeadPitch, userHeadRoll);
}


//======================================================================
//======================================================================
void getUserMoveData(const head_analyzer::MoveDataMSG moveData)
{
	userMoveClass = (char)(moveData.moveClassification.data).at(0);
	if(userMoveClass != 'U')
		userMoveDataReady = true;
		
	//ROS_INFO("[e2_brain]::UserMoveDataMSG received = %c ", userMoveClass);
}


//======================================================================
//======================================================================
void getUserPositionData(const user_tracker::Com com)
{
    //Save user's distance from camera (z coord)
    userDistance = (int)com.comPoints.z;
    userDistanceThreshold = (int)com.distanceThreshold.data;
    userDistanceY = (int)com.headPoint.y;
        
    //Check if there are still a user
    if (userDistance > 0)
    {
      userPositionDataReady = true;
		}
    else
    {
        ROS_INFO("User out of camera");
        userPositionDataReady = false;
    }
    //ROS_INFO("[e2_brain]::UserPositionDataMSG received = Distance %d  --  DistantThr %d", userDistance, userDistanceThreshold);
}


//======================================================================
//======================================================================
void loadSpeakConfigFile(string file)
{
	phraseData temp;
	string fileLine;
	size_t found;
	int i = 0;
	
	//Open file
	ifstream File;
	File.open(file.c_str(), ios::in);

	while(!File.eof())
	{
		getline(File, fileLine);
		if (fileLine.length() > 0)
		{
			//find first occurence of ":"
			found = fileLine.find_first_of(":");
			if(found == 0) ROS_ERROR("[e2_brain]::Error parsing speak configuration file");
						
			//Get speed param
			temp.speed = fileLine.substr(0,found);
			fileLine.erase(0, found+1);

			found = fileLine.find_first_of(":");
			if(found == 0) ROS_ERROR("[e2_brain]::Error parsing speak configuration file");
		
			//Get language param
			temp.language = fileLine.substr(0,found);
			fileLine.erase(0, found+2);

			fileLine.erase(fileLine.length()-1,fileLine.length());
			temp.data = fileLine;
			temp.id = (unsigned int)i;
			
			phrases.push_back(temp);
			i++;
		}
	}
	
	ROS_INFO("[e2_brain]::Speak configuration file ... [OK]");
	File.close();
}


//==============================================================================
//==============================================================================
int setMaxValueBt(int a, int b)
{
	pair <int, int> values[2];
	pair <int, int> highest;
	
	//initialization highest
	highest.first = -1;
	highest.second = -100;
	
	//save values into array
	values[0].first = INTERESTED ; values[0].second = a ;
	values[1].first = NOT_INTERESTED ; values[1].second =  b;
	
	//detect maximum value
	for(int i = 0; i < 2; i++)
	{
		//ROS_INFO("Current value: %d => %d", values[i].first, values[i].second);
		if(values[i].second > highest.second)
		{
			highest.first = values[i].first;
			highest.second = values[i].second;
			//ROS_INFO("Highest: %d => %d", highest.first, highest.second);
		}
		else if (values[i].second == highest.second)
		{
			//checking of equal value
			if(((values[i].first == INTERESTED) && (highest.first == NOT_INTERESTED)) ||
				 ((values[i].first == NOT_INTERESTED) && (highest.first == INTERESTED)))
				{
					if ((userData.movement == 'D') || (userData.movement == 'R') ||
							(userData.movement == 'L'))
						{ highest.first = NOT_INTERESTED; }
					else { highest.first = INTERESTED; } 
				}
		}
	}
	
	ROS_INFO("Chosen Highest: %d => %d", highest.first, highest.second);
	return highest.first;
	
}

//==============================================================================
//==============================================================================
char detectMeaningfulMove(vector<char>* vector)
{	
	map<char, int> counter;
	map<char,int>::iterator it;
	
	pair<char,int> highest;
	highest.first = 'U';
	highest.second = 0;
	
	//put vector value into map
	for(int i = 0; i < vector->size(); i++)
	{
		if(counter.count(vector->at(i))>0)
		{
			it = counter.find(vector->at(i));
			(*it).second += 1;
		}
		else
		{
			 counter.insert(pair<char,int>(vector->at(i),1));
		}
	}
	
	//go through map for finding max value
	for(it=counter.begin() ; it != counter.end(); it++)
	{
		//cout<<(*it).first<<" => "<<(*it).second<<endl;
		//define max value
		if((*it).second > highest.second)
		{
			highest = make_pair((*it).first, (*it).second);
			cout<<"Highest: "<<highest.first<<" => "<<highest.second<<endl;
		}
		else if((*it).second == highest.second)
		{
			highest.first = 'U';
		}
	}
	
	return highest.first;
}


//==============================================================================
//==============================================================================
int getUserRightEyeRatioBehaviour(vector<float>* rightEyeRatio)
{
	int result = 0;
	
	result = getVectorOutputValue(rightEyeRatio);
	
	return result;
}


//==============================================================================
//==============================================================================
int getUserLeftEyeRatioBehaviour(vector<float>* leftEyeRatio)
{
	int result = 0;
	
	result = getVectorOutputValue(leftEyeRatio);
	
	return result;
}


//==============================================================================
//==============================================================================
int getUserHeadPitchBehaviour(vector<int>* pitch)
{
	int result = 0;
	
	//Detect biggest behaviour inside vector
	result = getVectorOutputValue(pitch);

	return result;
}


//==============================================================================
//==============================================================================
int getUserHeadRollBehaviour(vector<int>* roll, vector<float>* ratio)
{
	int result = 0;

	//Detect biggest behaviour inside vector
	result = getVectorOutputValue(roll);
	
	return result;
}


//==============================================================================
//==============================================================================
int getUserDistanceBehaviour(vector<int>* distance)
{
	int result = 0;
	
	//Detect biggest behaviour inside vector
	result = getVectorOutputValue(distance);
	
	return result;
}

//==============================================================================
//==============================================================================
float getVectorOutputValue(vector<float>* vector)
{
	float result = 0.0;
	float temp = vector->at(0);
	
	for(int i = 1; i < vector->size(); i++)
	{
		if (vector->at(i) > temp) { result++; }
		else if ((vector->at(i) < temp)) { result--; }
		
		temp = vector->at(i);
	}
	
	return result;
}


//==============================================================================
//==============================================================================
void setStatusValue(int interested, int not_interested)
{
	interestedCounter =  interestedCounter + interested;
	not_interestedCounter =  not_interestedCounter + not_interested;

}


//==============================================================================
//==============================================================================
int getVectorOutputValue(vector<int>* vector)
{
	int result = 0;
	int temp = vector->at(0);
	
	for(int i = 1; i < vector->size(); i++)
	{
		if (vector->at(i) > temp) { result++; }
		else if ((vector->at(i) < temp)) { result--; }
		
		temp = vector->at(i);
	}
	
	return result;
}

//======================================================================
//======================================================================
void voiceDoneCallback(const actionlib::SimpleClientGoalState& state,
											 const e2_voice::VoiceResultConstPtr& result)
{
	ROS_INFO("[e2_brain]::Speak done [exit status %s]",result->result.c_str());
	sampleCounter = 40;
	speakHandlerFree = true;
}

//======================================================================
//======================================================================
void voiceActiveCallback()
{
	
}


//======================================================================
//======================================================================
void voiceFeedbackCallback(const e2_voice::VoiceFeedbackConstPtr& feed)
{
	
}

//======================================================================
//======================================================================
void neckDoneCallback(const actionlib::SimpleClientGoalState& state,
											 const e2_neck_controller::NeckResultConstPtr& result)
{
	neckHandlerFree = true;
}

//======================================================================
//======================================================================
void neckActiveCallback()
{}


//======================================================================
//======================================================================
void neckFeedbackCallback(const e2_neck_controller::NeckFeedbackConstPtr& feed)
{}

//======================================================================
//======================================================================
void navDoneCallback(const actionlib::SimpleClientGoalState& state,
											 const e2_navigation::NavResultConstPtr& result)
{
	navHandlerFree = true;
}

//======================================================================
//======================================================================
void navActiveCallback()
{}

//======================================================================
//======================================================================
void navFeedbackCallback(const e2_navigation::NavFeedbackConstPtr& feed)
{}

bool start_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	start = true;
	return true;
}

//======================================================================
//======================================================================
/*
void kinectDoneCallback(const actionlib::SimpleClientGoalState& state,
											 const kinect_motor::KinectResultConstPtr& result)
{
	//ROS_INFO("[e2_brain]::Kinect motor set point reached [exit status %d]", (int)result->status);
	kinectMotorFree = true;
}
*/
//======================================================================
//======================================================================
//void kinectActiveCallback(){}


//======================================================================
//======================================================================
//void kinectFeedbackCallback(const kinect_motor::KinectFeedbackConstPtr& feed){}
