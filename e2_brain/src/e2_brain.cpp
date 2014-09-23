//======================================================================
// Authors:	cristianmandelli@gmail.com 
//	      	deborahzamponi@gmail.com
// 			ripani.lorenzo@gmail.com
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
#include <string>
#include <fstream>

//ROS
#include "ros/ros.h"
#include "ros/package.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Float64.h"
//Actionlib
#include <kinect_motor/KinectAction.h>
#include <e2_voice/VoiceAction.h>
#include <e2_navigation/NavAction.h>
#include <actionlib/client/simple_action_client.h>
//Modules
#include <user_tracker/Com.h>
#include <head_analyzer/MoveDataMSG.h>
#include <head_analyzer/HeadDataMSG.h>
#include <head_analyzer/EyesDataMSG.h>

//OtherLibs
#include "StateMachine.h"
#include "yaml-operator.h"

//Primitives API
#define PRIMITIVE_API 10

//User params
#define USER_Y_MIN_POSITION 150
#define USER_Y_MAX_POSITION 50
//Interaction params
#define INTERACTION_SAMPLES 60

#define ROS_NODE_RATE	5

using namespace std;

//Action clients definition
typedef actionlib::SimpleActionClient<e2_voice::VoiceAction> VoiceClient;
typedef actionlib::SimpleActionClient<e2_navigation::NavAction> NavClient;
typedef actionlib::SimpleActionClient<kinect_motor::KinectAction> KinectClient;

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


typedef struct
{
	int action_aborted;
	int action_completed;

	// The robot status express a emotional status of the robot. It can be normal,sad, happy, frustrated, dreamer, or angry
	string status;
	ros::Time init_time;
	ros::Time last_interaction;

}robot_status;


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

// Load Speech
Speech *speechs_;
int speech_size_;

bool find_user;
bool approach_user;
bool check_user_interested;
bool navigate_user;

bool user_interested;

bool demo;

vector<phraseData> phrases;
userStruct userData;
apiCode activationAPI;

//Prototypes
//======================================================================
//======================================================================
//Server action done callbacks
void voiceDoneCallback(const actionlib::SimpleClientGoalState& state, const e2_voice::VoiceResultConstPtr& result);
void navDoneCallback(const actionlib::SimpleClientGoalState& state,  const e2_navigation::NavResultConstPtr& result);
void kinectDoneCallback(const actionlib::SimpleClientGoalState& state, const kinect_motor::KinectResultConstPtr& result);

//Server action activated callbacks
void voiceActiveCallback();
void navActiveCallback();
void kinectActiveCallback();

//Server action feedBack callback
void voiceFeedbackCallback(const e2_voice::VoiceFeedbackConstPtr& feed);
void navFeedbackCallback(const e2_navigation::NavFeedbackConstPtr& feed);
void kinectFeedbackCallback(const kinect_motor::KinectFeedbackConstPtr& feed);

//Messages callbacks
void getUserPositionData(const user_tracker::Com com);
void getUserEyesData(const head_analyzer::EyesDataMSG eyes);
void getUserHeadData(const head_analyzer::HeadDataMSG head);
void getUserMoveData(const head_analyzer::MoveDataMSG move);
void getVoiceCommands(const std_msgs::String command);

//Behaviour functions
int getUserDistanceBehaviour(vector<int>* vector);
int getUserHeadRollBehaviour(vector<int>* roll, vector<float>* ratio);
int getUserHeadPitchBehaviour(vector<int>* vector);
int getUserLeftEyeRatioBehaviour(vector<float>* vector);
int getUserRightEyeRatioBehaviour(vector<float>* vector);
char detectMeaningfulMove(vector<char>* vector);


//Other functions
void initialize();
void abort_call(const ros::TimerEvent& event);

void robot_talk(Speech speech);
Speech get_speech_by_name(string name);
Speech get_random_speech(string what);
void loadSpeakData(YAML::Node& doc);

int getVectorOutputValue(vector<int>* vector);
float getVectorOutputValue(vector<float>* vector);
void loadSpeakConfigFile(string file);
void setStatusValue(int interested, int not_interested);
int setMaxValueBt(int a, int b);
void primitiveMSGParser(string primitiveStr, string paramsStr);

bool start_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
bool stop_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

StateMachine *sm;
string stateMachineConfigFile;

robot_status robot;
ros::Timer abort_timeout;

//Servers goal definitions
e2_voice::VoiceGoal voiceGoal;
e2_navigation::NavGoal navGoal;
kinect_motor::KinectGoal kinectGoal;

VoiceClient *voiceClient;


//======================================================================
// Main
//======================================================================
int main(int argc, char **argv)
{
	//Ros initialization
	ros::init(argc, argv, "e2_brain");
	ros::NodeHandle nh;
	
	//Variables
	vectorStruct dataVectors;	//Values from vision modules
	Node* node;
	
	//Messages subscribtions
	ros::Subscriber subUserDistance = nh.subscribe("com", 10, getUserPositionData);
	ros::Subscriber subUserHeadData = nh.subscribe("headData", 10, getUserHeadData);
	ros::Subscriber subUserEyesData = nh.subscribe("eyesData", 10, getUserEyesData);
	ros::Subscriber subUserMoveData = nh.subscribe("moveData", 10, getUserMoveData);
	ros::Subscriber subVoiceCommand = nh.subscribe("/recognizer/output", 10, getVoiceCommands);

	//Services
	ros::ServiceServer start_service = nh.advertiseService("e2_brain/start",start_callback);
	ros::ServiceServer stop_service = nh.advertiseService("e2_brain/abort",stop_callback);
	// Abort Timer
	//abort_timeout = nh.createTimer(ros::Duration(120),abort_call,true,false); // Timer per annullare l'operazione

	
	//Inizializza le variabili di sistema allo stato iniziale
	stateMachineConfigFile = ros::package::getPath("e2_config")+"/state_machine_config/state_machine.txt";
	initialize();
	
	//Load speak configuration file
	string speakConfigFile = ros::package::getPath("e2_config")+"/speak_config/speak.txt";
	loadSpeakConfigFile(speakConfigFile);
	
	string speech_config = ros::package::getPath("e2_config")+"/speak_config/speech_config.yaml";
	YAML::Node doc_speech = YAML::LoadFile(speech_config.c_str());
	speech_size_=doc_speech.size();
	loadSpeakData(doc_speech);


	ROS_ERROR("[e2_brain]:: Loaded Speech Config %s with %d conversations", speech_config.c_str(),(int)doc_speech.size());

	//Clients definition
	voiceClient = new VoiceClient("e2_voice_node", true);
	NavClient navClient("e2_nav",true);
	KinectClient kinectClient("kinect_motor", true);
	
	//Wait for servers
	//while (!navClient.waitForServer(ros::Duration(5.0)))
	//	ROS_INFO("[e2_brain]:: Waiting for the navigation action server to come up");
	//while (!voiceClient->waitForServer(ros::Duration(5.0)))
	//	ROS_INFO("[e2_brain]:: Waiting for the voice action server to come up");
	//while (!kinectClient.waitForServer(ros::Duration(5.0)))
	//	ROS_INFO("[e2_brain]:: Waiting for the kinect action server to come up");

	ROS_INFO("[e2_brain]::Servers connected ... [OK]");
	
	//KinectMotor initializations
	kinectGoal.tilt = -100;
	kinectClient.sendGoal(kinectGoal, &kinectDoneCallback, &kinectActiveCallback, &kinectFeedbackCallback);
	kinectMotorFree = false;

	// Initialize Robot Status
	robot.action_aborted = 0;
	robot.action_completed = 0;
	robot.status = "normal_";

	robot.init_time =  ros::Time::now();
	robot.last_interaction  = robot.init_time ;

	//RosLoop
	ros::Rate r(ROS_NODE_RATE);

	while(ros::ok())
	{
		//======================================================================
		//					I STEP - FIND USER
		//======================================================================
		if(find_user)
		{

			if(navHandlerFree)
			{
				ROS_ERROR("[e2_brain]:: Request to find a user sent at e2_navigation module...");
				navGoal.action_id = 1;
				navClient.sendGoal(navGoal, &navDoneCallback, &navActiveCallback, &navFeedbackCallback);
				navHandlerFree = false;
				navClient.waitForResult();
			}

		}

		//======================================================================
		//					II STEP - APPROACH USER IF TOO DISTANT
		//======================================================================
		if(approach_user)
		{
			/*==================================================================
							Kinect_motor management
			==================================================================*/

			if(userPositionDataReady && userDistance < 1500 && navHandlerFree)
			{
				if(userDistanceY > USER_Y_MIN_POSITION && kinectMotorFree)
				{
					kinectMotorFree = false;
					kinectGoal.tilt = -2;
					kinectClient.sendGoal(kinectGoal, &kinectDoneCallback, &kinectActiveCallback, &kinectFeedbackCallback);
				}
				else if(userDistanceY < USER_Y_MAX_POSITION && kinectMotorFree)
				{
					kinectMotorFree = false;
					kinectGoal.tilt = 2;
					kinectClient.sendGoal(kinectGoal, &kinectDoneCallback, &kinectActiveCallback, &kinectFeedbackCallback);
				}
			}

			/*==================================================================
								Vision data management
			==================================================================*/
			if(userPositionDataReady && userDistance < 1500 && navHandlerFree)
				visionDataCapture = true;
			else if(userPositionDataReady && userDistance > 1500 && navHandlerFree)
			{
				ROS_ERROR("[e2_brain]:: NEED TO APPROACH!!!!!!!! Manual request, %d distance",userDistance);

				//navGoal.action_id = 2;
				//navClient.sendGoal(navGoal, &navDoneCallback, &navActiveCallback, &navFeedbackCallback);
				navHandlerFree = false;
				userPositionDataReady = false;
				userDistance = 9999;
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

		}

		//======================================================================
		//					III STEP - CHECK IF USER IS INTERESTED
		//======================================================================
		if(check_user_interested)
		{

			/*==================================================================
								INTEREST DETECTION - Vision data analysis
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
				//else
				//abort_timeout.start();

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
							ROS_ERROR("[e2_brain]:: User INTERESTED !");

							navigate_user = true;
							approach_user = false;
							check_user_interested = false;

							robot.last_interaction = ros::Time::now();

						}
						else
						{
							ROS_ERROR("[e2_brain]:: User NOT INTERESTED !");
							// Initialize due to not interested user
							initialize();
							sleep(2);

							find_user = true;
							robot.action_aborted++;

						}

					}
				}
			}

		}

		//======================================================================
		//					III STEP - NAVIGATE TO BASE (KEEP TRACK OF THE USER)
		//======================================================================
		if(navigate_user && navHandlerFree)
		{
			ROS_ERROR("[e2_brain]:: Navigate user to the base...");
			navGoal.action_id = 3;
			navClient.sendGoal(navGoal, &navDoneCallback, &navActiveCallback, &navFeedbackCallback);
			navHandlerFree = false;
		}

		/*==================================================================
			Do you need to say something E-2?
		==================================================================*/
		//Speaking
		if(activationAPI.apiCode[SPEAK] == 1 && navHandlerFree)
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
			voiceGoal.neck_action = 0;
			voiceGoal.face_action = 1;

			voiceClient->sendGoal(voiceGoal, &voiceDoneCallback, &voiceActiveCallback, &voiceFeedbackCallback);
		}

		if(demo)
		{
			robot_talk(get_random_speech("joke_"));
			voiceClient->waitForResult();
			sleep(5);
		}

		/*==================================================================
			How do you feel E-2?
		==================================================================*/
		if(true)
		{
			string temp_status = robot.status;

			if(robot.action_aborted == robot.action_completed)
			{
				robot.status = "normal_";
				//if((ros::Time::now() - robot.last_interaction > ros::Duration(120.0)))
				//	robot.status = "fuzzy_";
			}
			else if (robot.action_completed > robot.action_aborted)
			{
				robot.status = "happy_";
				if((robot.action_completed - robot.action_aborted) > 3)
					robot.status = "god_";
			}
			else if (robot.action_aborted > robot.action_completed)
			{
				robot.status = "angry_";
				if((robot.action_aborted  - robot.action_completed) > 3)
					robot.status = "frustrated_";
			}

			//==================================================
			//	Do you wanna talk ?
			//==================================================
			if(strcmp(temp_status.c_str(),robot.status.c_str()) != 0)
			{
				robot_talk(get_random_speech(robot.status));
				voiceClient->waitForResult();
			}
		}

		ros::spinOnce();
		r.sleep();
	}
}

//======================================================================
// Functions
//======================================================================

void abort_call(const ros::TimerEvent& event)
{
	ROS_ERROR("[e2_brain]:: Timeout occured, abort current action");
	initialize();
}

void initialize()
{
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


	demo = false;
	find_user = false;
	approach_user = false;
	check_user_interested = false;
	navigate_user = false;

	user_interested = false;

	interestedCounter = 0;
	not_interestedCounter = 0;
	sampleCounter = 0;

	//Load state machines that controls the interaction process
	sm = new StateMachine();

	err = sm->loadFromFile(stateMachineConfigFile);
	//sm->printMap();
	if(err < 0)
		ROS_ERROR("[e2_brain]:: Error loading state machine from configuration file");
	else
		ROS_INFO("[e2_brain]:: State machine from configuration file ... [OK]");

	//abort_timeout.stop();

}

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
//	Get voice commands and start the right action TODO
//======================================================================
void getVoiceCommands(const std_msgs::String command)
{
	std::size_t found = command.data.find("e_two");

	if (found!=std::string::npos)
	{
		ROS_ERROR("[e2_brain::Voice]:: Ho ricevuto un comando !");

		found = command.data.find("find people");
		if (found!=std::string::npos)
		{
			ROS_ERROR("[e2_brain::Voice]:: Trova persone !");
			robot_talk(get_speech_by_name("ack"));

			initialize();
			find_user = true;

			return;
		}

		found = command.data.find("joke");
		if (found!=std::string::npos)
		{
			ROS_ERROR("[e2_brain::Voice]:: Racconto una barzelletta!");
			robot_talk(get_speech_by_name("ack"));
			voiceClient->waitForResult();
			robot_talk(get_random_speech(string("joke_")));
			voiceClient->waitForResult();
			return;
		}

		found = command.data.find("news");
		if (found!=std::string::npos)
		{
			ROS_ERROR("[e2_brain::Voice]:: Racconto una notizia!");
			robot_talk(get_speech_by_name("ack"));
			voiceClient->waitForResult();

			robot_talk(get_random_speech(string("nav_")));
			voiceClient->waitForResult();

			return;
		}

		found = command.data.find("law");
		if (found!=std::string::npos)
		{
			ROS_ERROR("[e2_brain::Voice]:: Laws ");

			robot_talk(get_speech_by_name("law_0"));
			voiceClient->waitForResult();
			robot_talk(get_speech_by_name("law_1"));
			voiceClient->waitForResult();
			robot_talk(get_speech_by_name("law_2"));
			voiceClient->waitForResult();
			return;
		}

		found = command.data.find("abort");
		if (found!=std::string::npos)
		{
			ROS_ERROR("[e2_brain::Voice]:: Mi Fermo!");
			initialize();

			robot_talk(get_speech_by_name("ack"));
			voiceClient->waitForResult();
			return;
		}

		found = command.data.find("how are");
		if (found!=std::string::npos)
		{
			ROS_ERROR("[e2_brain::Voice]:: Robot Status");

			robot_talk(get_random_speech(robot.status));
			voiceClient->waitForResult();
			return;
		}

		found = command.data.find("how feel");
		if (found!=std::string::npos)
		{
			ROS_ERROR("[e2_brain::Voice]:: Robot Status");

			robot_talk(get_random_speech(robot.status));
			voiceClient->waitForResult();
			return;
		}

		found = command.data.find("thank you");
		if (found!=std::string::npos)
		{
			ROS_ERROR("[e2_brain::Voice]::Polite");

			robot_talk(get_speech_by_name("polite_0"));
			voiceClient->waitForResult();
			return;
		}

		found = command.data.find("go hell");
		if (found!=std::string::npos)
		{
			ROS_ERROR("[e2_brain::Voice]:: System Overflow - YHBT ");
			robot_talk(get_speech_by_name("not_polite_0"));
			voiceClient->waitForResult();
			return;
		}

	}

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
	
	if(rightEyeRatio->empty())
	{
		result = 0;
	}
	else
	{
		result = getVectorOutputValue(rightEyeRatio);
	}


	
	return result;
}


//==============================================================================
//==============================================================================
int getUserLeftEyeRatioBehaviour(vector<float>* leftEyeRatio)
{
	int result = 0;
	
	if(leftEyeRatio->empty())
	{
		result = 0;
	}
	else
	{
		result = getVectorOutputValue(leftEyeRatio);
	}


	
	return result;
}


//==============================================================================
//==============================================================================
int getUserHeadPitchBehaviour(vector<int>* pitch)
{
	int result = 0;
	
	if(pitch->empty())
	{
		result = 0;
	}
	else
	{
		//Detect biggest behaviour inside vector
		result = getVectorOutputValue(pitch);
	}

	return result;
}


//==============================================================================
//==============================================================================
int getUserHeadRollBehaviour(vector<int>* roll, vector<float>* ratio)
{
	int result = 0;

	if(roll->empty())
	{
		result = 0;
	}
	else
	{
		//Detect biggest behaviour inside vector
		result = getVectorOutputValue(roll);
	}
	
	return result;
}


//==============================================================================
//==============================================================================
int getUserDistanceBehaviour(vector<int>* distance)
{
	int result = 0;
	
	if(distance->empty())
	{
		result = 0;
	}
	else
	{
		//Detect biggest behaviour inside vector
		result = getVectorOutputValue(distance);
	}

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


//============================================================================================================================================
//============================================================================================================================================
//==============================================	Service & CALLBACKS		==================================================================
//============================================================================================================================================
//============================================================================================================================================
//TODO - CALLBACKS NAVIGATION
bool start_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	initialize();
	find_user = true;
	return true;
}

bool stop_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	initialize();
	robot.action_aborted++;
	return true;
}

//======================================================================
//	Navigation Callbacks
//======================================================================
void navDoneCallback(const actionlib::SimpleClientGoalState& state,const e2_navigation::NavResultConstPtr& result)
{
	navHandlerFree = true;
	ROS_INFO("[e2_brain]:: Navigation for task %d %s ",result->action_id,state.toString().c_str());

	if(state.toString() == "SUCCEEDED")
	{
		if(result->action_id == 1)
		{
			find_user=false;
			approach_user=true;
			check_user_interested=true;
		}
		else if(result->action_id == 3)
		{
			robot.action_completed++;
			initialize();
		}

	}
	else if(state.toString() == "ABORTED")
	{
		if(result->action_id == 3)
		{
			robot.action_aborted++;
			initialize();
		}

	}
}

void navActiveCallback(){}
void navFeedbackCallback(const e2_navigation::NavFeedbackConstPtr& feed){}

//======================================================================
// Kinect Callbacks
//======================================================================
void kinectDoneCallback(const actionlib::SimpleClientGoalState& state, const kinect_motor::KinectResultConstPtr& result)
{
	//ROS_INFO("[e2_brain]::Kinect motor set point reached [exit status %d]", (int)result->status);
	kinectMotorFree = true;
}
void kinectActiveCallback(){}
void kinectFeedbackCallback(const kinect_motor::KinectFeedbackConstPtr& feed){}

//======================================================================
// Voice Callbacks
//======================================================================
void voiceDoneCallback(const actionlib::SimpleClientGoalState& state,
											 const e2_voice::VoiceResultConstPtr& result)
{
	//ROS_INFO("[e2_brain]::Speak done [exit status %s]",result->result.c_str());
	sampleCounter = 40;
	speakHandlerFree = true;
}
void voiceActiveCallback(){}
void voiceFeedbackCallback(const e2_voice::VoiceFeedbackConstPtr& feed){}

//=================================================================
// Retrieve text for a conversation using string id
//=================================================================
Speech get_speech_by_name(string name)
{
	int i=0;
	for (i=0; i < speech_size_ ; ++i)
	{
		if(strcmp(speechs_[i].id.c_str(),name.c_str()) == 0)
			break;
	}

	return speechs_[i];
}

//=================================================================
// Retrieve text for a conversation using string id
//=================================================================
Speech get_random_speech(string what)
{
	int i=0;
	int topic_tot=0;

	for (i=0; i < speech_size_ ; ++i)
	{
		// different member versions of find in the same order as above:
		std::size_t found = speechs_[i].id.find(what);
		if (found!=std::string::npos)
			topic_tot++;
	}

	srand (time(NULL));
	int rand_= rand() % topic_tot;
	std::stringstream out;
	out << what << rand_;

	return get_speech_by_name(out.str());

}

//=====================================
// Load Speech data
//=====================================
void loadSpeakData(YAML::Node& doc)
{
	speechs_ = new Speech [doc.size()];

	int i=0;
	for(YAML::const_iterator it=doc.begin();it != doc.end();++it) {
		std::string key = it->first.as<std::string>();
		speechs_[i] = it->second.as<Speech>();

		//ROS_ERROR("CARICO   %s",key.c_str());
		//ROS_INFO("id   = %s",speechs_[i].id.c_str());
		//ROS_INFO("text = %s",speechs_[i].text.c_str());
		//ROS_INFO("n_a  = %d",speechs_[i].neck_action);
		//ROS_INFO("f_a  = %d",speechs_[i].face_action);
		i++;
	}
}


//=================================================================
// Make the robot talk
//=================================================================
void robot_talk(Speech speech)
{
	if(speakHandlerFree)
	{
		speakHandlerFree = false;
		//Define goal and send it to the server side
		voiceGoal.action_id = 1;
		voiceGoal.text = speech.text;
		voiceGoal.face_action = speech.face_action;
		voiceGoal.neck_action = speech.neck_action;

		voiceClient->sendGoal(voiceGoal, &voiceDoneCallback, &voiceActiveCallback, &voiceFeedbackCallback);
	}
}
