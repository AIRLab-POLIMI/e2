/*=================================================
 Authors:	cristianmandelli@gmail.com 
	      	deborahzamponi@gmail.com
 Data: 17/01/2012
 Description: Brian
=================================================*/

#include <stdio.h>
#include <string.h>
#include <vector>
#include <set>
#include <utility>

//Ros Libraries
#include <brian.h>
#include <getFuzzy.h>
#include "ros/ros.h"
#include "ros/package.h"
#include "StateMachine.h"

//Ros Messages
#include "user_tracker/Com.h"
#include "head_analyzer/MoveDataMSG.h"
#include "head_analyzer/HeadDataMSG.h"
#include "head_analyzer/EyesDataMSG.h"
#include "robot_brain/WheelData.h"
#include "robot_brain/HighLevelData.h"
#include "std_msgs/String.h"

#define MAX_WHEEL_SPEED_GAP 5
#define INTERACTION_SAMPLES 60

using namespace std;

enum transaction { AGREE, DISAGREE, INTERESTED, NOT_CLOSED };
enum nodeID { APPROACH, REGARDS, REQUEST_ATTENTION, START_INTERACTION, EXPLANATION, UPSET_REQ_ATTENTION, INVITATION };

//Structures
//=============================================================================
//=============================================================================
/*struct vectorOutputValue
{
	double maxValue;
	double median;
	double minValue;
	
};*/

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

//Variables
//==============================================================================
//==============================================================================
int userDistance;
int userDistanceThreshold;
int userXPosition;
int OUTtanSpeed;
int OUTrotSpeed;

int agreeCounter = 0;
int disagreeCounter = 0;
int interestedCounter = 0;
int not_closedCounter = 0;

float userEyeLRatio;
float userEyeRRatio;
float userHeadRatio;
int userHeadPitch;
int userHeadRoll;

char userMoveClass;

userStruct userData;

StateMachine* sm;

//Control Variables
bool userPositionDataReady;
bool userEyesDataReady;
bool userMoveDataReady;
bool userHeadRatioDataReady;
bool userHeadPitchDataReady;
bool userHeadRollDataReady;

//Messages published
robot_brain::WheelData wheelDataMSG;
robot_brain::HighLevelData hldMSG;

//Protoypes
//==============================================================================
//==============================================================================
//Get data from modules
void getUserPositionData(const user_tracker::Com com);
void getUserEyesData(const head_analyzer::EyesDataMSG eyes);
void getUserHeadData(const head_analyzer::HeadDataMSG head);
void getUserMoveData(const head_analyzer::MoveDataMSG move);
void stateMachineInit();

//Compile messages
void compileMessages();

//Other functions
//vectorOutputValue getVectorOutputValue(vector<int>* vector);
//vectorOutputValue getVectorOutputValue(vector<float>* vector);
int getVectorOutputValue(vector<int>* vector);
float getVectorOutputValue(vector<float>* vector);
char detectMeaningfulMove(vector<char>* vector);
void setStatusValue(int agree, int disagree, int interested, int not_closed);
int setMaxValueBt(int a, int b, int c, int d);

//Functions
//==============================================================================
//==============================================================================
int main(int argc, char **argv)
{
    //Global Variable Initialization
    userDistance = -1;
    userEyeLRatio = -1.0;
    userEyeRRatio = -1.0;
    userHeadRatio = -1.0;
    userMoveClass = 'U';
    OUTrotSpeed = 0;
    OUTtanSpeed = 0;
    
    //Local Variables
    vectorStruct dataVectors;
    int selectedTransaction = -1;
    
    //State Machine initialization
    stateMachineInit();
    
    //Control Variable
    bool brianActivated = false;
    bool WheelSpeedMessages = false;
    bool hldMessages = false;
    bool capturing_interaction_sample = true;
    
    userPositionDataReady = false;
    userEyesDataReady = false;
    userMoveDataReady = false;
	userHeadRatioDataReady = false;
	userHeadPitchDataReady = false;
	userHeadRollDataReady = false;
    
    ros::init(argc, argv, "robot_brain");
    ros::NodeHandle nh;
    
    //Liste per lo "scambio" dati con brian
	crisp_data_list* cdl;    //lista in ingresso
	command_list * cl;       //lista in uscita
    
    //Messages subscribers
    ros::Subscriber subUserDistance = nh.subscribe("com", 100, getUserPositionData);
    ros::Subscriber subUserHeadData = nh.subscribe("headData", 10, getUserHeadData);	
    ros::Subscriber subUserEyesData = nh.subscribe("eyesData", 10, getUserEyesData);	
    ros::Subscriber subUserMoveData = nh.subscribe("moveData", 10, getUserMoveData);	
    
    //Messages Published
	ros::Publisher pubWheelData = nh.advertise<robot_brain::WheelData>("wheel_data", 100);
	ros::Publisher pubHldData = nh.advertise<robot_brain::HighLevelData>("hl_data", 100);

   
    ROS_INFO("---------------> Starting Brian...");
    string ctofFilename = ros::package::getPath("robot_brain")+"/config/brian_config/ctof.txt";
    string shape_ctofFilename = ros::package::getPath("robot_brain")+"/config/brian_config/shape_ctof.txt";
    string PredicateFilename = ros::package::getPath("robot_brain")+"/config/brian_config/predicate.ini";
    string PredicateActionsFilename = ros::package::getPath("robot_brain")+"/config/brian_config/predicateActions.ini";
    string CandoFilename = ros::package::getPath("robot_brain")+"/config/brian_config/cando.ini";
    string behaviourFilename = ros::package::getPath("robot_brain")+"/config/brian_config/behaviour.txt";
    string wantFilename = ros::package::getPath("robot_brain")+"/config/brian_config/want.txt";
    string s_ftocFilename = ros::package::getPath("robot_brain")+"/config/brian_config/s_ftoc.txt";
    string s_shapeFilename = ros::package::getPath("robot_brain")+"/config/brian_config/s_shape.txt";

   /* MrBrian *brian = new MrBrian((char*)ctofFilename.c_str(), 
                                 (char*)shape_ctofFilename.c_str(),
                                 (char*)PredicateFilename.c_str(), 
                                 (char*)PredicateActionsFilename.c_str(),
                                 (char*)CandoFilename.c_str(),
                                 (char*)behaviourFilename.c_str(), 
                                 (char*)wantFilename.c_str(), 
                                 (char*)s_ftocFilename.c_str(), 
                                 (char*)s_shapeFilename.c_str());
	ROS_INFO("!! MrBrian !! ");*/
			
	ros::Rate r(30);
	while(ros::ok())	//ROS LOOP
    {
        //==========================================================================
        //			Read informations from head_analyzer and user_tracker
        //==========================================================================
        if(capturing_interaction_sample)
        {
			//reset all vectors at each capturing interaction phase
			dataVectors.userDistanceVect.clear();
			dataVectors.userHeadRatioVect.clear();
			dataVectors.userHeadPitchVect.clear();
			dataVectors.userHeadRollVect.clear();
			dataVectors.userEyeLRatioVect.clear();
			dataVectors.userEyeRRatioVect.clear();
			dataVectors.userMovesVect.clear();
			
			if(userPositionDataReady)
			{
				dataVectors.userDistanceVect.push_back(userDistanceThreshold);
				userPositionDataReady = false;
				
				//ending of capturing data
				if(dataVectors.userDistanceVect.size() == INTERACTION_SAMPLES)
					capturing_interaction_sample = false;
				
				//Analyze head data only if userPositionDataReady is TRUE
				if(userEyesDataReady)
				{
					dataVectors.userEyeLRatioVect.push_back(userEyeLRatio);
					dataVectors.userEyeRRatioVect.push_back(userEyeRRatio);
					userEyesDataReady = false;
				}
				
				if(userMoveDataReady)
				{
					dataVectors.userMovesVect.push_back(userMoveClass);
					userMoveDataReady = false;
				}
				//--->TO DO: DEFINE HEAD_RATIO PITCH AND ROLL THRESHOLD
				/*if(userHeadRatioDataReady)
				{
					dataVectors.userHeadRatioVect.push_back(userHeadRatio);
					
					userHeadRatioDataReady = false;
				}
				
				if(userHeadPitchDataReady)
				{
					dataVectors.userHeadPitchVect.push_back(userHeadPitch);
					
					userHeadPitchDataReady = false;
				}
				
				if(userHeadRollDataReady)
				{
					dataVectors.userHeadRollVect.push_back(userHeadRoll);
					
					userHeadRollDataReady = false;
				}*/
			}
		}//end capturing interaction sample
		else //interaction analysis
		{
			userData.distance = getVectorOutputValue(&dataVectors.userDistanceVect);
			//userData.headData.ratio = getVectorOutputValue(&dataVectors.userHeadRatioVect);
			userData.headData.pitch = getVectorOutputValue(&dataVectors.userHeadPitchVect);
			userData.headData.roll = getVectorOutputValue(&dataVectors.userHeadRollVect);
			userData.eyesData.leftRatio = getVectorOutputValue(&dataVectors.userEyeLRatioVect);
			userData.eyesData.rightRatio = getVectorOutputValue(&dataVectors.userEyeRRatioVect);
			userData.movement = detectMeaningfulMove(&dataVectors.userMovesVect);
				
			//define state machine transaction
			if (userData.distance > 0) {setStatusValue(-1, 1, -1, 0); }
			else if (userData.distance = 0) {setStatusValue(1, -1, 2, -2); }
			else if (userData.distance < 0) {setStatusValue(1, -1, 1, 0); }
			
			/*if (userData.headData.pitch > 0) {setStatusValue(-1, 2, -1, 1); }
			else if (userData.headData.pitch = 0) {setStatusValue(1, -2, 2, -2); }
			else if (userData.headData.pitch < 0) {setStatusValue(0, 1, 1, 0); }
			
			if (userData.headData.roll != 0) {setStatusValue(1, -1, 2, -2); }
			else if (userData.headData.roll = 0) {setStatusValue(0, 0, 1, -2); }*/
			
			if (userData.eyesData.leftRatio > 0) {setStatusValue(-1, 1, 1, -2); }
			else if (userData.eyesData.leftRatio = 0) {setStatusValue(1, -1, 1, -1); }
			else if (userData.eyesData.leftRatio < 0) {setStatusValue(0, 1, 1, -2); }
			
			if (userData.eyesData.rightRatio > 0) {setStatusValue(-1, 1, 1, -2); }
			else if (userData.eyesData.rightRatio = 0) {setStatusValue(1, -1, 1, -1); }
			else if (userData.eyesData.rightRatio < 0) {setStatusValue(0, 1, 1, -2); }
			
			if (userData.movement == 'N') {setStatusValue(2, -2, 1, -2); }
			else if (userData.movement == 'D') {setStatusValue(-2, 2, -2, 1); }
			else if (userData.movement == 'L') {setStatusValue(-1, 1, -1, 1); }
			else if (userData.movement == 'R') {setStatusValue(-1, 1, -1, 1); }
			
			selectedTransaction = setMaxValueBt(agreeCounter, disagreeCounter, interestedCounter, not_closedCounter);
			ROS_ERROR("SelectedTransaction: %d", selectedTransaction);
			
			if (selectedTransaction >= 0)
				sm->run(selectedTransaction);
			
			//re-able capturing data phase
			capturing_interaction_sample = true;
		}
		        
        //==========================================================================
        //			Brian (motor wheels)
        //==========================================================================
        
        
        
        //==========================================================================
        //			Sending messages
        //==========================================================================
  
        ros::spinOnce();
		r.sleep();
    }    
}


//==============================================================================
//==============================================================================
int setMaxValueBt(int a, int b, int c, int d)
{
	if ((a > b) && (a > c) && (a > d)) { return AGREE; }
	else if ((b > a) && (b > c) && (b > d)) { return DISAGREE; }
	else if ((c > a) && (c > b) && (c > d)) { return INTERESTED; }
	else if ((d > a) && (d > b) && (d > c)) { return DISAGREE; }
	else {return -1; }
}


//==============================================================================
//==============================================================================
void setStatusValue(int agree, int disagree, int interested, int not_closed)
{
	agreeCounter =  agreeCounter + agree;
	disagreeCounter = disagreeCounter + disagree;
	interestedCounter = interestedCounter + interested;
	not_closedCounter = not_closedCounter + not_closed;
}


//==============================================================================
//==============================================================================
char detectMeaningfulMove(vector<char>* vector)
{	
	map<char, int> counter;
	map<char,int>::iterator it;
	
	pair<char,int> highest;
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
		cout<<(*it).first<<" => "<<(*it).second<<endl;
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

//==============================================================================
//==============================================================================
/*vectorOutputValue getVectorOutputValue(vector<float>* vector)
{
	vectorOutputValue result;
	
	multiset<float> sortedVector;
	multiset<float>::iterator it;
	multiset<float>::reverse_iterator rit;
	
	//Initializing
	int size = (int)vector->size();
	
	//Put value into multiset for maxValue, minValue and median computation
	for(int i = 0; i < vector->size(); i++)
		sortedVector.insert(vector->at(i));
	
	it = sortedVector.begin();
	result.minValue = *it;
	
	rit = sortedVector.rbegin();
	result.maxValue = *rit;
	
	//median computation
	for(it = sortedVector.begin(); it != sortedVector.end(); it++)
	{
		cout<<" "<<*it;
	}
	cout<<endl;
	
	if(size%2 != 0)			//odd number of elements
	{
		it =  sortedVector.begin();
		advance(it, ((size-1)/2));
		result.median = *it;
	}
	else 					//even number of elements
	{
		float temp1, temp2;	//variables for saving the n/2_th and the
							//(n+1)/2_th element
		it = sortedVector.begin();
		advance(it, ((size-1)/2));
		temp1 = *it;
		it = sortedVector.begin();
		advance(it, (size/2));
		temp2 = *it;
		
		result.median = (temp1+temp2)/2;
	}
	
	return result;
}


//==============================================================================
//==============================================================================
vectorOutputValue getVectorOutputValue(vector<int>* vector)
{
	vectorOutputValue result;
	
	multiset<int> sortedVector;
	multiset<int>::iterator it;
	multiset<int>::reverse_iterator rit;
	
	//Initializing
	int size = (int)vector->size();
	
	//Put value into multiset for maxValue, minValue and median computation
	for(int i = 0; i < vector->size(); i++)
		sortedVector.insert(vector->at(i));
	
	it = sortedVector.begin();
	result.minValue = *it;
	
	rit = sortedVector.rbegin();
	result.maxValue = *rit;
	
	//median computation
	cout<<"La dimensione del vettore è: "<<size<<endl;
	for(it = sortedVector.begin(); it != sortedVector.end(); it++)
	{
		cout<<" "<<*it;
	}
	cout<<endl;
	
	if(size%2 != 0)			//odd number of elements
	{
		it =  sortedVector.begin();
		advance(it, ((size-1)/2));
		result.median = *it;
	}
	else 					//even number of elements
	{
		int temp1, temp2;	//variables for saving the n/2_th and the
							//(n+1)/2_th element
		it = sortedVector.begin();
		advance(it, ((size-1)/2));
		temp1 = *it;
		it = sortedVector.begin();
		advance(it, (size/2));
		temp2 = *it;
		
		result.median = (temp1+temp2)/2;
	}
	
	return result;
}*/

//==============================================================================
//==============================================================================
void compileMessages()
{
	//Publish Wheel Data
	wheelDataMSG.tanSpeed.data = OUTtanSpeed;
	wheelDataMSG.rotSpeed.data = OUTrotSpeed;
	
	//TO_DO: Publish High Level Data
	
	
	//ROS_INFO("TanSpeed: %d   RotSpeed: %d", OUTtanSpeed, OUTrotSpeed);
}


//==============================================================================
//==============================================================================
void stateMachineInit()
{
	Node node = Node(APPROACH, "Approach", "001001", "a");
	sm = new StateMachine(node);
	
	node = Node(REGARDS, "Regards", "110101", "a");
	sm->addNode(node);
	
	node = Node(REQUEST_ATTENTION, "RequestAttention", "010101", "a");
	sm->addNode(node);
	
	node = Node(START_INTERACTION, "StartInteraction", "110011", "a");
	sm->addNode(node);
	
	node = Node(EXPLANATION, "Explanation", "111000", "a");
	sm->addNode(node);
	
	node = Node(UPSET_REQ_ATTENTION, "UpsetRequestAttention", "000111", "a");
	sm->addNode(node);
	
	node = Node(INVITATION, "Invitation", "111111", "a");
	sm->addNode(node);
	
	sm->setTransictionBt(APPROACH,START_INTERACTION, AGREE);
	sm->setTransictionBt(APPROACH,START_INTERACTION, INTERESTED);
	sm->setTransictionBt(APPROACH,REQUEST_ATTENTION, NOT_CLOSED);
	sm->setTransictionBt(APPROACH,REGARDS, DISAGREE);
	sm->setTransictionBt(START_INTERACTION,EXPLANATION, AGREE);
	sm->setTransictionBt(START_INTERACTION,EXPLANATION, INTERESTED);
	sm->setTransictionBt(START_INTERACTION,UPSET_REQ_ATTENTION, NOT_CLOSED);
	sm->setTransictionBt(START_INTERACTION,REGARDS, DISAGREE);
	sm->setTransictionBt(EXPLANATION,INVITATION, AGREE);
	sm->setTransictionBt(EXPLANATION,INVITATION, INTERESTED);
	sm->setTransictionBt(EXPLANATION,UPSET_REQ_ATTENTION, NOT_CLOSED);
	sm->setTransictionBt(UPSET_REQ_ATTENTION,EXPLANATION, INTERESTED);
	sm->setTransictionBt(UPSET_REQ_ATTENTION,REGARDS, DISAGREE);
	sm->setTransictionBt(UPSET_REQ_ATTENTION,REGARDS, NOT_CLOSED);
	sm->setTransictionBt(REQUEST_ATTENTION,START_INTERACTION, INTERESTED);
	sm->setTransictionBt(REQUEST_ATTENTION,START_INTERACTION, AGREE);
	sm->setTransictionBt(REQUEST_ATTENTION,REGARDS, DISAGREE);
	sm->setTransictionBt(REQUEST_ATTENTION,REGARDS, NOT_CLOSED);
	
	sm->printMap();
}


//==============================================================================
//==============================================================================
void getUserPositionData(const user_tracker::Com com)
{
	
    //Save user's distance from camera (z coord)
    userDistance = (int)com.comPoints.z;
    userDistanceThreshold = (int)com.distanceThreshold.data;
    userXPosition = (int)com.comPoints.x;
    
    //Check if there are still a user
    if (userDistanceThreshold > 0)
        userPositionDataReady = true;
    else
    {
        ROS_INFO("User out of camera");
        userPositionDataReady = false;
    }
    
    //ROS_INFO("PosizioneXUtente: %d", userXPosition);
}


//==============================================================================
//==============================================================================
void getUserEyesData (const head_analyzer::EyesDataMSG eyesData)
{
	userEyeLRatio = (float)eyesData.eyeLRatio.data;
	userEyeRRatio = (float)eyesData.eyeRRatio.data;
	
	if(userEyeLRatio > 0.0 && userEyeRRatio > 0.0)
		userEyesDataReady = true;
	else
		userEyesDataReady = false;
		
	//ROS_INFO("Eyes Ratio: R=%f L=%f", userEyeRRatio, userEyeLRatio);
}


//==============================================================================
//==============================================================================
void getUserMoveData (const head_analyzer::MoveDataMSG moveData)
{
	userMoveClass = (moveData.moveClassification.data).at(0);
	if(userMoveClass != 'U')
		userMoveDataReady = true;

}

//==============================================================================
//==============================================================================
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
	if( userHeadRoll > 0 )
		userHeadRollDataReady = true;
	else
		userHeadRollDataReady = false;
}
