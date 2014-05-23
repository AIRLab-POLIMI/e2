/*
// Authors: cristianmandelli@gmail.com, deborahzamponi@gmail.com
// Data: 15/10/2011
//
// Description: This package use the XnOpenNi driver for Kinect to detect
// AMX_USERS user into scene.
// The publicated topics are:
// - /com  com of each user identified into scene
// - /users-roi  ROI of each user identified into scene
			Here the package publish pt1(-1,-1) pt2(-1,-1) as ROI point when user 
			goes out fron the scene before that kinect launch lostUser callback."
*/

//Ros Libraries
#include "ros/ros.h"
#include "ros/package.h"
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"
#include "user_tracker/Com.h"

//Other libraries
#include <sstream>

//Openni libreries
#include <XnOpenNI.h>
#include <XnCppWrapper.h>
#include <XnCodecIDs.h>

using std::string;

#define MAX_USERS 10  /*  If you need to modify this constant you MUST mofify 
						  also nUser variable and the lenght of the com and roi
						  messages into their .msg files.
					  */
#define MAX_ROI_SAMPLE 10
int max_roi_sample = 10;
int max_user_distance = 10000;

using namespace cv;

//Structures
//==============================================================================
//==============================================================================
struct point
{
	int x;
	int y;
};

struct userData
{
	int userID;
	//User's ROI avarage point
	point headROIpt1;
	point headROIpt2;
	//User Center of Mass
	XnPoint3D com;
	//Control Variables
	int i;
	bool fullQueue;
	bool userInRange;
};

//Prototipi
//==============================================================================
//==============================================================================
//Callback
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
//tracking users
void getBestUser(XnUserID usersArray [MAX_USERS], XnUInt16 nUsers, xn::UserGenerator& generator);
//define ROI of user's head saved on pt1 and pt2
void getUserHeadROI(XnUserID nId, xn::UserGenerator& generator);
//compile messages
void compileMessages();

#define CHECK_RC(nRetVal, what)                                     \
 if (nRetVal != XN_STATUS_OK)                                        \
 {                                                                   \
	 printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
     return nRetVal;                                             \
 }

//Variables
//==============================================================================
//==============================================================================
//Contest and generators
xn::Context g_Context;
xn::UserGenerator g_UserGenerator;

//Controll
XnStatus nRetVal;
bool roiQueue;

//Users Information Structure
userData userDataArray[MAX_USERS];
userData nearestUserData;
XnUserID usersArrayCopy [MAX_USERS];

//Center of Mass
user_tracker::Com coms;

//ROI
bool firstPoint;
int nUserDetected;
int IDNearUser = -1;
int *userPixels;

//Functions
//==============================================================================
//==============================================================================
int main(int argc, char **argv)
{
	XnUserID usersArray [MAX_USERS];
	XnUInt16 nUsers = 10;
	
	//Initializations
	nUserDetected = 0;
	bool getNewNearUser = true;
	
	for (int i=0; i < MAX_USERS; i++)
	{
		userDataArray[i].i = 0;
		userDataArray[i].fullQueue = false;
		usersArray[i] = -1;

	}
	
	//Define user windows
	//cvNamedWindow("User Shape", CV_WINDOW_AUTOSIZE);

	//Ros and OpenNi initialization
	ros::init(argc, argv, "user_tracker");
	ros::NodeHandle nh;
	string configFilename = ros::package::getPath("openni_tracker")+"/openni_tracker.xml";

	nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
	nRetVal = g_UserGenerator.Create(g_Context);

	XnCallbackHandle hUserCallbacks;
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

	//User's CoM messages
	ros::Publisher pubCoM = nh.advertise<user_tracker::Com>("com", 1000);

	ros::Rate r(30);
	while(ros::ok())	//ROS LOOP
	{

		//Update all data
		g_Context.WaitAndUpdateAll();

		//If user is detected
		if (nUserDetected > 0)
		{
			//Get Best user
			if(getNewNearUser)
			{
				//Get number of users into the scene and save their ID into usersArray array
				g_UserGenerator.GetUsers(usersArray, nUsers);		
				getBestUser(usersArray, nUsers, g_UserGenerator);
				getNewNearUser = false;
			}
			else
			{
				for(int i = 0; i < MAX_USERS; i++)
				{
					if(usersArray[i] == nearestUserData.userID)
					{
						g_UserGenerator.GetCoM(nearestUserData.userID, nearestUserData.com);
						getUserHeadROI(nearestUserData.userID, g_UserGenerator);
						
						if((int)nearestUserData.com.Z != 0  && nearestUserData.com.Z < max_user_distance)
						{
							getNewNearUser = false;
							//ROS_ERROR("%d, Utente più vicino ID:%d, COM.z = %d", nUsers, nearestUserData.userID, (int)nearestUserData.com.Z);
						}
						else
							getNewNearUser = true;
					}
				}
			}
			
			if(getNewNearUser == false && nearestUserData.userID > 0)
			{
				//Compile messages
				compileMessages();
				
				ROS_INFO("Sended Message:  Best User COM(%d,%d,%d)", 
						(int)coms.comPoints.x, (int)coms.comPoints.y, (int)coms.comPoints.z);
				//Send messages
				pubCoM.publish(coms);	
			}
		}
		
		//if(waitKey(30) >= 0) break;
		//waitKey(0);
		
  	ros::spinOnce();
		r.sleep();
	
	}
	g_Context.Shutdown();
	return 0;
}

//==============================================================================
//==============================================================================
void getUserHeadROI(XnUserID nId, xn::UserGenerator& generator)
{
	//Support Variables
	int i,j;
	int row = 0;
	int column = 0;
	
	bool firstPoint = true;
	
	//User's pixels
	xn::SceneMetaData smd;
	
	//Load background image
	//string path = ros::package::getPath("user_tracker")+"/background.jpeg";
	//IplImage* background = cvLoadImage(path.c_str(), CV_LOAD_IMAGE_COLOR );
	
	generator.GetUserPixels(nId, smd);
	userPixels = (int*)smd.Data();
	
	//Getting user's head ROI
	for(i = 0; i < 320*480; i++)
	{
		if (userPixels[i] == (int)nId)
		{
			j = i;
			
			//Draw pixels into image
			row = i/320;
			column = i % 320;
			
			//First line of image
			if(firstPoint)
			{
				nearestUserData.headROIpt1.y = row;
				nearestUserData.headROIpt1.x = column*2;
				firstPoint = false;
				break;
			}
			
			/*CvScalar s;
			s.val[0] = 0;
			s.val[1] = 0;
			s.val[2] = 255;
			
			//get the (i,j) pixel value
			cvSet2D(background,row,column*2, s);
			*/
		}
	}

	//cvShowImage("User Shape", background);
}


//==============================================================================
//==============================================================================
void getBestUser(XnUserID usersArray [MAX_USERS], XnUInt16 nUsers, xn::UserGenerator& generator)
{
	for (int i = 0; i<MAX_USERS; i++)
		ROS_INFO("UserTrovati %d", usersArray[i]);

	//Find the nearest user
	userData temp;
	//Get user's ID	
	nearestUserData.userID = -1;//(int)usersArray[0];
	//Get User's COM
	nearestUserData.com.Z = max_user_distance;
	//generator.GetCoM(usersArray[0], nearestUserData.com);	
	for(int k = 0; k < nUsers; k++)  //For each detected user
	{
		//if there is nearest user
		generator.GetCoM(usersArray[k],temp.com);
		if((int)nearestUserData.com.Z > (int)temp.com.Z && (int)temp.com.Z != 0)
		{
			nearestUserData.userID = (int)usersArray[k];
			nearestUserData.com = temp.com;
		}
	}
}


//==============================================================================
//==============================================================================
void compileMessages()
{
	//Compiling COM message	
	
	coms.comPoints.x = (float)nearestUserData.com.X;
	coms.comPoints.y = (float)nearestUserData.com.Y;
	coms.comPoints.z = (float)nearestUserData.com.Z;
	
	//set distance threshold to send to robot_brain
	if((float)nearestUserData.com.Z <= 600) 
		{ coms.distanceThreshold.data = 600; }
	else if(((float)nearestUserData.com.Z > 600) && ((float)nearestUserData.com.Z <= 700))
		{ coms.distanceThreshold.data = 700; }
	else if(((float)nearestUserData.com.Z > 700) && ((float)nearestUserData.com.Z <= 800))
		{ coms.distanceThreshold.data = 800; }
	else if(((float)nearestUserData.com.Z > 800) && ((float)nearestUserData.com.Z <= 900))
		{ coms.distanceThreshold.data = 900; }
	else if(((float)nearestUserData.com.Z > 900) && ((float)nearestUserData.com.Z <= 1000))
		{ coms.distanceThreshold.data = 1000; }
	else
		{ coms.distanceThreshold.data = -1; }
	
	//Compiling head data
	coms.headPoint.x = (float)nearestUserData.headROIpt1.x;
	coms.headPoint.y = (float)nearestUserData.headROIpt1.y;
	
}

//==============================================================================
//==============================================================================
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	//Initialization of user structure
	userDataArray[nId].i = 0;
	userDataArray[nId].fullQueue = false;
	userDataArray[nId].userInRange = true;
	
	ROS_INFO("New User %d", nId);
	nUserDetected++;
}


//==============================================================================
//==============================================================================
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	ROS_INFO("Lost User %d", nId);
	userDataArray[nId].userInRange = true;
	nUserDetected--;
}
