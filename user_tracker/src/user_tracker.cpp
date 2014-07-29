/*
// Authors: cristianmandelli@gmail.com, deborahzamponi@gmail.com
 *
// Data: 15/10/2011
// Modified:28/07/2014 ripani.lorenzo@gmail.com
 *
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
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"
#include "user_tracker/Com.h"

//Other libraries
#include <sstream>

//Openni libreries
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>

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
//tracking users
void getBestUser(XnUserID usersArray [MAX_USERS], XnUInt16 nUsers, xn::UserGenerator& generator);
//define ROI of user's head saved on pt1 and pt2
void getUserHeadROI(XnUserID nId, xn::UserGenerator& generator);
//compile messages
void compileMessages();


//Variables
//==============================================================================
//==============================================================================
//Contest and generators
xn::Context g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator g_UserGenerator;

XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";

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


void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	//Initialization of user structure
	userDataArray[nId].i = 0;
	userDataArray[nId].fullQueue = false;
	userDataArray[nId].userInRange = true;

	ROS_INFO("New User %d", nId);
	nUserDetected++;

	if (g_bNeedPose)
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	else
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);

}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	ROS_INFO("Lost User %d", nId);
	userDataArray[nId].userInRange = true;
	nUserDetected--;
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
	ROS_INFO("Calibration started for user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
	if (bSuccess) {
		ROS_INFO("Calibration complete, start tracking user %d", nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}
	else {
		ROS_INFO("Calibration failed for user %d", nId);
		if (g_bNeedPose)
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		else
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) {
    ROS_INFO("Pose %s detected for user %d", strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}


void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id, string const& child_frame_id) {
    static tf::TransformBroadcaster br;

    XnSkeletonJointPosition joint_position;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
    double x = -joint_position.position.X / 1000.0;
    double y = joint_position.position.Y / 1000.0;
    double z = joint_position.position.Z / 1000.0;

    XnSkeletonJointOrientation joint_orientation;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

    XnFloat* m = joint_orientation.orientation.elements;
    KDL::Rotation rotation(m[0], m[1], m[2],
    					   m[3], m[4], m[5],
    					   m[6], m[7], m[8]);
    double qx, qy, qz, qw;
    rotation.GetQuaternion(qx, qy, qz, qw);

    char child_frame_no[128];
    snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

    // #4994
    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion frame_rotation;
    frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
    change_frame.setRotation(frame_rotation);

    transform = change_frame * transform;

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
}

void publishTransforms(const std::string& frame_id) {
    XnUserID users[15];
    XnUInt16 users_count = 15;
    g_UserGenerator.GetUsers(users, users_count);

    for (int i = 0; i < users_count; ++i) {
        XnUserID user = users[i];
        if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;


        publishTransform(user, XN_SKEL_HEAD,           frame_id, "user_head");
        publishTransform(user, XN_SKEL_NECK,           frame_id, "user_neck");
        publishTransform(user, XN_SKEL_TORSO,          frame_id, "user_torso");

    }
}

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
	CHECK_RC(nRetVal, "InitFromXml");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    CHECK_RC(nRetVal, "Find depth generator");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
    	if (nRetVal != XN_STATUS_OK) {
    		nRetVal = g_UserGenerator.Create(g_Context);
    		CHECK_RC(nRetVal, "Find user generator");
    	}

    XnCallbackHandle hUserCallbacks;
    	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);

    		XnCallbackHandle hCalibrationCallbacks;
    		g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

    		if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
    			g_bNeedPose = TRUE;
    			if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
    				ROS_INFO("Pose required, but not supported");
    				return 1;
    			}

    			XnCallbackHandle hPoseCallbacks;
    			g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);

    			g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
    		}

    		g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

    		nRetVal = g_Context.StartGeneratingAll();
    		CHECK_RC(nRetVal, "StartGenerating");


	//User's CoM messages
	ros::Publisher pubCoM = nh.advertise<user_tracker::Com>("com", 1000);

	ros::Rate r(30);

    ros::NodeHandle pnh("~");
    string frame_id("openni_depth_frame");
    pnh.getParam("camera_frame_id", frame_id);

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
				
				ROS_DEBUG("Sended Message:  Best User COM(%d,%d,%d)",(int)coms.comPoints.x, (int)coms.comPoints.y, (int)coms.comPoints.z);
				//Send messages
				pubCoM.publish(coms);	
			}
		}
		
		//if(waitKey(30) >= 0) break;
		//waitKey(0);
		publishTransforms(frame_id);
		
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
		ROS_DEBUG("UserTrovati %d", usersArray[i]);

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
