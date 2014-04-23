/*
 * RobotInterface.h - AIRLab (Politecnico di Milano)
 *
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 27/feb/2014
 *
 */

#ifndef ROBOTINTERFACE_H_
#define ROBOTINTERFACE_H_

#define BASE_ROTATION_ANGLE 0.785		// Radians 45°
#define ERROR_DISTANCE	15						// distance greater than this will be considered as kinect error and so discarded. Mainly used for simulation bug

#include "tf/tf.h"
#include "common.h"
#include <e2_voice/VoiceAction.h>
#include <e2_neck_controller/NeckAction.h>
#include <actionlib/client/simple_action_client.h>
#include <face_recognition/FaceRecognitionAction.h>
#include <face_recognition/FaceRecognitionFeedback.h>
#include <face_recognition/FaceRecognitionActionResult.h>

using namespace geometry_msgs;
using namespace e2_voice;
using namespace e2_neck_controller;
using namespace move_base_msgs;
using namespace face_recognition;

typedef move_base_msgs::MoveBaseGoal MBGoal;
typedef actionlib::SimpleActionClient<VoiceAction> VoiceClient;
typedef actionlib::SimpleActionClient<NeckAction> NeckClient;
typedef actionlib::SimpleActionClient<FaceRecognitionAction> FRClient;
typedef actionlib::SimpleActionClient<MoveBaseAction> MoveBaseClient;

typedef struct
{
	string name;
	float distance;				//	m
	float angle; 					// degree
}t_user;

class RobotInterface
{
	public:
		bool neck_enabled;
		bool voice_enabled;
		bool train_enabled;

		RobotInterface(bool enable_neck=true,bool enable_voice=true,bool enable_train=true);
		~RobotInterface();

		void cancell_all_goal();													//	Cancel all goal - neck, voice, base, facerec

		void base_setGoal(MBGoal goal);																				// Set new goal for robot base
		bool base_getStatus();																									//	Get current state of goal
		void base_rotate(char *direction,float angle = BASE_ROTATION_ANGLE);		// Rotate Robot base
		void base_stop();																											//	Stop moving base

		void neck_action(int action, int sub_action);

		void robot_talk(string text);
		bool robot_train_user(string user_name);
		bool robot_check_user(string user_name);

		char *get_battery_status();

		Pose getRobotPose();
		void setRobotPose(geometry_msgs::Pose pose);

		void clearDetectedUser();
		t_user getDetectedUser();
		void setDetectedUser(t_user detectedUser);

private:
		MBGoal current;
		FRClient* ac_fr;
		NeckClient *ac_nc;
		VoiceClient *ac_vc;
		MoveBaseClient *ac_mb;

		t_user detected_user_;
		Pose robot_pose_;

		void voice_callback(const actionlib::SimpleClientGoalState& state, const VoiceResultConstPtr& result);
		void facerecognition_callback(const actionlib::SimpleClientGoalState& state, const FaceRecognitionResultConstPtr& result);
};

#endif /* ROBOTINTERFACE_H_ */
