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

#define BASE_ROTATION_ANGLE 30		// Rotation of 30 degree
#define SPEECH_COMMAND "espeak"
#define SPEECH_PARAM " -s 150 -v it "

#include "tf/tf.h"
#include "common.h"
#include <actionlib/client/simple_action_client.h>
#include <face_recognition/FaceRecognitionAction.h>
#include <face_recognition/FaceRecognitionFeedback.h>
#include <face_recognition/FaceRecognitionActionResult.h>


typedef move_base_msgs::MoveBaseGoal MBGoal;
typedef actionlib::SimpleActionClient<face_recognition::FaceRecognitionAction> FRClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RobotInterface
{
	public:
		RobotInterface();
		~RobotInterface();

		void CancelAllGoals();
		void setGoal(MBGoal goal);
		bool getBaseGoalStatus();

		void StopBase();
		void RotateNeck(char *direction);
		void RotateBase(char *direction,float angle = BASE_ROTATION_ANGLE);

		char *getBatteryStatus();
		void SpeechTalk(string text);

		geometry_msgs::Pose getRobotPose();

		bool TrainUserFace();
		bool CheckFace(string guest_user);
		void setRobotPose(geometry_msgs::Pose pose);

	private:
		MBGoal current;
		FRClient * ac_fr; 														// Face recognition
		MoveBaseClient *ac;

		string recognized_user;
		geometry_msgs::Pose robot_pose;

		void FaceRecognCb(const actionlib::SimpleClientGoalState& state, const face_recognition::FaceRecognitionResultConstPtr& result);
};

#endif /* ROBOTINTERFACE_H_ */
