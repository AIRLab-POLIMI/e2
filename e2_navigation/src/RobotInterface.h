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

#include "tf/tf.h"
#include "common.h"
#include <e2_voice/VoiceAction.h>
#include <e2_neck_controller/NeckAction.h>
#include <actionlib/client/simple_action_client.h>
#include <face_recognition/FaceRecognitionAction.h>
#include <face_recognition/FaceRecognitionFeedback.h>
#include <face_recognition/FaceRecognitionActionResult.h>


typedef move_base_msgs::MoveBaseGoal MBGoal;
typedef actionlib::SimpleActionClient<e2_voice::VoiceAction> VoiceClient;
typedef actionlib::SimpleActionClient<e2_neck_controller::NeckAction> NeckClient;
typedef actionlib::SimpleActionClient<face_recognition::FaceRecognitionAction> FRClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

typedef struct
{
	string name;
	int distance;
}t_user;

class RobotInterface
{
	public:
		RobotInterface(bool enable_neck=true,bool enable_voice=true,bool enable_train=true);
		~RobotInterface();

		void CancelAllGoals();
		void setGoal(MBGoal goal);
		bool getBaseGoalStatus();

		void StopBase();
		void RotateNeck(char *direction);
		void RotateBase(char *direction,float angle = BASE_ROTATION_ANGLE);
		void NeckAction(int action, int sub_action);

		char *getBatteryStatus();
		void Talk(string text);

		bool TrainUserFace(string user_name);
		bool CheckFace(string guest_user);

		geometry_msgs::Pose getRobotPose();
		void setRobotPose(geometry_msgs::Pose pose);

	private:

		MBGoal current;
		FRClient * ac_fr; 														// Face recognition
		NeckClient *ac_nc;
		VoiceClient *ac_vc;
		MoveBaseClient *ac_mb;

		bool neck_enabled;
		bool voice_enabled;
		bool train_enabled;

		t_user detected_user;
		geometry_msgs::Pose robot_pose;

		void VoiceCB(const actionlib::SimpleClientGoalState& state, const e2_voice::VoiceResultConstPtr& result);
		void FaceRecognCB(const actionlib::SimpleClientGoalState& state, const face_recognition::FaceRecognitionResultConstPtr& result);
};

#endif /* ROBOTINTERFACE_H_ */
