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
#define SPEECH_DELAY		30						// Time between each speech
#define ERROR_DISTANCE	8							// distance greater than this will be considered as kinect error and so discarded. Mainly used for simulation bug

#include "common.h"

class RobotInterface
{
	public:
		bool neck_enabled;
		bool voice_enabled;
		bool train_enabled;

		RobotInterface(bool enable_neck=true,bool enable_voice=true,bool enable_train=true);
		~RobotInterface();

		void cancell_all_goal();													//	Cancel all goal - neck, voice, base, facerec

		string base_getStatus();																								//	Get current state of goal
		void base_setGoal(MBGoal goal);																				// Set new goal for robot base
		void base_rotate(char *direction,float angle = BASE_ROTATION_ANGLE);		// Rotate Robot base
		void base_stop();																											//	Stop moving base

		void kinect_motor(float angle);																					// rotate kinect

		void neck_action(int action, int sub_action);

		void robot_talk(string text,bool force = false);
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
		ros::NodeHandle nh_;
	    ros::Publisher kinect_pub_;

		t_user detected_user_;
		Pose robot_pose_;
		ros::Time last_speech_;

		void voice_callback(const actionlib::SimpleClientGoalState& state, const VoiceResultConstPtr& result);
		void facerecognition_callback(const actionlib::SimpleClientGoalState& state, const FaceRecognitionResultConstPtr& result);
};

#endif /* ROBOTINTERFACE_H_ */
