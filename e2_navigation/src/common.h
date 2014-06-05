/*
 * common.h - AIRLab (Politecnico di Milano)
 * 
 *
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 27/feb/2014
 *
 */

#ifndef COMMON_H_
#define COMMON_H_

#include "ros/ros.h"
#include "ros/package.h"
#include "tf/tf.h"

#include <fstream>
#include <angles/angles.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <e2_voice/VoiceAction.h>
#include <kinect_motor/KinectAction.h>
#include <e2_neck_controller/NeckAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <face_recognition/FaceRecognitionAction.h>
#include <face_recognition/FaceRecognitionFeedback.h>
#include <face_recognition/FaceRecognitionActionResult.h>

using namespace std;
using namespace geometry_msgs;
using namespace e2_voice;
using namespace e2_neck_controller;
using namespace move_base_msgs;
using namespace face_recognition;
using namespace kinect_motor;

typedef move_base_msgs::MoveBaseGoal MBGoal;
typedef actionlib::SimpleActionClient<VoiceAction> VoiceClient;
typedef actionlib::SimpleActionClient<NeckAction> NeckClient;
typedef actionlib::SimpleActionClient<FaceRecognitionAction> FRClient;
typedef actionlib::SimpleActionClient<MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<KinectAction> KinectClient;

typedef struct
{
	string name;
	float distance;					// m
	float angle; 					// degree
}t_user;

#endif /* COMMON_H_ */
