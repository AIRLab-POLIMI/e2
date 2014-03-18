/*
 * Navigation.h - AIRLab (Politecnico di Milano)
 * 
 * description
 *
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 27/feb/2014
 *
 */

#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include "RobotInterface.h"
#include "yaml-operator.h"

#define DETECT_TIMEOUT	 			60																// Define the time before fire a detection request
#define ABORT_TIMEOUT 				300																// Navigation timeout

class Navigation
{

	public:
		Navigation(ros::NodeHandle *nh, string marker_config,string speech_config,int rate,bool en_neck,bool en_voice,bool en_train);
		~Navigation();

		void loadSpeakData(YAML::Node& doc);
		void loadMarkerData(YAML::Node& doc);

		string getSpeechById(string name);
		bool MarkerExist(string name);
		MBGoal getMarkerById(string name);

	    void NewTask();
	    void AbortTask();
	    void AbortTask(const ros::TimerEvent& e);

	    void Controller();

		void DetectUser(const ros::TimerEvent& e);									// Check for a user in faces database
		void RecoverUser(void);																		// Recover User following the path of last position detection

		void NavigateTo(string name);

		void getNavigationStatus();
		void setUserDetection(bool status);
		void UpdateRobotPose(geometry_msgs::Pose pose);

		string base_name;
		string guest_name;

	private:
	    bool active_task;
	    bool path_planned;
	    bool user_recognized;
	    bool detect_enabled;

	    int node_rate;
	    int marker_size,speech_size;

		ros::Time initial_time;

		Marker *markers;
		Speech *speechs;
		RobotInterface irobot;

	    ros::NodeHandle *Handle;

	    ros::Timer abort_timeout;
	    ros::Timer detect_timeout;
	    ros::Timer recover_timeout;

		move_base_msgs::MoveBaseGoal last_user_detection;

};

#endif /* NAVIGATION_H_ */
