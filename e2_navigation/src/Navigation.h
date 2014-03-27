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
#define WAIT_TIME							5

class Navigation
{

	public:
		string base_name;
		string target_name;
		string guest_name;

		RobotInterface irobot;

		// Class constructors
		Navigation(ros::NodeHandle *nh, string marker_config,string speech_config,int rate,bool en_neck,bool en_voice,bool en_train);
		~Navigation();

		// Load speech data and navigation data in memory
		void loadSpeakData(YAML::Node& doc);
		void loadMarkerData(YAML::Node& doc);

		// Usefull to check the presence of a location or a string by name
		bool MarkerExist(string name);
		string getSpeechById(string name);
		MBGoal getMarkerById(string name);

		// Navigation functions
	    void NewTask();					// Start a new navigation task
	    void AbortTask(); 				// Kill a current task
	    void AbortTask(const ros::TimerEvent& e); // Kill a current task

	    void Controller(); 				// Navigation controller loop

	    void NavigateTo(string name);	// Navigate to known location
	    void getNavigationStatus(); 	// Print navigation info in console
	    void Wait();

		void DetectUser(void); 			// Detect user face and check if it's the last trained person
		void RecoverUser(void);			// Recover User following the path of last position detection
		void DetectTimer(const ros::TimerEvent& e); // Timer to be fired after timeout

		void UpdateRobotPose(geometry_msgs::Pose pose);

	private:
	    bool active_task;				//if there's an active task
	    bool path_planned;				// if the robot is following a navigation path
	    bool user_recognized;			// User recognized by facerecognition
	    bool train_enabled;			// If face detection is enabled

	    int node_rate;
	    int marker_size,speech_size;

	    // Stand Location and speech data
		Marker *markers;
		Speech *speechs;

	    ros::NodeHandle *Handle;

	    ros::Time initial_time;
	    ros::Timer abort_timeout;
	    ros::Timer detect_timeout;

		move_base_msgs::MoveBaseGoal last_user_detection;

		// Functions
		void setUserDetection(bool status);	// Set new position for user detection

};

#endif /* NAVIGATION_H_ */
