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

#include "e2_msgs/Goto.h"
#include "e2_msgs/Train.h"
#include "e2_msgs/Talk.h"
#include "e2_msgs/NeckAction.h"
#include "std_srvs/Empty.h"
#include "nav_msgs/Odometry.h"

#define DETECT_TIMEOUT	 			60																// Define the time before fire a detection request
#define ABORT_TIMEOUT 				300																// Navigation timeout
#define WAIT_TIME							5

class Navigation
{

	public:

		// Class constructors
		Navigation(string name, int rate);
		~Navigation();

		// Navigation functions
	    void controller(); 																// Navigation controller loop

	    void nav_newTask();															// Start a new navigation task
	    void nav_abortTask(); 														// Kill a current task
	    void nav_abortTask(const ros::TimerEvent& e); 			// Kill a current task

	    void nav_get_status(); 														// Print navigation info in console
	    void nav_goto(string name);											// Navigate to known location
	    void nav_goto(float distance,float angle);					// Navigate to new position given angle and distance
	    void nav_goto_user();														//	Check user in the video frame and navigate to him
	    void nav_wait();

		void user_detect(string user_name); 								// Detect user face and check if it's the last trained person
		void user_recover(string user_name);							// Recover User following the path of last position detection
		void user_detectTimer(const ros::TimerEvent& e); 	// Timer to be fired after timeout

		// Define services
		bool abort_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
		bool detect_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
		bool goto_callback(e2_msgs::Goto::Request& request, e2_msgs::Goto::Response& response);
		bool start_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
		bool neck_callback(e2_msgs::NeckAction::Request& request, e2_msgs::NeckAction::Response& response);
		bool train_callback(e2_msgs::Train::Request& request, e2_msgs::Train::Response& response);
		bool talk_callback(e2_msgs::Talk::Request& request, e2_msgs::Talk::Response& response);
		void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);

	private:

		ros::NodeHandle nh_;
		ros::Rate r_;
	    ros::Time initial_time_;
	    ros::Timer abort_timeout_;
	    ros::Timer detect_timeout_;

		RobotInterface *irobot_;
		MoveBaseGoal last_user_detection_;

	    // Stand Location and speech data
		Marker *markers_;
		Speech *speechs_;

		int marker_size_,speech_size_;
		string base_name_,target_name_,god_name_,guest_name_;

		bool active_task_;					//if there's an active task
	    bool path_planned_;				// if the robot is following a navigation path
	    bool user_recognized_;			// User recognized by facerecognition

		void setUserDetection(bool status);	// Set new position for user detection

		// Load speech data and navigation data in memory
		void loadSpeakData(YAML::Node& doc);
		void loadMarkerData(YAML::Node& doc);

		// Usefull to check the presence of a location or a string by name
		bool marker_exist(string name);
		string get_speech_by_name(string name);
		MBGoal get_marker_by_name(string name);

};

#endif /* NAVIGATION_H_ */
