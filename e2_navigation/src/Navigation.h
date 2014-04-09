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

//===========================
#include "e2_msgs/Goto.h"
#include "e2_msgs/Train.h"
#include "e2_msgs/Talk.h"
#include "e2_msgs/NeckAction.h"
#include "std_srvs/Empty.h"
#include "nav_msgs/Odometry.h"
//===========================

#define DETECT_TIMEOUT	 			60																// Define the time before fire a detection request
#define ABORT_TIMEOUT 				300																// Navigation timeout
#define WAIT_TIME							5

class Navigation
{

	public:
		string base_name;
		string target_name;
		string guest_name;

	    bool active_task;				//if there's an active task
	    bool path_planned;				// if the robot is following a navigation path
	    bool user_recognized;			// User recognized by facerecognition

		// Class constructors
		Navigation(string name, int rate);
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


		void OdometryCb(const nav_msgs::Odometry::ConstPtr& msg);

		// Define services
		bool Abortcallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
		bool Detectcallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
		bool Gotocallback(e2_msgs::Goto::Request& request, e2_msgs::Goto::Response& response);
		bool Startcallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
		bool Neckcallback(e2_msgs::NeckAction::Request& request, e2_msgs::NeckAction::Response& response);
		bool Traincallback(e2_msgs::Train::Request& request, e2_msgs::Train::Response& response);
		bool Talkcallback(e2_msgs::Talk::Request& request, e2_msgs::Talk::Response& response);

	private:

		ros::NodeHandle nh_;
		RobotInterface *irobot_;
		ros::Rate r_;
	    // Stand Location and speech data
		Marker *markers_;
		Speech *speechs_;

		int marker_size_,speech_size_;

	    ros::Time initial_time;
	    ros::Timer abort_timeout;
	    ros::Timer detect_timeout;

		move_base_msgs::MoveBaseGoal last_user_detection;

		// Functions
		void setUserDetection(bool status);	// Set new position for user detection

};

#endif /* NAVIGATION_H_ */
