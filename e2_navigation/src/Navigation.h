/*
 * Navigation.h - AIRLab (Politecnico di Milano)
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

#include "std_srvs/Empty.h"
#include "e2_msgs/Goto.h"
#include "e2_msgs/Train.h"
#include "e2_msgs/Talk.h"
#include "e2_msgs/NeckAction.h"
#include "e2_msgs/MotorAngle.h"

#include <user_tracker/Com.h>
#include <tf/transform_listener.h>

#include <stdlib.h>

#define APPROACH_DISTANCE		0.5 					// Define distance where robot had to place once detected a user (m)
#define DETECT_TIMEOUT	 		30						// Define the time before fire a detection request
#define ABORT_TIMEOUT 			300						// Navigation timeout
#define WAIT_TIMEOUT			30						//	Min time the robot will wait in position before abort task
#define WAIT_DISTANCE			1						//	Min distance the robot will stop to wait user
#define WAIT_TIME				5						//	Time the robot wait in position

typedef struct user_detected
{
	bool detected;
	float angle;		// angle respect center of camera
	float distance;  	// center respect center of camera
}user_detected;

class Navigation
{

	public:

		// Class constructors
		Navigation(string name, int rate);
		~Navigation();

		// Navigation functions
	    void ActionController(); 																// Navigation controller loop

	    void NavigateTarget();							 							// Start a new navigation task
	    void LookingUser();															// Start looking for user in the ambient
	    void ApproachUser();

	    void ActionAbort(); 															// Kill a current action
	    void ActionAbort(const ros::TimerEvent& e); 				// Kill a current action for timer
	    bool isActionCompleted();												// Check if an action is completed
	    void ActionReset();															// Reset navigation status

	    void getNavStatus(); 														// Print navigation info in console

		// Define services
		bool abort_service(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
		bool detect_service(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
		bool goto_service(e2_msgs::Goto::Request& request, e2_msgs::Goto::Response& response);

		bool kinect_service(e2_msgs::MotorAngle::Request& request, e2_msgs::MotorAngle::Response& response);
		bool neck_service(e2_msgs::NeckAction::Request& request, e2_msgs::NeckAction::Response& response);
		bool train_service(e2_msgs::Train::Request& request, e2_msgs::Train::Response& response);
		bool talk_service(e2_msgs::Talk::Request& request, e2_msgs::Talk::Response& response);

		bool approach_user_service(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
		bool find_user_service(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
		bool navigate_target_service(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

		void face_callback(const user_tracker::ComConstPtr& msg);
		void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);


	private:

		bool abort;							// to quit loop

		user_detected userdetected_;

		ros::NodeHandle nh_;
		ros::Rate r_;
	    ros::Time initial_time_;
	    ros::Timer abort_timeout_;
	    ros::Timer detect_timeout_;

		RobotInterface *irobot_;
		MoveBaseGoal last_user_detection_;

		// Services & subscribers
		ros::Subscriber odom_sub_;
		ros::Subscriber face_sub_;
		ros::ServiceServer abort_service_;
		ros::ServiceServer navigate_target_service_;
		ros::ServiceServer approach_user_service_;
		ros::ServiceServer find_user_service_;
		ros::ServiceServer goto_service_;
		ros::ServiceServer detect_service_;
		ros::ServiceServer talk_service_;
		ros::ServiceServer train_service_;
		ros::ServiceServer neck_service_;
		ros::ServiceServer motor_service_;

	    // Stand Location and speech data
		Marker *markers_;
		Speech *speechs_;

		int marker_size_,speech_size_;
		string base_name_,target_name_,guest_name_;

		int pass_count_;

		// Behaviours
		bool approach_user_;
		bool find_user_;				//	If true the robot will start to randomly navigate in the ambient looking for people
		bool navigate_target;			//if there's an active task

	    bool path_planned_;				// if the robot is following a navigation path
	    bool path_to_user_;				// true if the robot is following a path to reach a user
	    bool user_recognized_;			// User recognized by facerecognition
	    bool action_completed_;

	    tf::TransformListener listener_;

		void setUserDetection(bool status);	// Set new position for user detection

		// Load speech data and navigation data in memory
		void loadSpeakData(YAML::Node& doc);
		void loadMarkerData(YAML::Node& doc);

		// Usefull to check the presence of a location or a string by name
		bool marker_exist(string name);
		string get_speech_by_name(string name);
		string get_random_speech(string what);
		MBGoal get_marker_by_name(string name);

	    void nav_goto(string name);											// Navigate to known location
	    void nav_goto(MBGoal goal);											// Navigate to known location
	    void nav_goto(float distance,float angle);					// Navigate to new position given angle and distance
	    void nav_goto_detected_user(t_user user);				//	go to the last position of detected user
	    bool nav_is_goal_reached();											//	check if navigatation goal is reached
	    void nav_random_path();													//	Create a random navigation path
	    void nav_wait();

	    void user_wait();
		void user_detect(string user_name); 								// Detect user face and check if it's the last trained person
		void user_recover(string user_name);							// Recover User following the path of last position detection
		void user_detectTimer(const ros::TimerEvent& e); 	// Timer to be fired after timeout
		void user_clear();									// Delete user data

};

#endif /* NAVIGATION_H_ */
