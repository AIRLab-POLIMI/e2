/*
 * Navigation.cpp - AIRLab (Politecnico di Milano)
 * 
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 27/feb/2014
 *
 */

#include "Navigation.h"

using namespace std;

//=================================================================
// Class Constructor
//=================================================================
Navigation::Navigation(string name, int rate) :	nh_("~"),r_(rate)
{

    string marker_config,speech_config;
    bool en_neck,en_voice,en_train;

	nh_.param<bool>("en_neck", en_neck, true);
	nh_.param<bool>("en_voice", en_voice, true);
	nh_.param<bool>("en_train", en_train, true);

	nh_.param("marker_config", marker_config, ros::package::getPath("e2_navigation")+"/config/marker_config.yaml");
	nh_.param("speech_config", speech_config, ros::package::getPath("e2_navigation")+"/config/speech_config.yaml");

	// Load Stand positions in memory
	YAML::Node doc_marker,doc_speech;
	ifstream fin(marker_config.c_str());
	YAML::Parser parser(fin);
	parser.GetNextDocument(doc_marker);
	loadMarkerData(doc_marker);

	// Load speech data
	ifstream fin_speech(speech_config.c_str());
	YAML::Parser parser_speech(fin_speech);
	parser_speech.GetNextDocument(doc_speech);
	loadSpeakData(doc_speech);

	marker_size_=doc_marker.size();
	speech_size_=doc_speech.size();

	// Default config
	base_name = "base";
	target_name ="airlab";
	guest_name = "Lorenzo";		// User to be detected

	active_task = false;
	path_planned = false;
	user_recognized = false;

	initial_time_ = ros::Time::now();

	ROS_INFO("[Navigation]:: Neck Enabled '%s' ",(en_neck ? "true" : "false"));
	ROS_INFO("[Navigation]:: Voice Enabled '%s' ",(en_voice ? "true" : "false"));
	ROS_INFO("[Navigation]:: Train Enabled '%s' ",(en_train ? "true" : "false"));

	ROS_INFO("[Navigation]:: Default User Name set to '%s' ", guest_name.c_str());
	ROS_INFO("[Navigation]:: Default Base location set to '%s' ",base_name.c_str());

	ROS_INFO("[Navigation]:: Loaded Marker Config %s with %d markers", marker_config.c_str(),(int)doc_marker.size());
	ROS_INFO("[Navigation]:: Loaded Speech Config %s with %d conversations", speech_config.c_str(),(int)doc_speech.size());

	//	Enable Robot interface
	irobot_= new RobotInterface(en_neck,en_voice,en_train);
	irobot_->NeckAction(2,1); // Staigth neck position
}

Navigation::~Navigation()
{
	irobot_->NeckAction(4,1); 		// Turn off neck
	irobot_->~RobotInterface();
}

//=================================================================
// Navigation Controller. Define action to be taken by the robot
//=================================================================
void Navigation::Controller()
{

	while(ros::ok())
	{
		ros::spinOnce();

		// Check Battery status
		if(strcasecmp(irobot_->getBatteryStatus(),"LOW") == 0)
		{
			ROS_INFO("[Navigation]:: WARNING ! Battery Low, go to base to refill");
			NavigateTo(base_name);
		}
		else if(active_task)
		{
			if(!path_planned)
			{
				NavigateTo(target_name);
				abort_timeout_.start();
				detect_timeout_.start();
				path_planned = true;
			}
			else if(path_planned)
			{
				//getNavigationStatus();
			}

		}
		else
		{
			/* Looking for user */
		}

		r_.sleep();
	}
	irobot_->StopBase();
}

//=================================================================
// Execute necessary actions to be performed before robot navigation plannig
//=================================================================
void Navigation::NewTask()
{
	ROS_INFO("[Navigator]:: New navigation task started");

	if(irobot_->train_enabled)
	{
		irobot_->Talk(getSpeechById("train"));

		// Save new user face
		if(irobot_->TrainUserFace(guest_name))
		{
			active_task = true;
			irobot_->Talk(getSpeechById("train_success"));

			if(irobot_->train_enabled)
			{
				abort_timeout_ = nh_.createTimer(ros::Duration(ABORT_TIMEOUT), &Navigation::AbortTask,this,true,false);
				detect_timeout_ = nh_.createTimer(ros::Duration(DETECT_TIMEOUT), &Navigation::DetectTimer,this,false,false);
			}

			// Save current position as first user detection position
			initial_time_ = ros::Time::now();
			setUserDetection(true);
		}
		else
		{
			active_task = false;
			irobot_->Talk(getSpeechById("train_failed"));
		}
	}
	else
		active_task = true;

	irobot_->NeckAction(2,2);	//	Invitation Left
	irobot_->Talk(getSpeechById("follow_me"));
	irobot_->NeckAction(2,1);	//	Straight again
}

//=================================================================
// Abort current task action
//=================================================================
void Navigation::AbortTask()
{
	ROS_INFO("[Navigator]:: Abort Task");

	active_task = false;
	path_planned = false;
	irobot_->CancelAllGoals();

	if(irobot_->train_enabled)
	{
		abort_timeout_.stop();
		detect_timeout_.stop();
	}
}

//=================================================================
// Abort current task action
//=================================================================
void Navigation::AbortTask(const ros::TimerEvent& e)
{
	ROS_INFO("[Navigator]:: Task killed due to timeout. Go back home.");

	AbortTask();
	NavigateTo(base_name);
}

//=================================================================
// Navigate to goal position
//=================================================================
void Navigation::NavigateTo(string name)
{
	ROS_INFO("[Navigation]:: Going to %s ",name.c_str());
	irobot_->setGoal(getMarkerById(name));
}
//=================================================================
// Navigate to position given angle and distance
//=================================================================
void Navigation::NavigateTo(float distance,float deg_angle)
{
	ROS_INFO("[Navigation]:: Received a goal at distance %f and angle respect camera of %f ",distance,deg_angle);

	// calculate new goal
	float rad_angle =  (deg_angle * M_PI)	/ 180 ;

	double th = tf::getYaw(irobot_->getRobotPose().orientation);
	double th_new= th - rad_angle;

	MBGoal goal;

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.pose = irobot_->getRobotPose();
	goal.target_pose.pose.orientation.z = sin(th_new/2);
	goal.target_pose.pose.orientation.w = cos(th_new/2);

	//	Polar coordinate calculate delta
	double delta_x = distance * cos(th_new);
	double delta_y = distance * sin(th_new);

	goal.target_pose.pose.position.x += delta_x;
	goal.target_pose.pose.position.y += delta_y;

	irobot_->setGoal(goal);
}

//=================================================================
// Wait in position till one condition is verified
//=================================================================
void Navigation::Wait()
{
	ROS_INFO("[Navigation]:: Waiting  for user.....");

	sleep(WAIT_TIME);

}

//=================================================================
//	Fire a Detection timeout
//=================================================================
void Navigation::DetectTimer(const ros::TimerEvent& e)
{
	ROS_INFO("[Navigator]:: Detect timeout. Check user presence.");
	DetectUser();
}

//=====================================
// Check user face and start backtract action if none detected
//=====================================
void Navigation::DetectUser(void)
{
	ROS_INFO("[Navigator]:: Start detection procedure.");

	// Remove current navigation goal
	path_planned=false;
	user_recognized=false;

	irobot_->CancelAllGoals();

	ros::Time init_detection = ros::Time::now();
	ros::Duration timeout(30.0);

	while((ros::Time::now() - init_detection < timeout) && !user_recognized)
	{

		irobot_->RotateBase(const_cast<char *>("LEFT"));

		if(irobot_->CheckFace(guest_name))
			setUserDetection(true);

		ros::spinOnce();
		r_.sleep();
	}

	irobot_->CancelAllGoals();	// Force stop face detection still alive

	if(user_recognized)
	{
		path_planned=false;
		user_recognized=false;

	}
	else if(active_task)		// Recover user only if there's a navigation goal. Not used in testing
		RecoverUser();			// User not found start Backtracking procedure

}

//=====================================
//	Recover user using last detected position
//=====================================
void Navigation::RecoverUser(void)
{
	ROS_INFO("[Navigator]:: Start backtracking user.");

	irobot_->CancelAllGoals();
	irobot_->setGoal(last_user_detection);

	user_recognized=false;	// Just in case

	while(!irobot_->getBaseGoalStatus() || !user_recognized)
	{
		if(irobot_->CheckFace(guest_name))
		{
			active_task=true;
			path_planned=false;
			setUserDetection(true);
		}
		ros::spinOnce();
		r_.sleep();
	}

	if(!user_recognized)
	{
		ROS_INFO("[Navigator]:: No user found while navigating in last detected position. He disappear. Task Aborted. ");
		nh_.createTimer(ros::Duration(0), &Navigation::AbortTask,this,true,true);
	}
}

//=================================================================
// Get information about Navigation status
//=================================================================
void Navigation::getNavigationStatus()
{
	double elapsed = (ros::Time::now() - initial_time_).toSec();

	int minutes = (((int)elapsed/60)%60);
	int seconds = ((int)elapsed%60) ;

	ROS_INFO("[Navigator]:: Navigation Status");
	ROS_INFO("[Navigator]:: Time Elapsed: %d m %d s",minutes,seconds);

	if(path_planned && active_task)
		ROS_INFO("[Navigator]:: Going to %s",target_name.c_str());
	else
		ROS_INFO("[Navigator]:: Waiting for an action");
}

//=================================================================
// Set new user detection for a known user
//=================================================================
void Navigation::setUserDetection(bool status)
{
	if(status)
	{
		user_recognized =true;
		last_user_detection.target_pose.header.frame_id = "map";
		last_user_detection.target_pose.header.stamp = ros::Time::now();
		last_user_detection.target_pose.pose = irobot_->getRobotPose();
	}else
		user_recognized = false;
}

//=================================================================
// Update Robot pose
//=================================================================
void Navigation::UpdateRobotPose(geometry_msgs::Pose pose)
{
	ROS_DEBUG("[Navigator]:: Updated Robot Position");
	irobot_->setRobotPose(pose);
}

//=================================================================
// Retrieve a marker position by its name
//=================================================================
MBGoal Navigation::getMarkerById(string name)
{
	int i=0;
	for (i=0; i < marker_size_; ++i)
	{
		if(strcmp(markers_[i].name.c_str(),name.c_str()) == 0)
			break;
	}

	MBGoal goal;

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = markers_[i].position.x;
	goal.target_pose.pose.position.y = markers_[i].position.y;
	goal.target_pose.pose.orientation.z = markers_[i].orientation.z;
	goal.target_pose.pose.orientation.w = markers_[i].orientation.w;

	return goal;
}

//=================================================================
// Check if a marker exist in config file
//=================================================================
bool Navigation::MarkerExist(string name)
{
	int i=0;
	for (i=0; i < marker_size_; ++i)
	{
		if(strcmp(markers_[i].name.c_str(),name.c_str()) == 0)
			return true;
	}
	return false;
}

//=================================================================
// Retrieve text for a conversation using string id
//=================================================================
string Navigation::getSpeechById(string name)
{
	int i=0;
	for (i=0; i < speech_size_ ; ++i)
	{
		if(strcmp(speechs_[i].id.c_str(),name.c_str()) == 0)
			break;
	}
	return speechs_[i].text;
}

//=====================================
// Load Marker data
//=====================================
void Navigation::loadMarkerData(YAML::Node& doc)
{
	markers_ = new Marker [doc.size()];
	// Load Markers from map file
	for(unsigned i=0;i<doc.size();i++)
		doc[i] >> markers_[i];
}

//=====================================
// Load Speech data
//=====================================
void Navigation::loadSpeakData(YAML::Node& doc)
{
	speechs_ = new Speech [doc.size()];
	// Load Markers from map file
	for(unsigned i=0;i<doc.size();i++)
		doc[i] >> speechs_[i];
}


//=====================================
// Service definitions
//=====================================

//=====================================
// Odometry callback for robot pose update
//=====================================
void Navigation::OdometryCb(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_DEBUG("[Odometry]:: Odometry pose x,y,z: [%f,%f,%f]: ", msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
	UpdateRobotPose(msg->pose.pose);
}

//=====================================
// Kill everything and shutdown
//=====================================
bool Navigation::Abortcallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	AbortTask();
	return true;
}

//=====================================
// Start new navigation task
//=====================================
bool Navigation::Startcallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	NewTask();
	return true;
}

//=====================================
// Navigate to known location
//=====================================
bool Navigation::Gotocallback(e2_msgs::Goto::Request& request, e2_msgs::Goto::Response& response)
{
	if(MarkerExist(request.location))
	{
		NavigateTo(request.location);
		response.result = true;
		return true;
	}
	response.result = false;
	return false;

}

//=====================================
// This service launch a detection face
//=====================================
bool Navigation::Detectcallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	DetectUser();
	t_user user= irobot_->getDetectedUser();
	NavigateTo(user.distance,user.angle);
	return true;
}

//=====================================
// Train callback
//=====================================
bool Navigation::Traincallback(e2_msgs::Train::Request& request, e2_msgs::Train::Response& response)
{
	if(strcmp(request.username.c_str(), "") == 0)
	{
		ROS_INFO("[Navigator]:: Can't train without a username. Abort action.");
		return false;
	}

	irobot_->Talk(getSpeechById("train"));
	if(irobot_->TrainUserFace(request.username.c_str()))
	{
		irobot_->Talk(getSpeechById("train_success"));
		return true;
	}
	irobot_->Talk(getSpeechById("train_failed"));
	return false;

}

//=====================================
// Service to test neck actions
//=====================================
bool Navigation::Neckcallback(e2_msgs::NeckAction::Request& request, e2_msgs::NeckAction::Response& response)
{
	irobot_->NeckAction(request.action,request.sub_action);
	return true;
}

//=====================================
// Service to test robot voice
//=====================================
bool Navigation::Talkcallback(e2_msgs::Talk::Request& request, e2_msgs::Talk::Response& response)
{

	if(strcmp(request.text.c_str(), "") == 0)
	{
		ROS_INFO("[Navigator]:: Empty string. Cant test voice. Abort");
		return false;
	}
	irobot_->Talk(request.text);

	return true;
}


//=====================================
// Common operator for yaml file reading
//=====================================
void operator >> (const YAML::Node& node, Vec3& v)
{
  	node[0] >> v.x;
  	node[1] >> v.y;
  	node[2] >> v.z;
}

void operator >> (const YAML::Node& node, Vec2& v)
{
  	node[0] >> v.z;
  	node[1] >> v.w;
}

void operator >> (const YAML::Node& node, Marker& marker)
{
	node["id"] >> marker.id;
	node["name"] >> marker.name;
	node["position"] >> marker.position;
	node["orientation"] >> marker.orientation;
}

void operator >> (const YAML::Node& node, Speech& speech)
{
	node["id"] >> speech.id;
	node["text"] >> speech.text;
	node["duration"] >> speech.duration;
}
