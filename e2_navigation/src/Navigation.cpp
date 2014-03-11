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
#include <cstddef>

using namespace std;

//=================================================================
// Class Constructor
//=================================================================
Navigation::Navigation(ros::NodeHandle *nh, string marker_config,string speech_config,int rate) : irobot()
{
	Handle = nh;

	YAML::Node doc_marker,doc_speech;
	ifstream fin(marker_config.c_str());
	YAML::Parser parser(fin);
	parser.GetNextDocument(doc_marker);
	loadMarkerData(doc_marker);

	ifstream fin_speech(speech_config.c_str());
	YAML::Parser parser_speech(fin_speech);
	parser_speech.GetNextDocument(doc_speech);
	loadSpeakData(doc_speech);

	marker_size=doc_marker.size();
	speech_size=doc_speech.size();

	base_name = "base";
	guest_name = "Lorenzo";

	initial_time = ros::Time::now();
	node_rate = rate;

	active_task = false;
	path_planned = false;
	user_recognized = false;

	ROS_INFO("[Navigation]:: Default User Name set to '%s' ", guest_name.c_str());
	ROS_INFO("[Navigation]:: Default Base location set to '%s' ",base_name.c_str());

	ROS_INFO("[Navigation]:: Loaded Marker Config %s with %d markers", marker_config.c_str(),(int)doc_marker.size());
	ROS_INFO("[Navigation]:: Loaded Speech Config %s with %d conversations", speech_config.c_str(),(int)doc_speech.size());

}

Navigation::~Navigation()
{
	irobot.~RobotInterface();
}

//=================================================================
// Navigation Controller. Define action to be taken by the robot
//=================================================================
void Navigation::Controller()
{
	ros::Rate r(node_rate);

	while(ros::ok())
	{
		ros::spinOnce();

		if(strcasecmp(irobot.getBatteryStatus(),"LOW") == 0)
		{
			ROS_INFO("[Navigation]:: WARNING ! Battery Low, go to base to refill");
			NavigateTo(base_name);
		}
		else if(active_task)
		{

			if(!path_planned)
			{

				NavigateTo("airlab");

				detect_timeout.start();
				path_planned = true;

			}
			else if(path_planned)
			{
				//getNavigationStatus();
			}

		}
		else
		{	/* Looking for user */

			//NewTask();
			//irobot.NeckAction(1);

			usleep(19000000);
		}

		r.sleep();
	}

	irobot.StopBase();

}

//=================================================================
// Execute necessary actions to be performed before robot navigation plannig
//=================================================================
void Navigation::NewTask()
{
	ROS_INFO("[Navigator]:: New navigation task started");

	irobot.SpeechTalk(getSpeechById("train"));

	// Save new user face
	if(irobot.TrainUserFace())
	{
		active_task = true;
		irobot.SpeechTalk(getSpeechById("train_success"));

		abort_timeout = Handle->createTimer(ros::Duration(ABORT_TIMEOUT), &Navigation::AbortTask,this,true,false);
		detect_timeout = Handle->createTimer(ros::Duration(DETECT_TIMEOUT), &Navigation::DetectUser,this,false,false);

		// Save current position as first user detection position
		initial_time = ros::Time::now();
		setUserDetection(true);

	}
	else
	{
		active_task = false;
		irobot.SpeechTalk(getSpeechById("train_failed"));
	}

}

//=================================================================
// Abort current task action
//=================================================================
void Navigation::AbortTask(const ros::TimerEvent& e)
{
	ROS_INFO("[Navigator]:: Task killed due to timeout");

	active_task = false;
	irobot.CancelAllGoals();

	abort_timeout.stop();
	detect_timeout.stop();

	NavigateTo(base_name);

}

//=================================================================
// Navigate to goal position
//=================================================================
void Navigation::NavigateTo(string name)
{
	ROS_INFO("[Navigation]:: Going to %s stand",name.c_str());
	irobot.setGoal(getMarkerById(name));
}

//=====================================
// Check user face and start backtract action if none detected
//=====================================
void Navigation::DetectUser(const ros::TimerEvent& e)
{
	ROS_INFO("[Navigator]:: Default timeout. Check user presence.");

	path_planned=false;
	irobot.CancelAllGoals();

	ros::Time init_detection = ros::Time::now();
	ros::Duration timeout(30.0);
	ros::Rate r(node_rate);

	while((ros::Time::now() - init_detection < timeout) && !user_recognized)
	{
		irobot.RotateBase(const_cast<char *>("LEFT"),3.14);

		if(irobot.CheckFace(guest_name))
		{
			active_task=true;
			path_planned=false;
			setUserDetection(true);
		}
		ros::spinOnce();
		r.sleep();
	}

	if(user_recognized)
		user_recognized=false;
	else
		RecoverUser();			// User not found start Backtracking procedure

}

//=====================================
//	Recover user using last detected position
//=====================================
void Navigation::RecoverUser(void)
{
	ROS_INFO("[Navigator]:: Start backtracking user.");

	irobot.CancelAllGoals();
	irobot.setGoal(last_user_detection);

	ros::Rate r(node_rate);

	while(!irobot.getBaseGoalStatus() && !user_recognized)
	{
		if(irobot.CheckFace(guest_name))
		{
			active_task=true;
			path_planned=false;
			user_recognized=false;
			setUserDetection(true);
		}
		ros::spinOnce();
		r.sleep();
	}

	if(!user_recognized)
	{
		ROS_INFO("[Navigator]:: No user found while navigating in last position. He disappear. Task Aborted. ");
		Handle->createTimer(ros::Duration(0), &Navigation::AbortTask,this,true,true);
	}

}

//=================================================================
// Retrieve a marker position by its name
//=================================================================
MBGoal Navigation::getMarkerById(string name)
{

	int i=0;
	for (i=0; i < marker_size; ++i)
	{
		if(strcmp(markers[i].name.c_str(),name.c_str()) == 0)
			break;
	}

	MBGoal goal;

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = markers[i].position.x;
	goal.target_pose.pose.position.y = markers[i].position.y;
	goal.target_pose.pose.orientation.z = 0.1;
	goal.target_pose.pose.orientation.w = 0.1;

	return goal;
}

//=================================================================
// Retrieve text for a conversation using string id
//=================================================================
string Navigation::getSpeechById(string name)
{

	int i=0;
	for (i=0; i < speech_size ; ++i)
	{
		if(strcmp(speechs[i].id.c_str(),name.c_str()) == 0)
			break;
	}

	return speechs[i].text;

}

//=================================================================
// Get information about Navigation status
//=================================================================
void Navigation::getNavigationStatus()
{
	double elapsed = (ros::Time::now() - initial_time).toSec();

	int minutes = (((int)elapsed/60)%60);
	int seconds = ((int)elapsed%60) ;

	ROS_INFO("[Navigator]:: Navigation Status");
	ROS_INFO("[Navigator]:: Time Elapsed: %d m %d s",minutes,seconds);

	string status = "none";

	/*
	if(path_planned && active_task)
		status = "Going to " + base_name;
	else if()

	ROS_INFO("[Navigator]:: Going: ........ ");
	 */

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
		last_user_detection.target_pose.pose = irobot.getRobotPose();
	}else
		user_recognized = false;
}

//=================================================================
// Update Robot pose
//=================================================================
void Navigation::UpdateRobotPose(geometry_msgs::Pose pose)
{
	ROS_DEBUG("[Navigator]:: Updated Robot Position");
	irobot.setRobotPose(pose);
}

//=====================================
// Load Marker data
//=====================================
void Navigation::loadMarkerData(YAML::Node& doc)
{
	markers = new Marker [doc.size()];

	// Load Markers from map file
	for(unsigned i=0;i<doc.size();i++)
		doc[i] >> markers[i];
}

//=====================================
// Load Speech data
//=====================================
void Navigation::loadSpeakData(YAML::Node& doc)
{
	speechs = new Speech [doc.size()];

	// Load Markers from map file
	for(unsigned i=0;i<doc.size();i++)
		doc[i] >> speechs[i];

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

void operator >> (const YAML::Node& node, Marker& marker)
{
	node["id"] >> marker.id;
	node["name"] >> marker.name;
	node["position"] >> marker.position;
}

void operator >> (const YAML::Node& node, Speech& speech)
{
	node["id"] >> speech.id;
	node["text"] >> speech.text;
	node["duration"] >> speech.duration;
}
