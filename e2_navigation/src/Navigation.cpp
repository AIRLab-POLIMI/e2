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

	// Default config
	base_name_     = "base";
	target_name_  = "airlab";
	god_name_	   = "Lorenzo";
	guest_name_   = "Lorenzo";				// User to be detected

	en_auto_ =  false;
	active_task_ 		= false;
	path_planned_  = false;
	path_to_user_   = false;
	user_recognized_ = false;

	initial_time_ = ros::Time::now();

    nh_.param<bool>("en_auto", en_auto_, true);
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

	ROS_INFO("[Navigation]:: Auto Enabled '%s' ",(en_auto_ ? "true" : "false"));
	ROS_INFO("[Navigation]:: Neck Enabled '%s' ",(en_neck ? "true" : "false"));
	ROS_INFO("[Navigation]:: Voice Enabled '%s' ",(en_voice ? "true" : "false"));
	ROS_INFO("[Navigation]:: Train Enabled '%s' ",(en_train ? "true" : "false"));

	ROS_INFO("[Navigation]:: Default User Name set to '%s' ", guest_name_.c_str());
	ROS_INFO("[Navigation]:: Default Base location set to '%s' ",base_name_.c_str());

	ROS_INFO("[Navigation]:: Loaded Marker Config %s with %d markers", marker_config.c_str(),(int)doc_marker.size());
	ROS_INFO("[Navigation]:: Loaded Speech Config %s with %d conversations", speech_config.c_str(),(int)doc_speech.size());

	//	Enable Robot interface
	irobot_= new RobotInterface(en_neck,en_voice,en_train);
	irobot_->neck_action(2,1);		// Staigth neck position
}

Navigation::~Navigation()
{
	irobot_->neck_action(4,1); 		// Turn off neck
	irobot_->~RobotInterface();
}

//=================================================================
// Navigation Controller. Define action to be taken by the robot
//=================================================================
void Navigation::controller()
{

	// Check Battery status
	if(strcasecmp(irobot_->get_battery_status(),"LOW") == 0)
	{
		ROS_INFO("[Navigation]:: WARNING ! Battery Low, go to base to refill");
		nav_goto(base_name_);
	}
	else if(active_task_)
	{
		if(!path_planned_)
		{
			nav_goto(target_name_);
			abort_timeout_.start();
			detect_timeout_.start();
			path_planned_ = true;
		}
		else if(path_planned_)
		{
			//getNavigationStatus();
		}
	}
	/*
	else if (en_auto_)
	{
		if(!path_planned_)
		{
			nav_random_path();			// Ramdom navigation
		}
		else	 if(user_recognized_)
		{
			// Robot just find someone. Go toward him
			nav_goto_detected_user();
			user_recognized_= false;	// Set path just once
			path_to_user_ = true;			// Set following user path
		}
		else if(!path_to_user_)
		{

			user_detect("none");
		}
	}
	else
	{
		//nav_is_goal_reached();
	}
*/

}

//=================================================================
// Execute necessary actions to be performed before robot navigation plannig
//=================================================================
void Navigation::nav_newTask()
{
	ROS_INFO("[Navigator]:: New navigation task started");

	if(irobot_->train_enabled)
	{
		irobot_->robot_talk(get_speech_by_name("train"));

		// Save new user face
		if(irobot_->robot_train_user(guest_name_))
			irobot_->robot_talk(get_speech_by_name("train_success"));
		else
		{
			irobot_->robot_talk(get_speech_by_name("train_failed"));
			return;
		}

	}

	active_task_ = true;

	// Save current position as first user detection position
	initial_time_ = ros::Time::now();
	setUserDetection(true);

	abort_timeout_ = nh_.createTimer(ros::Duration(ABORT_TIMEOUT), &Navigation::nav_abortTask,this,true,false);
	detect_timeout_ = nh_.createTimer(ros::Duration(DETECT_TIMEOUT), &Navigation::user_detectTimer,this,false,false);

	irobot_->neck_action(2,2);	//	Invitation Left
	irobot_->robot_talk(get_speech_by_name("follow_me"));
	irobot_->neck_action(2,1);	//	Straight again
}

//=================================================================
// Abort current task action
//=================================================================
void Navigation::nav_abortTask()
{
	ROS_INFO("[Navigator]:: Abort Task");

	irobot_->cancell_all_goal();

	active_task_ 		= false;
	path_planned_  = false;
	path_to_user_   = false;
	user_recognized_ = false;

	abort_timeout_.stop();
	detect_timeout_.stop();
	r_.sleep();
}

//=================================================================
// Abort current task action
//=================================================================
void Navigation::nav_abortTask(const ros::TimerEvent& e)
{
	ROS_INFO("[Navigator]:: Task killed due to timeout. Go back home.");

	nav_abortTask();
	nav_goto(base_name_);
}

//=================================================================
// Navigate to goal position
//=================================================================
void Navigation::nav_goto(string name)
{
	ROS_INFO("[Navigation]:: Going to %s ",name.c_str());
	irobot_->base_setGoal(get_marker_by_name(name));
}

//=================================================================
// Navigate to goal position
//=================================================================
void Navigation::nav_goto(MBGoal goal)
{
	ROS_INFO("[Navigation]:: Going to point [x,y] :: %f, %f",goal.target_pose.pose.position.x,goal.target_pose.pose.position.y);
	irobot_->base_setGoal(goal);
}

//=================================================================
// Navigate to position given angle and distance
//=================================================================
void Navigation::nav_goto(float distance,float deg_angle)
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

	irobot_->base_setGoal(goal);
}
//=================================================================
// Navigate to unknown user
//=================================================================
void Navigation::nav_goto_detected_user()
{
	t_user user= irobot_->getDetectedUser();
	user.distance = user.distance - APPROACH_DISTANCE > APPROACH_DISTANCE ? (user.distance - APPROACH_DISTANCE) : 0 ;
	ROS_DEBUG("[Navigation]:: Detected %s at %f m delta angle %f degree",user.name.c_str(),user.distance,user.angle);
	nav_goto(user.distance- APPROACH_DISTANCE,user.angle);	// We want the robot to keep a small distance from detection point
}

//=================================================================
// Navigate using random path
//=================================================================
void Navigation::nav_random_path()
{
	MBGoal goal;

	goal.target_pose.header.frame_id = "/map";
	goal.target_pose.header.stamp = ros::Time::now();

	srand (time(NULL));

	//ROS_INFO("%d %d",map_.info.width,map_.info.height);
	int rand_point = rand() % marker_size_;

	goal.target_pose.pose.position.x = markers_[rand_point].position.x;
	goal.target_pose.pose.position.y = markers_[rand_point].position.y;
	goal.target_pose.pose.orientation.z = markers_[rand_point].orientation.z;
	goal.target_pose.pose.orientation.w = markers_[rand_point].orientation.w;

	nav_goto(goal);
}

//=================================================================
// Check if navigation goal is reached
//=================================================================
bool Navigation::nav_is_goal_reached()
{
	return irobot_->base_getStatus();
}

//=================================================================
// Wait in position till one condition is verified
//=================================================================
void Navigation::nav_wait()
{
	ROS_INFO("[Navigation]:: Waiting...");
	sleep(WAIT_TIME);
}

//=================================================================
// Get information about Navigation status
//=================================================================
void Navigation::nav_get_status()
{
	double elapsed = (ros::Time::now() - initial_time_).toSec();

	int minutes = (((int)elapsed/60)%60);
	int seconds = ((int)elapsed%60) ;

	ROS_INFO("[Navigator]:: Navigation Status");
	ROS_INFO("[Navigator]:: Time Elapsed: %d m %d s",minutes,seconds);

	if(path_planned_ && active_task_)
		ROS_INFO("[Navigator]:: Going to %s",target_name_.c_str());
	else
		ROS_INFO("[Navigator]:: Waiting for an action");
}

//=================================================================
//	Check user face and start backtract action if none detected
//=================================================================
void Navigation::user_detect(string user_name)
{
	if(irobot_->robot_check_user(user_name))
		setUserDetection(true);
}

//=================================================================
//	Fire a Detection timeout
//=================================================================
void Navigation::user_detectTimer(const ros::TimerEvent& e)
{
	ROS_INFO("[Navigator]:: Detect timeout. Check user presence.");

	//	First Stop every robot action
	irobot_->cancell_all_goal();

	// Remove current navigation goal
	path_planned_=false;
	user_recognized_=false;

	ros::Time init_detection = ros::Time::now();
	ros::Duration timeout(30.0);

	while((ros::Time::now() - init_detection < timeout) && !user_recognized_)
	{
		irobot_->base_rotate(const_cast<char *>("LEFT"));	// Rotate robot base because he should be on the robot side

		//Check user presence
		if(irobot_->robot_check_user(guest_name_))					//	Check for guest user
			setUserDetection(true);

		ros::spinOnce();
		r_.sleep();
	}

	if(active_task_ && user_recognized_)
	{
		/*
		if(irobot_->getDetectedUser().distance > WAIT_DISTANCE)
		{
			init_detection = ros::Time::now();
			ros::Duration timeout(WAIT_TIMEOUT);
			while((ros::Time::now() - init_detection < timeout) && irobot_->getDetectedUser().distance > WAIT_DISTANCE)
			{
				//Check user presence
				if(irobot_->robot_check_user(guest_name_))					//	Check for guest user
					setUserDetection(true);

				nav_wait();

				ros::spinOnce();
				r_.sleep();
			}
			if(irobot_->getDetectedUser().distance > WAIT_DISTANCE)
			{
				nav_abortTask();
				return;
			}
		}
		*/
		path_planned_=false;							//	Not usefull but to remember that	 now the controller had to recalculate new robot path to target
		user_recognized_=false;					//	User found no more interesting
	}
	else if(active_task_)								// Recover user only if there's a navigation goal to target location.
		user_recover(guest_name_);				// User not found start Backtracking procedure


	// Reset timers
	abort_timeout_.stop();
	detect_timeout_.stop();
}

//=====================================
//	Recover user using last detected position
//=====================================
void Navigation::user_recover(string user_name)
{
	ROS_INFO("[Navigator]:: Start backtracking user.");

	irobot_->base_setGoal(last_user_detection_);						//	Go to last detection position of user

	user_recognized_=false;															// Just in case

	while(!irobot_->base_getStatus() && !user_recognized_)		//	Check if reached last detection position or if the user has been found
	{

		user_detect(user_name);

		ros::spinOnce();
		r_.sleep();
	}

	if(!user_recognized_)
	{
		ROS_INFO("[Navigator]:: No user found during Backtracking. He disappear. Task Aborted. ");
		nav_abortTask();
	}
}

//=================================================================
// Set new user detection for a known user
//=================================================================
void Navigation::setUserDetection(bool status)
{
	if(status)
	{
		user_recognized_ =true;
		last_user_detection_.target_pose.header.frame_id = "map";
		last_user_detection_.target_pose.header.stamp = ros::Time::now();
		last_user_detection_.target_pose.pose = irobot_->getRobotPose();
	}else
		user_recognized_ = false;
}

//=================================================================
// Retrieve a marker position by its name
//=================================================================
MBGoal Navigation::get_marker_by_name(string name)
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
bool Navigation::marker_exist(string name)
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
string Navigation::get_speech_by_name(string name)
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
void Navigation::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_DEBUG("[Odometry]:: Odometry pose x,y,z: [%f,%f,%f]: ", msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
	irobot_->setRobotPose(msg->pose.pose);
}

//=====================================
// Map callback
//=====================================
void Navigation::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	map_=*msg;
	ROS_DEBUG("[Map]:: Received map");
}

//=====================================
// Kill everything and shutdown
//=====================================
bool Navigation::abort_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	nav_abortTask();
	return true;
}

//=====================================
// Start new navigation task
//=====================================
bool Navigation::start_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	nav_newTask();
	return true;
}

//=====================================
// Navigate to known location
//=====================================
bool Navigation::goto_callback(e2_msgs::Goto::Request& request, e2_msgs::Goto::Response& response)
{
	if(marker_exist(request.location))
	{
		nav_goto(request.location);
		response.result = true;
		return true;
	}
	response.result = false;
	return false;

}

//=====================================
// This service launch a detection face
//=====================================
bool Navigation::detect_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	user_detect("unknown");

	if(user_recognized_)
	{
		nav_goto_detected_user();
		user_recognized_ = false;
	}

	return true;
}

//=====================================
// Train callback
//=====================================
bool Navigation::train_callback(e2_msgs::Train::Request& request, e2_msgs::Train::Response& response)
{
	if(strcmp(request.username.c_str(), "") == 0)
	{
		ROS_INFO("[Navigator]:: Can't train without a username. Abort action.");
		return false;
	}

	irobot_->robot_talk(get_speech_by_name("train"));
	if(irobot_->robot_train_user(request.username.c_str()))
	{
		irobot_->robot_talk(get_speech_by_name("train_success"));
		return true;
	}
	irobot_->robot_talk(get_speech_by_name("train_failed"));
	return false;

}

//=====================================
// Service to test neck actions
//=====================================
bool Navigation::neck_callback(e2_msgs::NeckAction::Request& request, e2_msgs::NeckAction::Response& response)
{
	irobot_->neck_action(request.action,request.sub_action);
	return true;
}

//=====================================
// Service to test robot voice
//=====================================
bool Navigation::talk_callback(e2_msgs::Talk::Request& request, e2_msgs::Talk::Response& response)
{

	if(strcmp(request.text.c_str(), "") == 0)
	{
		ROS_INFO("[Navigator]:: Empty string. Cant test voice. Abort");
		return false;
	}
	irobot_->robot_talk(request.text);

	return true;
}

//=====================================
// TODO - Fix this
//=====================================
bool Navigation::auto_engage_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{

	nav_abortTask();
	//en_auto_ = true;
	nav_random_path();
	/*
	nav_random_path();			// Ramdom navigation

	while(!user_recognized_)
		user_detect(guest_name_);

	nav_goto_detected_user();
	*/
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
