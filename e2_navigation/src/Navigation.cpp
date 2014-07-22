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
Navigation::Navigation(string name, int rate) :	nh_("~"), r_(rate)
{

    string marker_config,speech_config;
    bool en_neck,en_voice,en_train,en_kinect;

	// Default config
	base_name_     = "base";
	target_name_  = "target";
	guest_name_   = "guest";				// User to be detected

	pass_count_= 0;
	delay_detect = 0;

	find_user_ =  false;
	navigate_target = false;
	path_planned_  = false;
	path_to_user_   = false;
	user_recognized_ = false;
	action_aborted_ = false;
	action_completed_ = false;


	userdetected_.detected = false;
	userdetected_.angle = 0;
	userdetected_.distance = 0;

	initial_time_ = ros::Time::now();

    nh_.param<bool>("en_auto", find_user_, true);
	nh_.param<bool>("en_neck", en_neck, true);
	nh_.param<bool>("en_voice", en_voice, true);
	nh_.param<bool>("en_train", en_train, true);
	nh_.param<bool>("en_kinect", en_kinect, true);

	nh_.param("marker_config", marker_config, ros::package::getPath("e2_config")+"/map_config/sim_marker_config.yaml");
	nh_.param("speech_config", speech_config, ros::package::getPath("e2_config")+"/speak_config/speech_config.yaml");

	//	Suscribers
	face_sub_= nh_.subscribe("/com", 10,&Navigation::face_callback,this);
	odom_sub_= nh_.subscribe("/odom", 10,&Navigation::odometry_callback,this);

	// Enable Services
	abort_service_ = nh_.advertiseService(name+"/abort",&Navigation::abort_service,this);

	find_user_service_ = nh_.advertiseService(name+"/find_user",&Navigation::find_user_service,this);
	approach_user_service_= nh_.advertiseService(name+"/approach_user",&Navigation::approach_user_service,this);
	navigate_target_service_ = nh_.advertiseService(name+"/navigate_target",&Navigation::navigate_target_service,this);

	goto_service_ = nh_.advertiseService(name+"/test_goto",&Navigation::goto_service,this);
	detect_service_ = nh_.advertiseService(name+"/test_detect",&Navigation::detect_service,this);
	talk_service_ = nh_.advertiseService(name+"/test_voice",&Navigation::talk_service,this);
	train_service_ = nh_.advertiseService(name+"/test_train",&Navigation::train_service,this);
	neck_service_ = nh_.advertiseService(name+"/test_neck",&Navigation::neck_service,this);
	motor_service_ = nh_.advertiseService(name+"/test_kinect_motor",&Navigation::kinect_service,this);

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

	ROS_ERROR("[Navigation]:: Auto Enabled '%s' ",(find_user_ ? "true" : "false"));
	ROS_ERROR("[Navigation]:: Neck Enabled '%s' ",(en_neck ? "true" : "false"));
	ROS_ERROR("[Navigation]:: Voice Enabled '%s' ",(en_voice ? "true" : "false"));
	ROS_ERROR("[Navigation]:: Train Enabled '%s' ",(en_train ? "true" : "false"));

	ROS_ERROR("[Navigation]:: Default User Name set to '%s' ", guest_name_.c_str());
	ROS_ERROR("[Navigation]:: Default Base location set to '%s' ",base_name_.c_str());

	ROS_ERROR("[Navigation]:: Loaded Marker Config %s with %d markers", marker_config.c_str(),(int)doc_marker.size());
	ROS_ERROR("[Navigation]:: Loaded Speech Config %s with %d conversations", speech_config.c_str(),(int)doc_speech.size());

	//	Enable Robot interface
	irobot_= new RobotInterface(en_neck,en_voice,en_train,en_kinect);

	// Staigth neck and clear face
	irobot_->neck_action(2,1);
	irobot_->neck_action(1,1);
	irobot_->kinect_action(15);
}

Navigation::~Navigation()
{
	irobot_->neck_action(4,1); 		// Turn off neck
	irobot_->~RobotInterface();
}

//=================================================================
// Navigation Controller. Define action to be taken by the robot
//=================================================================
void Navigation::ActionController()
{
	/*==================================================================
		Checking battery status
	==================================================================*/
	if(strcasecmp(irobot_->get_battery_status(),"LOW") == 0)
	{
		ROS_ERROR("[Navigation]:: WARNING ! Battery Low, go to base to refill");

		irobot_->robot_talk(get_speech_by_name("battery_empty"));
		nav_goto(base_name_);
	}
	/*==================================================================
		Starting new Task - Navigation Base + detection
	==================================================================*/
	else if(navigate_target)
	{
		if(!path_planned_)
		{
			ROS_ERROR("[Navigation]:: Path planned. I'm going home !");

			// Clear Face
			irobot_->neck_action(1,1);
			irobot_->kinect_action(15);

			// Start Timer
			abort_timeout_.start();
			detect_timeout_.start();

			// Go Home....Bzzzz
			nav_goto(target_name_);
			path_planned_ = true;
		}
		else if(path_planned_)
		{
			// If navigation is aborted the goal is blocked by an obstacle so robot will ask to pass
			if(strcmp(irobot_->base_getStatus().c_str(),"ABORTED") == 0)
			{
				if(pass_count_> 1 )
				{
					irobot_->neck_action(1,3); 	// Angry face
					irobot_->robot_talk(get_speech_by_name("abort"),true);

					ActionAbort();
				}
				else
				{

					irobot_->robot_talk(get_speech_by_name("pass"),true);
					nav_wait();

					path_planned_= false;		// Force to set goal again
					pass_count_++;
				}
			}
			else if(strcmp(irobot_->base_getStatus().c_str(),"SUCCEEDED")==0)
			{
				ros::Time init_detection = ros::Time::now();
				ros::Duration timeout(20.0);
				user_recognized_ = false;

				// Stop Timer
				abort_timeout_.stop();
				detect_timeout_.stop();

				while((ros::Time::now() - init_detection < timeout) && !user_recognized_)
				{
					irobot_->base_rotate(const_cast<char *>("LEFT"));	// Rotate robot base because he should be on the robot side

					//Check user presence
					user_detect(guest_name_);

					ros::spinOnce();
					r_.sleep();
				}

				irobot_->base_stop();

				if(user_recognized_)
				{
					irobot_->neck_action(1,2); 	// happy face
					irobot_->neck_action(1,4);	// Make a Bow
					irobot_->robot_talk(get_speech_by_name("complete"),true);

					action_completed_ = true;
				}
				else
				{
					irobot_->neck_action(1,3); 	// angry face
					irobot_->robot_talk(get_speech_by_name("abort"),true);

					action_aborted_ = true;
				}
			}
		}
	}
	/*==================================================================
		Start Navigating in auto mode looking for user
	==================================================================*/
	else if(find_user_) //TODO
	{

		// Plan a Random Path to find people
		if(!path_planned_)
		{
			irobot_->neck_action(1,1); 	// norm face
			nav_random_path();			// Ramdom navigation

			path_planned_ = true;
		}// Once planned we need to recognize user's faces
		else
		{
			user_detect("unknown");

			if(user_recognized_ && userdetected_.detected)
			{
				ROS_ERROR("[Navigation]:: Ho trovato qualcosa !");

				if(userdetected_.distance < 1.5 || irobot_->getDetectedUser().distance < 1.5)
				{
					ROS_INFO("[Navigation]:: User in front of me !");
					irobot_->neck_action(1,2); 	// happy face

					action_completed_ = true;
				}
				else if(!path_to_user_)
				{
					ROS_INFO("[Navigation]:: User still distant (%f-%f)(%f-%f) !",userdetected_.distance,userdetected_.angle,irobot_->getDetectedUser().distance,irobot_->getDetectedUser().angle);
					nav_goto(0.4,irobot_->getDetectedUser().angle); // slow aproach
					path_to_user_ = true; // Set following user path

				}
			}


			if(strcmp(irobot_->base_getStatus().c_str(),"ABORTED")==0)
			{
				if(path_to_user_)
				{
					ROS_INFO("[Navigation]:: Non posso avvicinarmi alla persona (dist %f).",userdetected_.distance);
				}
				else
				{
					path_planned_ = false;
					ActionAbort();
				}
			}
			else if(strcmp(irobot_->base_getStatus().c_str(),"SUCCEEDED")==0 )
			{

				if(userdetected_.detected && path_to_user_)
				{
					if(userdetected_.distance < 1.5)
					{
						find_user_= false;
						action_completed_ = true;
						irobot_->neck_action(1,2); 	// happy face
						ROS_ERROR("[Navigation]:: Sono davanti una persona !");
					}
					else
					{
						ROS_ERROR("[Navigation]:: Sono arrivato ma la pesona è lontana!");
					}

				}
				else
				{
					path_planned_ = false;
					path_to_user_= false;
				}

			}

		}

		user_clear();
		irobot_->clearDetectedUser();


	}
	/*==================================================================
		Navigate to approach a user detected TODO
	==================================================================*/
	else if(approach_user_)
	{

		if(userdetected_.detected)
		{
			if(!path_planned_)
			{
				// if distance is lower than 1 meter or is not too far from center camera the robot stay on place
				if(userdetected_.distance < 1.5)
				{
					ROS_INFO("[Navigation]:: User at distance %f and %f deg from camera. No action Done ! ",userdetected_.distance,userdetected_.angle);
					action_completed_ = true;
				}else{

					nav_goto(0.3,0);
					path_planned_ = true;
				}
			}
			else if(strcmp(irobot_->base_getStatus().c_str(),"ABORTED")==0)
			{
				path_planned_ = false;
			}
			else if(strcmp(irobot_->base_getStatus().c_str(),"SUCCEEDED")==0 && userdetected_.distance < 1.5)
			{
				action_completed_ = true;
			}else
				path_planned_= false;
		}
	}
}

//=================================================================
// Abort current task action
//=================================================================
void Navigation::ActionAbort()
{
	ROS_ERROR("[Navigation]:: Abort Task");
	ActionReset();
	action_aborted_ = true;
}

//=================================================================
// Abort current taskpath_to_user_ action
//=================================================================
void Navigation::ActionAbort(const ros::TimerEvent& e)
{
	ROS_ERROR("[Navigation]:: Task killed due to timeout. Go back home.");
	ActionAbort();

}

//=================================================================
// Reset current navigation status
//=================================================================
void Navigation::ActionReset()
{
	ROS_ERROR("[Navigation]:: Reset Navigation");

	irobot_->cancell_all_goal();

	pass_count_= 0;
	delay_detect = 0;

	navigate_target = false;
	find_user_= false;
	approach_user_ = false;

	action_completed_ = false;
	action_aborted_ = false;

	path_planned_  = false;
	path_to_user_   = false;
	user_recognized_ = false;

	abort_timeout_.stop();
	detect_timeout_.stop();

	sleep(2);	//	Sleep a bit

	irobot_->neck_action(2,1);		// Staigth neck position
	irobot_->neck_action(1,1);

}

//=================================================================
// Execute necessary actions to be performed before robot navigation plannig
//=================================================================
void Navigation::NavigateTarget()
{
	ROS_ERROR("[Navigation]:: New navigation task started");

	irobot_->neck_action(1,2); 	// happy face

	if(irobot_->train_enabled)
	{
		irobot_->robot_talk(get_speech_by_name("train_init"));

		// Save new user face
		if(irobot_->robot_train_user(guest_name_))
		{
			irobot_->neck_action(1,2);	// happy
			irobot_->robot_talk(get_speech_by_name("train_success"),true);
		}
		else
		{
			irobot_->neck_action(1,3);	// angry
			irobot_->robot_talk(get_speech_by_name("train_failed"),true);
			ActionReset();
			return;
		}
	}

	navigate_target = true;

	// Save current position as first user detection position
	initial_time_ = ros::Time::now();
	setUserDetection(true);
	nav_wait();
	irobot_->neck_action(2,2);	//	Invitation Left
	nav_wait();

	abort_timeout_ = nh_.createTimer(ros::Duration(ABORT_TIMEOUT), &Navigation::ActionAbort,this,true,false);
	detect_timeout_ = nh_.createTimer(ros::Duration(DETECT_TIMEOUT), &Navigation::user_detectTimer,this,true,false);

	irobot_->neck_action(2,1);	//	Straight again
	irobot_->neck_action(1,1);	//	Norm face

	irobot_->robot_talk(get_speech_by_name("follow_me"),true);

	nav_wait();

}

//=================================================================
// Action status functions
//=================================================================
bool Navigation::isActionAborted()
{
	return action_aborted_ ;
}

bool Navigation::isActionCompleted()
{
	return action_completed_ ;
}

//=================================================================
// Enable start Looking for user
//=================================================================
void Navigation::ApproachUser()
{
	ROS_ERROR("[Navigation]:: Approach user");

	approach_user_=true;
}

//=================================================================
// Enable start Looking for user
//=================================================================
void Navigation::LookingUser()
{
	ROS_ERROR("[Navigation]:: Looking for user");

	find_user_=true;
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

//	ROS_INFO("th: %f",th);
//	ROS_INFO("th_new: %f",th_new);
//	ROS_INFO("d_x: %f",delta_x);
//	ROS_INFO("d_y: %f",delta_y);

	irobot_->base_setGoal(goal);
}

//=================================================================
// Navigate to unknown user
//=================================================================
void Navigation::nav_goto_detected_user(t_user user)
{

	ROS_INFO("[Navigation]:: Detected %s at %f m delta angle %f degree",user.name.c_str(),user.distance,user.angle);

	user.distance = user.distance - APPROACH_DISTANCE > APPROACH_DISTANCE ? (user.distance - APPROACH_DISTANCE) : 0 ;

	nav_goto(user.distance,user.angle);	// We want the robot to keep a small distance from detection point
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
	if(strcmp(irobot_->base_getStatus().c_str(),"SUCCEEDED")==0)
	{
		return true;
	}

	return false;
}

//=================================================================
// Wait in position till one condition is verified
//=================================================================
void Navigation::nav_wait()
{
	ROS_INFO("[Navigation]:: Waiting a bit...");
	sleep(WAIT_TIME);
}

//=================================================================
// Get information about Navigation status
//=================================================================
void Navigation::getNavStatus()
{
	double elapsed = (ros::Time::now() - initial_time_).toSec();

	int minutes = (((int)elapsed/60)%60);
	int seconds = ((int)elapsed%60) ;

	ROS_INFO("[Navigation]:: Navigation Status");
	ROS_INFO("[Navigation]:: Time Elapsed: %d m %d s",minutes,seconds);

	if(path_planned_ && navigate_target)
		ROS_INFO("[Navigation]:: Going to %s",target_name_.c_str());
	else
		ROS_INFO("[Navigation]:: Waiting for an action");
}

//=================================================================
//	Clear data of detected user
//=================================================================
void Navigation::user_clear()
{
	userdetected_.detected = false;
	userdetected_.angle = 0;
	userdetected_.distance = 0;
}

//=================================================================
//	Check user face and start
//=================================================================
void Navigation::user_detect(string user_name)
{
	if(irobot_->robot_check_user(user_name))
		setUserDetection(true);
}

//=================================================================
//	Check user face and start
//=================================================================
void Navigation::user_wait()
{
	ros::Time init_detection = ros::Time::now();
	ros::Duration timeout(15.0);

	if(irobot_->getDetectedUser().distance > WAIT_DISTANCE)
	{
		init_detection = ros::Time::now();
		ROS_ERROR("[Navigation]:: Guest user at %f m. I'll wait a bit..",irobot_->getDetectedUser().distance);

		while((ros::Time::now() - init_detection < timeout) && irobot_->getDetectedUser().distance > WAIT_DISTANCE)
		{
			//Check user presence
			if(irobot_->robot_check_user(guest_name_))					//	Check for guest user
				setUserDetection(true);

			irobot_->robot_talk(get_speech_by_name("here"),true);
			nav_wait();

			ros::spinOnce();
			r_.sleep();
		}

	}
}

//=================================================================
//	Fire a Detection timeout
//=================================================================
void Navigation::user_detectTimer(const ros::TimerEvent& e)
{
	ROS_ERROR("[Navigation]:: Detect timeout. Check user presence.");

	//	First Stop every robot action
	irobot_->cancell_all_goal();

	// Remove current navigation goal
	path_planned_=false;
	user_recognized_=false;

	ros::Time init_detection = ros::Time::now();
	ros::Duration timeout(20.0);

	irobot_->base_rotate(const_cast<char *>("LEFT"));	// Rotate robot base because he should be on the robot side

	while((ros::Time::now() - init_detection < timeout) && !user_recognized_)
	{
		//Check user presence
		user_detect(guest_name_);

		ros::spinOnce();
		r_.sleep();
	}

	irobot_->base_stop();

	if(navigate_target && user_recognized_)
	{
		user_wait();
		irobot_->robot_talk(get_random_speech(string("nav_")),true);
		irobot_->robot_talk(get_speech_by_name("check_complete"),true);	// Just talk a little bit

		irobot_->cancell_all_goal();
		nav_wait();

		path_planned_=false;					//	Not usefull but to remember that	 now the controller had to recalculate new robot path to target
		user_recognized_=false;					//	User found no more interesting

		// User is following increase dalay before next check
		delay_detect +=DELAY_DETECT;
		detect_timeout_.stop();
		detect_timeout_ = nh_.createTimer(ros::Duration(DETECT_TIMEOUT + delay_detect), &Navigation::user_detectTimer,this,true,false);


	}
	else if(navigate_target)
	{
		// Recover user only if there's a navigation goal to target location.
		irobot_->robot_talk(get_speech_by_name("recover_start"),true);		// Just talk a little bit
		user_recover(guest_name_);										// User not found start Backtracking procedure

	}

	// Reset timers
	abort_timeout_.stop();
	detect_timeout_.stop();
}

//=====================================
//	Recover user using last detected position
//=====================================
void Navigation::user_recover(string user_name)
{
	ROS_ERROR("[Navigation]:: Start backtracking user.");

	irobot_->base_setGoal(last_user_detection_);						//	Go to last detection position of user

	user_recognized_=false;															// Just in case

	while(!nav_is_goal_reached() && !user_recognized_)		//	Check if reached last detection position or if the user has been found
	{
		user_detect(user_name);

		ros::spinOnce();
		r_.sleep();
	}

	irobot_->base_stop();

	if(user_recognized_)
	{
		user_wait();
		irobot_->robot_talk(get_speech_by_name("check_complete"),true);
		path_planned_=false;
	}
	if(!user_recognized_)
	{
		irobot_->robot_talk(get_speech_by_name("abort"),true);
		ROS_INFO("[Navigation]:: No user found during Backtracking.");
		ActionAbort();
	}

}

//=================================================================
// Set new user detection for a known user
//=================================================================
void Navigation::setUserDetection(bool status)
{
	if(status)
	{
		user_recognized_  = true;
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

//=================================================================
// Retrieve text for a conversation using string id
//=================================================================
string Navigation::get_random_speech(string what)
{
	int i=0;
	int topic_tot=0;

	for (i=0; i < speech_size_ ; ++i)
	{
		// different member versions of find in the same order as above:
		std::size_t found = speechs_[i].id.find(what);
		if (found!=std::string::npos)
			topic_tot++;
	}

	srand (time(NULL));
	int rand_= rand() % topic_tot;
	std::stringstream out;
	out << what << rand_;

	return get_speech_by_name(out.str());

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

	tf::StampedTransform transform;
	Pose pose;

	try{
		listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform);
		pose.position.x = transform.getOrigin().x();
		pose.position.y = transform.getOrigin().y();
		pose.position.z = 0.0;

		pose.orientation.z = transform.getRotation().getZ();
		pose.orientation.w = transform.getRotation().getW();
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}

	//ROS_DEBUG("[Odometry]:: Odometry pose x,y,z: [%f,%f]: ",pose.position.x,pose.position.y );
	//ROS_DEBUG("[Odometry]:: Odometry orient z,w: [%f,%f]: ", pose.orientation.z,pose.orientation.w);
	irobot_->setRobotPose(pose);
}
//=====================================
// FACE CALLBACK
//=====================================
void Navigation::face_callback(const user_tracker::ComConstPtr& msg)
{
	//ROS_INFO("[USER INFO]:: Face[x,y,z]: %f--%f--%f", msg->comPoints.x,msg->comPoints.y,msg->comPoints.z);

	const float angle_pixel_kinect = 0.07125;
	float x = msg->comPoints.x;
	float y = msg->comPoints.y;

	userdetected_.detected = true;
	userdetected_.angle = -(320 - x) * angle_pixel_kinect;
	userdetected_.distance = msg->comPoints.z/1000; // convert in m
}


//=====================================
// Kill everything and shutdown
//=====================================
bool Navigation::abort_service(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	ActionAbort();
	return true;
}

//=====================================
// Start new navigation task
//=====================================
bool Navigation::navigate_target_service(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	NavigateTarget();
	return true;
}

//=====================================
// Service to start autonomous navigation with known user
//=====================================
bool Navigation::find_user_service(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	LookingUser();
	return true;
}

//=====================================
// Service to start autonomous navigation with known user
//=====================================
bool Navigation::approach_user_service(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	ApproachUser();
	return true;
}

//=====================================
// Navigate to known location
//=====================================
bool Navigation::goto_service(e2_msgs::Goto::Request& request, e2_msgs::Goto::Response& response)
{
	
	if(marker_exist(request.location))
	{
		nav_goto(request.location);
		response.result = true;
		return true;
	}
	response.result = false;
	return false;

	return true;
}

//=====================================
// This service launch a detection face
//=====================================
bool Navigation::detect_service(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	user_recognized_ = false;

	ros::Time init_detection = ros::Time::now();
	ros::Duration timeout(30.0);

	while((ros::Time::now() - init_detection < timeout) && !user_recognized_)
		user_detect("unknown");

	if(user_recognized_)
	{
		nav_goto_detected_user(irobot_->getDetectedUser());

		user_recognized_ = false;
		irobot_->clearDetectedUser();
	}
	else
	{
		ROS_INFO("[Brain::Test] Detection Failed. No face detected in 30 sec.");
		return false;
	}


	return true;
}

//=====================================
// Train callback
//=====================================
bool Navigation::train_service(e2_msgs::Train::Request& request, e2_msgs::Train::Response& response)
{
	if(strcmp(request.username.c_str(), "") == 0)
	{
		ROS_INFO("[Navigation]:: Can't train without a username. Abort action.");
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
bool Navigation::neck_service(e2_msgs::NeckAction::Request& request, e2_msgs::NeckAction::Response& response)
{
	irobot_->neck_action(request.action,request.sub_action);
	return true;
}

//=====================================
// Service to test robot voice
//=====================================
bool Navigation::talk_service(e2_msgs::Talk::Request& request, e2_msgs::Talk::Response& response)
{
	if(strcmp(request.text.c_str(), "") == 0)
	{
		ROS_INFO("[Navigation]:: Empty string. Cant test voice. Abort");
		return false;
	}
	irobot_->robot_talk(get_speech_by_name(request.text),true);

	return true;
}

//=====================================
// Make kinect motor move
//=====================================
bool Navigation::kinect_service(e2_msgs::MotorAngle::Request& request, e2_msgs::MotorAngle::Response& response)
{
	irobot_->kinect_action(request.angle);

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


