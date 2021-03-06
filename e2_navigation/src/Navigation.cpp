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

	rotating = false;
	moving=false;

	find_user_ =  false;
	navigate_target = false;
	path_planned_  = false;
	path_to_user_   = false;
	user_recognized_ = false;
	action_aborted_ = false;
	action_completed_ = false;


	guest_user_info_.detected = false;
	guest_user_info_.angle = 0;
	guest_user_info_.distance = 0;
	guest_user_info_.user_left = true;
	guest_user_info_.user_right = false;
	guest_user_info_.user_lost = false;
	guest_user_info_.valid_pose = false;
	guest_user_info_.kinect_detect_time = ros::Time::now();

	initial_time_ = ros::Time::now();

    nh_.param<bool>("en_auto", find_user_, true);
	nh_.param<bool>("en_neck", en_neck, true);
	nh_.param<bool>("en_voice", en_voice, true);
	nh_.param<bool>("en_train", en_train, true);
	nh_.param<bool>("en_kinect", en_kinect, true);

	nh_.param("marker_config", marker_config, ros::package::getPath("e2_config")+"/map_config/sim_marker_config.yaml");
	nh_.param("speech_config", speech_config, ros::package::getPath("e2_config")+"/speak_config/speech_config.yaml");

	//	Suscribers
	face_sub_= nh_.subscribe("/com", 1,&Navigation::face_callback,this);
	odom_sub_= nh_.subscribe("/odom", 10,&Navigation::odometry_callback,this);
	cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10,&Navigation::velocity_callback,this);
	sonar_sub_ = nh_.subscribe("/e2_sonar", 1,&Navigation::sonar_callback,this);

	// Enable Services
	abort_service_ = nh_.advertiseService(name+"/abort",&Navigation::abort_service,this);
	goto_service_ = nh_.advertiseService(name+"/test_goto",&Navigation::goto_service,this);
	talk_service_ = nh_.advertiseService(name+"/test_voice",&Navigation::talk_service,this);
	neck_service_ = nh_.advertiseService(name+"/test_neck",&Navigation::neck_service,this);

	find_user_service_ = nh_.advertiseService(name+"/find_user",&Navigation::find_user_service,this);
	approach_user_service_= nh_.advertiseService(name+"/approach_user",&Navigation::approach_user_service,this);
	navigate_target_service_ = nh_.advertiseService(name+"/navigate_target",&Navigation::navigate_target_service,this);


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
	irobot_->robot_talk(get_speech_by_name("init"));
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

			// Clear Kinect
			irobot_->kinect_action(15);

			// Go Home....Bzzzz
			nav_wait();
			nav_goto(target_name_);
			path_planned_ = true;

			// Start
			init_detect_time = ros::Time::now();

		}
		else if(path_planned_)
		{

			//===============================================
			//	Check if need to detect user
			//================================================
			ros::Duration timeout(DETECT_TIMEOUT);

			if(guest_user_info_.user_lost)
			{
				if(ros::Time::now() - init_detect_time > timeout)
				{
					ROS_ERROR("[Navigation]:: Detection Timeout !");
					user_detectTimer();
				}
			}

			//===============================================
			//	Check Navigation Status
			//================================================
			string nav_status = irobot_->base_getStatus();

			if(strcmp(nav_status.c_str(),"ABORTED") == 0)
			{
				if(pass_count_> 1 )
				{
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
			else if(strcmp(nav_status.c_str(),"SUCCEEDED")==0 && path_planned_)
			{
				//Sono arrivato a destinazione. C'è ancora la persona ? Controllo
				ros::Time init_detection = ros::Time::now();
				ros::Duration timeout(20.0);
				user_recognized_ = false;

				while((ros::Time::now() - init_detection < timeout) && !user_recognized_)
				{
					if(guest_user_info_.user_left)
						irobot_->base_rotate(const_cast<char *>("LEFT"));	// Rotate robot base because he should be on the robot side
					else
						irobot_->base_rotate(const_cast<char *>("RIGHT"));	// Rotate robot base because he should be on the robot side

					//Check user presence
					user_detect(guest_name_);

					ros::spinOnce();
					r_.sleep();
				}

				// Stop Engines !! Jawol !!
				irobot_->base_stop();

				if(user_recognized_)
				{
					irobot_->robot_talk(get_speech_by_name("complete"),true);
					action_completed_ = true;
				}
				else
				{
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

		if(!path_planned_)
		{
			// Plan a Random Path to find people
			//nav_random_path();
			path_planned_ = true;
			path_to_user_ = false;
		}
		else if(guest_user_info_.detected && guest_user_info_.valid_pose)
		{
			if(!path_to_user_)
				irobot_->cancell_all_goal();

			ROS_ERROR("[Navigation]:: Ho trovato qualcuno !");

			if(guest_user_info_.distance >= FACE_ANALYSIS_DISTANCE)
			{
				ROS_INFO("[Navigation]:: Vado dallo zio ! (%f)",guest_user_info_.distance);

				float distance = guest_user_info_.distance - FACE_ANALYSIS_DISTANCE;

				nav_goto(distance,guest_user_info_.angle);
				path_to_user_ = true; // Set following user path
				nav_wait();
				nav_wait();
			}

		}

		string nav_status = irobot_->base_getStatus();
		if(strcmp(nav_status.c_str(),"ABORTED")==0)
		{
			if(path_to_user_)
			{
				ROS_ERROR("[Navigation]:: Non posso avvicinarmi alla persona (dist %f).",guest_user_info_.distance);
				path_to_user_ = false;
			}
			else
			{
				ROS_ERROR("[Navigation]:: Obstacle free nav. Restart.");
				path_planned_ = false;
			}
		}
		else if(strcmp(nav_status.c_str(),"SUCCEEDED")==0 )
		{
			if(guest_user_info_.detected && guest_user_info_.distance <= FACE_ANALYSIS_DISTANCE )
			{
				ROS_INFO("[Navigation]:: User in front of me !");

				irobot_->cancell_all_goal();

				irobot_->robot_talk(get_speech_by_name("user_found"),true);
				action_completed_ = true;

			}
			else
			{
				ROS_ERROR("[Navigation]:: Sono arrivato ma non ho trovato nessuno. Ricomincio!");
				path_planned_ = false;
				path_to_user_= false;
			}
		}

		// Clear user info
		user_clear();

	}
	/*==================================================================
		Navigate to approach a user detected TODO
	==================================================================*/
	else if(approach_user_)
	{
		if(guest_user_info_.detected)
		{
			if(!path_planned_)
			{
				// if distance is lower than 1 meter or is not too far from center camera the robot stay on place
				if(guest_user_info_.distance < FACE_ANALYSIS_DISTANCE)
				{
					ROS_INFO("[Navigation]:: User at distance %f and %f deg from camera. No action Done ! ",guest_user_info_.distance,guest_user_info_.angle);
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
			else if(strcmp(irobot_->base_getStatus().c_str(),"SUCCEEDED")==0 && guest_user_info_.valid_pose && guest_user_info_.detected && guest_user_info_.distance < FACE_ANALYSIS_DISTANCE)
			{
				action_completed_ = true;
			}
			else
				path_planned_= false;
		}
	}

	// Very usefull for sensors detection
	moving = false;

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
// Reset current navigation status
//=================================================================
void Navigation::ActionReset()
{
	ROS_ERROR("[Navigation]:: Reset Navigation");

	irobot_->cancell_all_goal();

	pass_count_= 0;
	delay_detect = 0;
	rotating = false;

	navigate_target = false;
	find_user_= false;
	approach_user_ = false;

	action_completed_ = false;
	action_aborted_ = false;

	path_planned_  = false;
	path_to_user_   = false;
	user_recognized_ = false;

	guest_user_info_.detected = false;
	guest_user_info_.angle = 0;
	guest_user_info_.distance = 0;
	guest_user_info_.valid_pose = false;
	guest_user_info_.user_lost = false;

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

	if(irobot_->train_enabled)
	{
		irobot_->robot_talk(get_speech_by_name("train_init"));

		// Save new user face
		if(irobot_->robot_train_user(guest_name_))
		{
			irobot_->robot_talk(get_speech_by_name("train_success"),true);
		}
		else
		{
			irobot_->robot_talk(get_speech_by_name("train_failed"),true);
			ActionReset();
			return;
		}
	}

	navigate_target = true;

	// Save current position as first user detection position
	initial_time_ = ros::Time::now();
	setUserDetection(true);

	irobot_->robot_talk(get_speech_by_name("follow_me"),true);
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
//	Clear data of detected user
//=================================================================
void Navigation::user_clear()
{
	guest_user_info_.detected = false;
	guest_user_info_.angle = 0;
	guest_user_info_.distance = 0;
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
void Navigation::user_detectTimer()
{
	ROS_INFO("[Navigation]:: Checking user presence.");

	//	First Stop every robot action
	irobot_->cancell_all_goal();

	// Remove current navigation goal
	path_planned_=false;
	user_recognized_=false;

	ros::Time init_detection = ros::Time::now();
	ros::Duration timeout(30.0);

	while((ros::Time::now() - init_detection < timeout) && !user_recognized_)
	{
		if(!rotating)
			if(guest_user_info_.user_left)
				irobot_->base_rotate(const_cast<char *>("LEFT"));	// Rotate robot base because he should be on the robot side
			else
				irobot_->base_rotate(const_cast<char *>("RIGHT"));	// Rotate robot base because he should be on the robot side

		//Check user presence
		user_detect(guest_name_);

		ros::spinOnce();
		r_.sleep();
	}

	irobot_->base_stop();

	if(navigate_target && user_recognized_)
	{
		user_wait();

		irobot_->robot_talk(get_speech_by_name("check_complete"),true);	// Just talk a little bit

		irobot_->cancell_all_goal();

		nav_wait();

		path_planned_=false;					//	Not usefull but to remember that	 now the controller had to recalculate new robot path to target
		user_recognized_=false;					//	User found no more interesting

	}
	else if(navigate_target)
	{
		// Recover user only if there's a navigation goal to target location.
		irobot_->robot_talk(get_speech_by_name("recover_start"),true);		// Just talk a little bit
		user_recover(guest_name_);											// User not found start Backtracking procedure
	}

}

//=====================================
//	Recover user using last detected position
//=====================================
void Navigation::user_recover(string user_name)
{
	ROS_ERROR("[Navigation]:: Start backtracking user.");

	irobot_->base_setGoal(last_user_detection_);						//	Go to last detection position of user

	user_recognized_=false;												// Just in case

	while(!nav_is_goal_reached() && !user_recognized_)					//	Check if reached last detection position or if the user has been found
	{
		user_detect(user_name);

		ros::spinOnce();
		r_.sleep();
	}

	irobot_->base_stop();

	if(user_recognized_)
	{
		user_wait();
		irobot_->robot_talk(get_random_speech(string("nav_")),true);
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
Speech Navigation::get_speech_by_name(string name)
{
	int i=0;
	for (i=0; i < speech_size_ ; ++i)
	{
		if(strcmp(speechs_[i].id.c_str(),name.c_str()) == 0)
			break;
	}

	return speechs_[i];
}

//=================================================================
// Retrieve text for a conversation using string id
//=================================================================
Speech Navigation::get_random_speech(string what)
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
// Service definitions && Callbacks
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
// FACE CALLBACK - TODO
//=====================================
void Navigation::face_callback(const user_tracker::ComConstPtr& msg)
{
	guest_user_info_.kinect_detect_time = ros::Time::now();

	// Get user position
	tf::StampedTransform transform;
	std::stringstream out_frame;

	out_frame << "user_head_" << (int)msg->id;

	if(find_user_ || approach_user_)
	{
		try{

			listener_.lookupTransform("/openni_depth_frame",out_frame.str(), ros::Time(0), transform);

			guest_user_info_.pose.position.x = transform.getOrigin().x();
			guest_user_info_.pose.position.y = transform.getOrigin().y();
			guest_user_info_.pose.position.z = 0.0;

			guest_user_info_.pose.orientation.z = transform.getRotation().getZ();
			guest_user_info_.pose.orientation.w = transform.getRotation().getW();

			double x = transform.getOrigin().x();
			double y = transform.getOrigin().y();
			double dist = sqrt(x*x + y*y);

			const float angle_pixel_kinect = 0.07125;

			guest_user_info_.detected = true;
			guest_user_info_.valid_pose = true;

			guest_user_info_.distance = dist;
			//guest_user_info_.angle = -(320 - msg->comPoints.x) * angle_pixel_kinect;
			double angle = asin(-y/dist);
			guest_user_info_.angle = 180 * angle / M_PI; 
			ROS_ERROR("Agle %f",guest_user_info_.angle);
		}
		catch (tf::TransformException ex){
			//ROS_ERROR("%s",ex.what());
			//Trasform not availabe, consider as not detected
			guest_user_info_.detected = false;
			guest_user_info_.valid_pose = false;

			guest_user_info_.distance = 0.0;
			guest_user_info_.angle = 0.0;
		}

	}

}
//=====================================
// Velocity CALLBACK
//=====================================
void Navigation::velocity_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
	moving=true;

	if(msg->linear.x  == 0 && msg->angular.z > 0)
	{
		rotating=true;
		init_detect_time = ros::Time::now(); // reset detect time.
		ROS_DEBUG("[Navigation]:: Robot is rotating....................in place");
	}
	else
		rotating=false;

}

//=====================================
// Sonar CALLBACK - TODO
//=====================================
void Navigation::sonar_callback(const e2_sonar::Sonar::ConstPtr& msg)
{
	if(path_planned_ && !rotating && navigate_target)
	{
		// Sonar Sinistra: 5-4-6
		// Sonar Destra: 1-2-0

		// Frontal Data
		if(msg->sonar3 > 0 && msg->sonar3 < 20 )
		{
			ROS_ERROR("[Navigation::Sonar]:: Frontal Obstacle !!!!!");
			// DO SOMETHING
		}

		// Left Data
		if(guest_user_info_.user_left )
		{
			if((msg->sonar6 > 0 || msg->sonar5 > 0 || msg->sonar4 > 0) && ((msg->sonar6 < USER_SONAR_DISTANCE || msg->sonar5 < USER_SONAR_DISTANCE || msg->sonar4 < USER_SONAR_DISTANCE)) )
			{		
				float diff_ = abs(msg->sonar6 - msg->sonar4 );
			 	
				if(diff_ < 5)
				{
					ROS_INFO("[Navigation::Sonar]:: Fake data!!!! Maybe a Wall.");
					guest_user_info_.user_lost = true;
					init_detect_time = ros::Time::now();
				}
				else
				{
					ROS_INFO("[Navigation::Sonar]:: Utente a sinistra");

					guest_user_info_.user_lost = false;
					init_detect_time = ros::Time::now();
				}

			}
			else 
			{
				ROS_ERROR("[Navigation::Sonar]:: User Lost by LEFT sonar");
				guest_user_info_.user_lost = true;
			}
		}
		else if(guest_user_info_.user_right)
		{
			// Right Data
			if((msg->sonar0 > 0 || msg->sonar1 > 0 || msg->sonar2 > 0) && (msg->sonar0 < USER_SONAR_DISTANCE || msg->sonar1 < USER_SONAR_DISTANCE || msg->sonar2 < USER_SONAR_DISTANCE))
			{
				float diff_ = abs(msg->sonar2 - msg->sonar0 );

				if(diff_ < 5)
				{
					ROS_INFO("[Navigation::Sonar]:: Fake data!!!! Maybe a Wall.");
					guest_user_info_.user_lost = true;
					init_detect_time = ros::Time::now();
				}
				else
				{
					ROS_INFO("[Navigation::Sonar]:: Utente a destra");

					guest_user_info_.user_lost = false;
					init_detect_time = ros::Time::now();
				}
			}
			else 
			{
				ROS_ERROR("[Navigation::Sonar]:: User Lost by RIGHT sonar");
				guest_user_info_.user_lost = true;
			}
		}
	}

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
	node["neck_action"] >> speech.neck_action;
	node["face_action"] >> speech.face_action;
}


