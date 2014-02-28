/*************************************************************
 * This node define the robot behaviours during a navigation
 * task.
 *
 * by Lorenzo Ripani - AIRLab (Politecnico di Milano)
 * email: ripani.lorenzo@gmail.com
 *
 * version 0.1 - 01/2014
 *************************************************************/

// Default ros include
#include "ros/ros.h"
#include "ros/package.h"

// Yaml declaration
#include "yaml.h"
#include <yaml-cpp/yaml.h>
#include <e2_navigation/e2_struct.h>

#include <fstream>

// Standard Messages
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

// Actionlib
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Face recognition
#include <face_recognition/FaceRecognitionAction.h>
#include <face_recognition/FaceRecognitionActionResult.h>
#include <face_recognition/FaceRecognitionFeedback.h>

// Navigation messages
#include <e2_navigation/RobotAction.h>

// Simulation Include for sensors
#include "vrep_common/v_repConst.h"
#include "vrep_common/ProximitySensorData.h"
#include "vrep_common/VrepInfo.h"


//--------------------------------------------------------------------------------------------
//		Definitions
//--------------------------------------------------------------------------------------------
#define E2_ANGULAR_VELOCITY 	0.5																// Max angular velocity
#define MIN_USER_DISTANCE 		0.5 																// Min distance for user detection (sonar)
#define MAX_USER_DISTANCE 		1.5 																// Max distance for user detection (sonar)

#define DETECT_TIMEOUT	 			60																// Define the time before fire a detection request
#define ABORT_TIMEOUT 				300																// Navigation timeout

#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "e2_navigator"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<face_recognition::FaceRecognitionAction> FRClient;


//==========================================================================
// Navigation Class
//==========================================================================
class Navigation
{
	private:

		ros::Time start_task;
		ros::Time last_object_detection;

		ros::NodeHandle Handle;										// Node hadler
		geometry_msgs::PoseStamped robot_pose;		// the current robot position. Taken from odom topic

		std::string stand_map;											// Config file with stand positions

		ros::Publisher Publisher_base_control;				// Publisher for base messages
	    ros::Publisher Publisher_neck_control; 				// Neck joint topic publisher

	    // Suscribers for input messages
	    ros::Subscriber Subscriber_odom;						// Odometry
	    ros::Subscriber Subscriber_sonar;						// Sonar
	    ros::Subscriber Subscriber_robotaction;				// Used only when brian is active


	    bool init;																	// used to get initial robot position
	    bool active_task;
	    bool path_planned;
	    bool user_recognized;
	    bool object_detected;

	    bool user_interested;
	    bool user_face_saved;

	    std::string user_name;

	    MoveBaseClient *ac;												// Move base pointer
	    FRClient * ac_fr; 														// Face recognition

	    ros::Rate *LoopRate;

	    ros::Timer abort_timeout;
	    ros::Timer detect_timeout;
	    ros::Timer recover_timeout;

	    Marker *stand_positions;
	    move_base_msgs::MoveBaseGoal lastuser_detection;

	    void OdometryCb(const nav_msgs::Odometry::ConstPtr& msg);															// Odometry messages callback
	    void RobotActionCb(const e2_navigation::RobotAction::ConstPtr& msg);											// Brian messages callback
	    void SensorCb(const vrep_common::ProximitySensorData::ConstPtr& sens);										// Sonar message callback

	    void FaceRecognCb(const actionlib::SimpleClientGoalState& state, const face_recognition::FaceRecognitionResultConstPtr& result);

	public:

		bool autonomous;																					// If autonomous the robot use RobotController for navigation task else use messages from robot brain

		void Prepare();																						// Node initialization

		/* Robot Actions*/
		void RobotController(void);																// Autonomous controller
		void RobotRotate(char * direction);													// Rotate Robot
		void RobotStop(void);
		void DetectUser(const ros::TimerEvent& e);									// Check for a user in faces database
		void RecoverUser(void);																		// Recover User following the path of last position detection
		void AbortTask(const ros::TimerEvent& e);

		bool getNavigationStatus(void);														// Check if robot is in final position

		/* Face Recognition */
	    void TrainUserFace(void);																	// Used to train a new face in the database
	    void CheckFace(void);

		/* Yaml utility */
	    void getStandPosition(YAML::Node& doc);
	    move_base_msgs::MoveBaseGoal getStand(std::string name);

};

/*
 * Node initialization.
 */
void Navigation::Prepare()
{
	init = true;
	autonomous=true;
	active_task=false;
	path_planned=false;

	user_recognized=false;
	object_detected=false;

	user_interested = true;
	user_face_saved= true;				// true not to train in simulation

	ros::NodeHandle HandleP("~");
	HandleP.param<std::string>("stand_map", stand_map, ros::package::getPath("e2_navigation")+"/config/stand_map.yaml");

	std::ifstream fin(stand_map.c_str());

	YAML::Node doc;
	YAML::Parser parser(fin);
	parser.GetNextDocument(doc);

	getStandPosition(doc);

	ROS_DEBUG("MARK 1: %s",stand_positions[0].name.c_str());

	user_name="Lorenzo";
	//user_name="Guest";

	if(!autonomous)
		Subscriber_robotaction = Handle.subscribe("/e2/action", 1, &Navigation::RobotActionCb, this);

	Subscriber_odom = Handle.subscribe("/odom", 10, &Navigation::OdometryCb, this);
	Subscriber_sonar = Handle.subscribe("/e2/sonar", 10, &Navigation::SensorCb, this);

	Publisher_base_control = Handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	Publisher_neck_control = Handle.advertise<geometry_msgs::Twist>("/e2/neck", 1);

	LoopRate = new ros::Rate(1.0/RUN_PERIOD_DEFAULT);

	ac = new MoveBaseClient("move_base", true);
	ac_fr = new FRClient("face_recognition", true);

	while (!ac->waitForServer(ros::Duration(5.0))) {
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	while (!ac_fr->waitForServer(ros::Duration(5.0))) {
		ROS_INFO("Waiting for the face_recognition action server to come up");
	}

	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());

}

/*
 * Get odometry message and update robot position.
 */
void Navigation::OdometryCb(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_DEBUG("Robot Position x,y,z: [%f,%f,%f]: ", msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
	robot_pose.pose = msg->pose.pose;

	if(init)
	{
		ROS_INFO(" * Get initial robot position");
		lastuser_detection.target_pose.header.frame_id = "map";
		lastuser_detection.target_pose.header.stamp = ros::Time::now();
		lastuser_detection.target_pose.pose.position.x = robot_pose.pose.position.x;
		lastuser_detection.target_pose.pose.position.y = robot_pose.pose.position.y;
		lastuser_detection.target_pose.pose.orientation.z = robot_pose.pose.orientation.z;
		lastuser_detection.target_pose.pose.orientation.w = robot_pose.pose.orientation.w;

		init = false;

	}
}

/*
 * Get messages from sonar topic
 * TODO remove simulation type for real one
 */
void Navigation::SensorCb(const vrep_common::ProximitySensorData::ConstPtr& sens)
{

	double obj_distance = sens->detectedPoint.z;

	if (obj_distance >= MIN_USER_DISTANCE && obj_distance <= MAX_USER_DISTANCE) {

		ROS_DEBUG("DETECTED object  distance: %f", sens->detectedPoint.z);

		object_detected = true;
		last_object_detection	=  ros::Time::now();
	}

}

//==========================================================================
// Robot Actions
//==========================================================================

/*
 * Define the actions the robot had to take if controlled by e2_brain
 */
void Navigation::RobotActionCb(const e2_navigation::RobotAction::ConstPtr& msg)
{
	ROS_INFO(" * Received an action");

	int action = msg->order_id;
	move_base_msgs::MoveBaseGoal home;

	switch (action) {

		case 0: // New interested user. Train face
			ROS_INFO(" *Brain* New interested user. Training face ...");
			TrainUserFace();
			break;
		case 1:	// Go Base
			ROS_INFO(" *Brain* I'm going back home");
			ac->sendGoal(getStand("home"));
			break;
		case 2:	// Go stand
			ROS_INFO(" *Brain* I'm going to the stand");
			ac->sendGoal(getStand(msg->name.c_str()));
			break;
		case 3:	// Check User
			ROS_INFO(" *Brain* I'm checking user presence....");
			CheckFace();
			break;
		case 4:	// Backtrack User
			ROS_INFO(" *Brain* I'm backtracking user");
			RecoverUser();
			break;
		case 5:	// Wait
			ROS_INFO(" *Brain* I'm waiting");
			break;
		default: // Stop Robot
			ROS_INFO(" *Brain* Stop Robot...something strange appened");
			RobotStop();
			break;
	}
}

/*
 * Define the action the robot had to take if working in autonomous mode
 */
void Navigation::RobotController(void)
{
	ROS_INFO(" * New Task Started");

	active_task = true;
	start_task = ros::Time::now();

	abort_timeout = Handle.createTimer(ros::Duration(ABORT_TIMEOUT), &Navigation::AbortTask,this,true,false);
	detect_timeout = Handle.createTimer(ros::Duration(DETECT_TIMEOUT), &Navigation::DetectUser,this,false,false);

	abort_timeout.start();

	while(ros::ok() && active_task)
	{

		if(user_interested)
		{	/* Interested user. Bring him home */

			if(!user_face_saved)
			{	/* Save user face */
				TrainUserFace();
			}
			else if(!path_planned)
			{
				ROS_INFO(" * Going AirLab stand");
				ac->sendGoal(getStand("airlab"));

				detect_timeout.start();
				path_planned = true;

			}
			else if(path_planned)
			{
				getNavigationStatus();
			}

		}
		else
		{	/* Looking for user */

		}

		ros::spinOnce();
		LoopRate->sleep();
	}

	abort_timeout.stop();
	detect_timeout.stop();

	ac->sendGoal(getStand("home"));

}


/**
 * Send messages to make the robot rotate on place.
 * Allowed direction LEFT RIGHT
 */
void Navigation::RobotRotate(char * direction)
{
	ROS_DEBUG(" *** Rotating...");
	geometry_msgs::Twist rotation;

	rotation.linear.x = 0;
	rotation.angular.z = E2_ANGULAR_VELOCITY;

	if(strcmp(direction,"RIGHT") == 0)
	{
		ROS_DEBUG(" * * * * ROTATE RIGHT * * * *");
		rotation.angular.z  *= -1;
	}

	Publisher_base_control.publish(rotation);

}

/*
 * Stop the robot wheels
 */
void Navigation::RobotStop(void)
{
	ROS_INFO(" *** Robot stopped...");

	geometry_msgs::Twist stop;

	stop.linear.x = 0;
	stop.angular.z = 0;

	Publisher_base_control.publish(stop);
}

/*
 * Check if the robot has reached final position.
 */
bool Navigation::getNavigationStatus(void)
{
	// Check if the robot succeded it's task
	if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO(" *** Hooray, we reached the Goal ! ");
		active_task = false;
		return true;
	}
	else
	{
		active_task = true;
		return false;
	}
}

/*
 * Check if the robot is stuck in a task. If the task exceeded timeout, it kill the current task.
 */
void Navigation::AbortTask(const ros::TimerEvent& e)
{
	ROS_INFO(" * Timeout.... Task killed");
	active_task = false;

	ac_fr->cancelAllGoals();

	ac->cancelAllGoals();
	ac->sendGoal(getStand("home"));
}

/*
  * If this function is used a new face is added to the database
  */
void Navigation::TrainUserFace(void)
{
	face_recognition::FaceRecognitionGoal goal; //Goal message

	ac_fr->waitForServer();

    goal.order_id = 5;
    goal.order_argument = user_name;

    ac_fr->sendGoal(goal);
    ac_fr->waitForResult(ros::Duration(5.0));

    goal.order_id = 2;
    goal.order_argument = user_name;

    ac_fr->sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac_fr->waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
    	ROS_INFO(" * Face saved.");

       goal.order_id = 3;
       goal.order_argument = user_name;

       ac_fr->sendGoal(goal);

	   user_face_saved = true;

	   ROS_INFO(" * Database updated");
    }
    else
    {
    	ROS_INFO(" * Problem saving new face...timeout occurred");
    	active_task=false;
    }


}

/*
 * Checks if there's a face recognition
 */
void Navigation::DetectUser(const ros::TimerEvent& e)
{
	ROS_INFO(" * * Default timeout. Check user presence....");

	ac->cancelAllGoals();

	ros::Time init_time = ros::Time::now();
	ros::Duration timeout(30.0);

	while((ros::Time::now() - init_time < timeout) && !user_recognized)
	{
		RobotRotate("LEFT");
		CheckFace();
		LoopRate->sleep();
	}

	if(user_recognized)
	{
		path_planned=false;
		user_recognized=false;
	}
	else
	{
		RecoverUser();			// User not found start Backtracking procedure
	}

}

/*
 * Use previous detected position to recover the user.
 */
void Navigation::RecoverUser(void)
{
	ROS_INFO(" * * Start backtracking user... ");

	RobotStop();

	ac->waitForServer();
	ac->sendGoal(lastuser_detection);

	while(!getNavigationStatus() && !user_recognized)
	{
		CheckFace();
		LoopRate->sleep();
	}

	if(user_recognized)
	{
		active_task=true;
		path_planned=false;
		user_recognized=false;
	}
	else
	{
		ROS_INFO(" * * * Yoh dude i'm in the last position...but no one found.");
		active_task=false; // Killing current task because no user was found
	}

}
/*
 * Query Face detection node to see if there's a known face in the database
 */
void Navigation::CheckFace()
{
	ROS_INFO(" * * * Checking for known faces....");

	face_recognition::FaceRecognitionGoal goal;
    goal.order_id = 0;
    goal.order_argument = user_name;

    ac_fr->sendGoal(goal, boost::bind(&Navigation::FaceRecognCb, this, _1, _2),FRClient::SimpleActiveCallback(), FRClient::SimpleFeedbackCallback());
    ac_fr->waitForResult(ros::Duration(2.0));

}

/*
 * Callback for facedetection
 */
void Navigation::FaceRecognCb(const actionlib::SimpleClientGoalState& state,
            const face_recognition::FaceRecognitionResultConstPtr& result)
{
  ROS_INFO(" * * * * Goal [%i] Finished in state [%s]", result->order_id,state.toString().c_str());

  if(state.toString() != "SUCCEEDED") return;

  if( result->order_id==0)
  {
	  ROS_INFO(" * * * * %s was recognized with confidence %f", result->names[0].c_str(),result->confidence[0]);
	  if(strcmp(result->names[0].c_str(),user_name.c_str())==0)
	  {

			ROS_INFO(" * * * * Detected User: %s",result->names[0].c_str());

			user_recognized=true;

			lastuser_detection.target_pose.header.frame_id = "map";
			lastuser_detection.target_pose.header.stamp = ros::Time::now();
			lastuser_detection.target_pose.pose.position.x = robot_pose.pose.position.x;
			lastuser_detection.target_pose.pose.position.y = robot_pose.pose.position.y;
			lastuser_detection.target_pose.pose.orientation.z = robot_pose.pose.orientation.z;
			lastuser_detection.target_pose.pose.orientation.w = robot_pose.pose.orientation.w;

		}
		else
			user_recognized=false;
  }

  if( result->order_id==2)
    ROS_INFO(" * * * * Pictures of %s were successfully added to the training images",result->names[0].c_str());

}



/*
 * Load the position of the stands from config file
 */
void Navigation::getStandPosition(YAML::Node& doc)
{
	stand_positions = new Marker [doc.size()];

	// Load Markers from map file
	for(unsigned i=0;i<doc.size();i++)
		doc[i] >> stand_positions[i];
}

/*
  * Get the stand position by name and return a goal message
  */
move_base_msgs::MoveBaseGoal Navigation::getStand(std::string name)
{
	move_base_msgs::MoveBaseGoal goal;
	int i=0;

	for (i=0; i < sizeof(stand_positions); ++i) {

		if(strcmp(stand_positions[i].name.c_str(),name.c_str()) == 0)
			break;
	}

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = stand_positions[i].position.x;
	goal.target_pose.pose.position.y = stand_positions[i].position.y;

	goal.target_pose.pose.orientation.z = 0.7;
	goal.target_pose.pose.orientation.w = 0.7;

	return goal;
}

//-----------------------------------------------------------------
// End node functions
//-----------------------------------------------------------------


int main(int argc, char **argv)
{
	// Inizialize the node
	ros::init(argc, argv, NAME_OF_THIS_NODE);

	Navigation node;
	node.Prepare();

	if(node.autonomous)
		node.RobotController();

	return 0;

}
