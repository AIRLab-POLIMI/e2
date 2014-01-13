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

#include <fstream>

// Standard Messages
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

// Face recognition
#include <face_recognition/FRClientGoal.h>
#include <face_recognition/FaceRecognitionActionResult.h>

// Actionlib
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Navigation messages
#include <e2_navigation/RobotAction.h>

// Simulation Include for sensors
#include "vrep_common/v_repConst.h"
#include "vrep_common/ProximitySensorData.h"
#include "vrep_common/VrepInfo.h"


//--------------------------------------------------------------------------------------------
//		Definitions
//--------------------------------------------------------------------------------------------
#define MIN_USER_DISTANCE 0.5 // Min distance for user detection
#define MAX_USER_DISTANCE 1.5 // Max distance for user detection
#define E2_ANGULAR_VELOCITY 0.5

#define TIME_CHECK_OBJECT 20 // Define time interval (in seconds) before check user if no object is detected by sonar
#define TIME_CHECK_USER 60 // Define time interval (in seconds) before fire a find user

#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "e2_navigator"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


//--------------------------------------------------------------------------------------------
//		Structure for yaml file reading (Stand map)
//--------------------------------------------------------------------------------------------
struct Vec3
{
   float x, y, z;
};

struct Marker
{
   int id;
   std::string name;
   Vec3 position;
};

void operator >> (const YAML::Node& node, Vec3& v) {
   node[0] >> v.x;
   node[1] >> v.y;
   node[2] >> v.z;
}

void operator >> (const YAML::Node& node, Marker& marker) {
   node["id"] >> marker.id;
   node["name"] >> marker.name;
   node["position"] >> marker.position;
}

//==========================================================================
// Navigation Class
//==========================================================================
class Navigation
{
	private:

		std::string stand_map;												// Config file
		ros::Time last_object_detection;
		ros::NodeHandle Handle;										// Node hadler
		geometry_msgs::PoseStamped robot_pose;

	    ros::Publisher Publisher_fr_order; 						// Face recognition order
	    ros::Publisher Publisher_base_control;				// Publisher for base messages
	    ros::Publisher Publisher_neck_control; 				// Neck joint topic

	    // Suscribers for input messages
	    ros::Subscriber Subscriber_odom;						// Odometry
	    ros::Subscriber Subscriber_sensor0;					// Sonar
	    ros::Subscriber Subscriber_sensor1;					// Sonar
	    ros::Subscriber Subscriber_sensor2;					// Sonar
	    ros::Subscriber Subscriber_robotaction;			// Used only by brian
	    ros::Subscriber Subscriber_fr_status; 					// Face recognition status

	    bool backtracking;
	    bool path_planned;
	    bool user_recognized;
	    bool object_detected;


	    std::string user_name;
	    std::string navigation_status;
	    std_msgs::String msg;

	    void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);															// Odometry messages
	    void RobotActionCallback(const e2_navigation::RobotAction::ConstPtr& msg);											// Brian messages
	    void SensorCallback(const vrep_common::ProximitySensorData::ConstPtr& sens);										// Sonar messages
	    void FaceRecognCallback(const face_recognition::FaceRecognitionActionResult::ConstPtr& msg);		// Face recognition messages


	public:

		int stand_size;
		double RunPeriod;

		bool autonomous;										// If autonomous the robot use RobotController for navigation task else use messages from robot brain
		bool task_completed;								// true if the robot has completed the task requested
		Marker *stand_positions;

		move_base_msgs::MoveBaseGoal goal,last_user_detection;

		void Prepare(MoveBaseClient* temp);															// Node initialization

		void setGoalPosition(move_base_msgs::MoveBaseGoal goal);			// Define goal position
		void getGoalStatus(void);																					// Check if robot is in final position

		void RobotController(void);																				// Decide what action the robot had to choose if in autonomous mode
	    void RobotRotate(void);
	    void RobotStop(void);

	    void DetectUser(bool rotate = true);															// Check for a user in database faces
	    void RecoverUser(void);																						// Recover User following the path of last position detection

	    void TrainUserFace(void);																					// Used to train a new face in the database

	    void getStandPosition(YAML::Node& doc);
	    move_base_msgs::MoveBaseGoal getStand(std::string name);			// Get a movebasegoal from a stand name

	    MoveBaseClient *ac;																							// Action client pointer

};

/*
 * Node initialization.
 */
void Navigation::Prepare(MoveBaseClient* actionclient)
{
	autonomous=true;
	backtracking=false;
	path_planned=false;
	task_completed=false;
	user_recognized=false;
	object_detected=false;

	ros::NodeHandle HandleP("~");
	std::ifstream fin(stand_map.c_str());
	RunPeriod = RUN_PERIOD_DEFAULT;

	HandleP.param<std::string>("stand_map", stand_map, ros::package::getPath("e2_navigation")+"/config/stand_map.yaml");

	YAML::Node doc;
	YAML::Parser parser(fin);
	parser.GetNextDocument(doc);

	getStandPosition(doc);

	ROS_DEBUG("MARK 1: %s",stand_positions[0].name.c_str());

	user_name="Lorenzo";
	//user_name="Guest";

	Subscriber_odom = Handle.subscribe("/odom", 10, &Navigation::OdometryCallback, this);
	Subscriber_sensor0 = Handle.subscribe("/e2/sensor_0", 10, &Navigation::SensorCallback, this);
	Subscriber_sensor1= Handle.subscribe("/e2/sensor_1", 10, &Navigation::SensorCallback, this);
	Subscriber_sensor2 = Handle.subscribe("/e2/sensor_2", 10, &Navigation::SensorCallback, this);
	Subscriber_fr_status = Handle.subscribe("/face_recognition/result", 1, &Navigation::FaceRecognCallback, this);

	if(!autonomous)
		Subscriber_robotaction = Handle.subscribe("/e2/action", 1, &Navigation::RobotActionCallback, this);

	Publisher_fr_order = Handle.advertise<face_recognition::FRClientGoal>("/fr_order", 1);
	Publisher_base_control = Handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	Publisher_neck_control = Handle.advertise<geometry_msgs::Twist>("/e2/neck", 1);

	last_user_detection.target_pose.header.stamp = ros::Time::now() - ros::Duration(100); 		// So we force the first detection in autonomous mode

	ac = actionclient;

	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());


}


/*
 * This function set the goal position for the robot.
 */
void Navigation::setGoalPosition(move_base_msgs::MoveBaseGoal goal)
{

	  //wait for the action server to come up
	  while (!ac->waitForServer(ros::Duration(5.0))) {
		  ROS_INFO("Waiting for the move_base action server to come up");
	  }

	  //Set some position to test robot navigation
	  ac->sendGoal(goal);
}



/*
 * Check if the robot has reached final position.
 */
void Navigation::getGoalStatus(void)
{
	// Check if the robot succeded it's task
	if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO(" *** Hooray, we reached the Goal ! ");
		task_completed = true;
	}
	else
	{
		ROS_DEBUG(" *** Still not finished.");
		task_completed = false;
	}
}



/*
 * Get odometry message and update robot position.
 */
void Navigation::OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_DEBUG("Robot Position x,y,z: [%f,%f,%f]: ", msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
	robot_pose.pose = msg->pose.pose;
}

/*
 * This function suscribe to sensors topic and check for detection.
 */
void Navigation::SensorCallback(const vrep_common::ProximitySensorData::ConstPtr& sens)
{

	double obj_distance = sens->detectedPoint.z;

	if (obj_distance >= MIN_USER_DISTANCE && obj_distance <= MAX_USER_DISTANCE) {

		ROS_DEBUG("Received a detection! ");
		ROS_DEBUG("Distance from object %f", sens->detectedPoint.z);

		object_detected = true;
		last_object_detection	=  ros::Time::now();
	}

}

/*
 * This function  suscribed to face_recognition node and check for successfull detections
 */
void Navigation::FaceRecognCallback(const face_recognition::FaceRecognitionActionResult::ConstPtr& msg)
{

	if(msg->status.status == 3 && strcmp(msg->result.names[0].c_str(),user_name.c_str())==0)
	{

		ROS_INFO(" *** Detected User: %s",msg->result.names[0].c_str());

		backtracking = false;
		user_recognized=true;

		last_user_detection.target_pose.header.frame_id = "map";
		last_user_detection.target_pose.header.stamp = ros::Time::now();
		last_user_detection.target_pose.pose.position.x = robot_pose.pose.position.x;
		last_user_detection.target_pose.pose.position.y = robot_pose.pose.position.y;
		last_user_detection.target_pose.pose.orientation.z = robot_pose.pose.orientation.z;
		last_user_detection.target_pose.pose.orientation.w = robot_pose.pose.orientation.w;

	}
	else
		user_recognized=false;

}


/*
 * Define the actions the robot had to take if controlled by e2_brain
 */
void Navigation::RobotActionCallback(const e2_navigation::RobotAction::ConstPtr& msg)
{
	ROS_INFO(" * Received an action");

	int action = msg->order_id;
	move_base_msgs::MoveBaseGoal home;

	switch (action) {

		case 0: // New interested user. Train face
			ROS_INFO(" ** New interested user. Training face ...");
			TrainUserFace();
			break;
		case 1:	// Go Base
			ROS_INFO(" ** I'm going back home");
			setGoalPosition(getStand("home"));
			break;
		case 2:	// Go stand
			ROS_INFO(" ** I'm going to the stand");
			setGoalPosition(getStand(msg->name.c_str()));
			break;
		case 3:	// Check User
			ROS_INFO(" ** I'm checking user presence....");
			DetectUser(true);
			break;
		case 4:	// Backtrack User
			ROS_INFO(" ** I'm backtracking user");
			//RecoverUser();
			break;
		case 5:	// Wait
			ROS_INFO(" ** I'm waiting");
			break;
		default: // Stop Robot
			ROS_INFO(" ** Stop Robot...something strange appened");
			RobotStop();
			break;
	}
}

/*
 * Define the action the robot had to take if working in autonomous mode
 */
void Navigation::RobotController(void)
{

	ros::Time time_userdetection = last_user_detection.target_pose.header.stamp + ros::Duration(TIME_CHECK_USER);

	if((ros::Time::now() > time_userdetection) && path_planned) // Default user checking
	{

		ROS_INFO(" * Detecting User.");
		DetectUser(true);

		path_planned = false;

	}
	else if(user_recognized && !path_planned)
	{
		ROS_INFO(" * Going home");
		setGoalPosition(getStand("home"));

		path_planned = true;
		user_recognized = false;

	}
	else if (ros::Time::now() > ( last_object_detection + ros::Duration(TIME_CHECK_OBJECT)))
	{
		ROS_INFO(" *** No object detected in the field...Force user check.");
		path_planned = false;

		DetectUser(true);
	}

	object_detected=false;

	if(path_planned)
		getGoalStatus();
}

/*
 * Tell the robot to start rotating on place. Used to check if there's a user near the robot
 */
void Navigation::RobotRotate(void)
{
	ROS_DEBUG(" *** Rotating ROBOT");
	geometry_msgs::Twist rotation;

	rotation.linear.x = 0;
	rotation.angular.z = E2_ANGULAR_VELOCITY;

	Publisher_base_control.publish(rotation);
}

/*
 * Stop the robot motors
 */
void Navigation::RobotStop(void)
{
	ROS_INFO(" *** Engine Shutdown. Motor offline. ROBOT Stopped");
	geometry_msgs::Twist stop;

	stop.linear.x = 0;
	stop.angular.z = 0;

	Publisher_base_control.publish(stop);
}

/*
 * Checks if there's a face recognition, if not it published a message to face_recognition node to query a new search
 */
void Navigation::DetectUser(bool rotate)
{

	ros::Rate LoopRate(1.0/RUN_PERIOD_DEFAULT);
	ros::Time time_backtracking = ros::Time::now() + ros::Duration(10);

	ac->cancelAllGoals();

	while (ros::ok() && !user_recognized)
	{
		ROS_INFO(" *** User not detected...Continue Looking");

		face_recognition::FRClientGoal fr_goal;
		fr_goal.order_id=0;
		fr_goal.order_argument="none";

		Publisher_fr_order.publish(fr_goal);

		if(rotate)
			RobotRotate();

	    ros::spinOnce();
	    LoopRate.sleep();

	}

	RobotStop();

}

/*
 * Use previous detected position to recover the user.
 */
void Navigation::RecoverUser(void)
{
	ROS_INFO(" *** Start backtracking user... ");

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac_2("move_base", true);

	//wait for the action server to come up
	while (!ac_2.waitForServer(ros::Duration(5.0))) {
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	ac_2.sendGoal(last_user_detection);

	backtracking=true;
	user_recognized=false;

	DetectUser(false);

}

/*
  * If this function is used a new face is added to the database
  */
void Navigation::TrainUserFace(void)
{

	face_recognition::FRClientGoal fr_goal;

    //==========================================================================
    // Center Face
    //==========================================================================

	//==========================================================================
	// Delete previous Pictures
	//==========================================================================

	//==========================================================================
	// Save new face
	//==========================================================================

	fr_goal.order_id = 2;
	fr_goal.order_argument = user_name;

	Publisher_fr_order.publish(fr_goal);


	// Wait end operation and timer


	//==========================================================================
	// Retrain Database Faces
	//==========================================================================
	fr_goal.order_id = 3;
	fr_goal.order_argument = user_name;

	Publisher_fr_order.publish(fr_goal);

	// Wait end operation

}

/*
 * Load the position of the stands from config file
 */
void Navigation::getStandPosition(YAML::Node& doc)
{
	stand_positions = new Marker [doc.size()];
	stand_size = doc.size();

	// Load Markers from map file
	for(unsigned i=0;i<doc.size();i++)
		doc[i] >> stand_positions[i];

}

/*
  * Get the stand position by name
  */
move_base_msgs::MoveBaseGoal Navigation::getStand(std::string name)
{
	move_base_msgs::MoveBaseGoal goal;
	int i=0;

	for (i; i < stand_size; ++i) {

		if(strcmp(stand_positions[i].name.c_str(),name.c_str()) == 0 )
			break;
	}

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = stand_positions[i].position.x;
	goal.target_pose.pose.position.y = stand_positions[i].position.y;

	// To Add orientation
	goal.target_pose.pose.orientation.z = 0.71;
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

	//tell the action client that we want to spin a thread by default
	MoveBaseClient client("move_base", true);

	node.Prepare(&client);

	ros::Rate LoopRate(1.0/RUN_PERIOD_DEFAULT);

	while(ros::ok() && !node.task_completed)
	{
		if(node.autonomous)
			node.RobotController();

		ros::spinOnce();
		LoopRate.sleep();

	}

	return 0;

}
