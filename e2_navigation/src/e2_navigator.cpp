/*************************************************************
 * This node define the robot behaviours during navigation
 *
 * by Lorenzo Ripani - AIRLab (Politecnico di Milano)
 * email: ripani.lorenzo@gmail.com
 *
 * version 0.1 - 12/2013
 *************************************************************/
  
#include "ros/ros.h"

#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "e2_navigator"

#define MIN_USER_DISTANCE 0.5 // Min distance for user detection
#define MAX_USER_DISTANCE 1.5 // Max distance for user detection
#define TIME_CHECK_OBJECT 10 // Define time interval (in seconds) before check user if no object is detected by sonar
#define TIME_CHECK_USER 60 // Define time interval (in seconds) before fire a find user

// Simulation Include for sensors
#include "vrep_common/v_repConst.h"
#include "vrep_common/ProximitySensorData.h"
#include "vrep_common/VrepInfo.h"

// Include message definitions

#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <face_recognition/FRClientGoal.h>
#include <face_recognition/FaceRecognitionActionResult.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

class ROSnode
/* includes all the elements needed to define a basic ROS node */
{
  private: 
    ros::NodeHandle Handle;
    
    ros::Subscriber Subscriber_odom;
    ros::Subscriber Subscriber_sensor0;
    ros::Subscriber Subscriber_sensor1;
    ros::Subscriber Subscriber_sensor2;
    ros::Subscriber Subscriber_fr_status; // Face recognition status
    
    ros::Publisher Publisher_fr_order; // Face recognition order
    ros::Publisher Publisher_nav_status;
    ros::Publisher Publisher_base_control;
    ros::Publisher Publisher_neck_control; // Neck joint topic

    ros::Timer TimeoutTimer;


    geometry_msgs::PoseStamped robot_pose;
    ros::Time last_object_detection;

    bool object_detected;
    bool user_recognized;
    bool path_planned;
    bool backtracking;

    std::string user_name;
    std::string navigation_status;
    std_msgs::String msg;

    void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void SensorCallback(const vrep_common::ProximitySensorData::ConstPtr& sens);
    void FRCallback(const face_recognition::FaceRecognitionActionResult::ConstPtr& msg);
    
  public:

    double RunPeriod;
    move_base_msgs::MoveBaseGoal rnd_pos[2],goal,starting_position,last_user_detection;

    void Prepare(void);
    void Shutdown(void);
    void RunPeriodically(float Period);
    void CheckUserPresence(float Period, bool rotate = true , bool swing_neck=false);
    void RotateRobot(void);
    void StopRobot(void);
    void BacktrackUser(void);
    void SwingNeck(void);
};

//-----------------------------------------------------------------
// End node class
//-----------------------------------------------------------------

void ROSnode::Prepare(void)
{
  RunPeriod = RUN_PERIOD_DEFAULT;

  std::string FullParamName;

  object_detected=false;
  user_recognized=false;
  path_planned=false;
  backtracking=false;

  user_name="Lorenzo";
  //user_name="Guest";

  Subscriber_odom = Handle.subscribe("/odom", 10, &ROSnode::OdomCallback, this);
  Subscriber_sensor0 = Handle.subscribe("/e2/sensor_0", 10, &ROSnode::SensorCallback, this);
  Subscriber_sensor1= Handle.subscribe("/e2/sensor_1", 10, &ROSnode::SensorCallback, this);
  Subscriber_sensor2 = Handle.subscribe("/e2/sensor_2", 10, &ROSnode::SensorCallback, this);
  Subscriber_fr_status = Handle.subscribe("/face_recognition/result", 1, &ROSnode::FRCallback, this);
  
  Publisher_fr_order = Handle.advertise<face_recognition::FRClientGoal>("/fr_order", 1);
  Publisher_nav_status = Handle.advertise<std_msgs::String>("/e2/navigator/status", 1);
  Publisher_base_control = Handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  Publisher_neck_control = Handle.advertise<geometry_msgs::Twist>("/e2/neck", 1);

  ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());


}

void ROSnode::RunPeriodically(float Period)
{
  ros::Rate LoopRate(1.0/Period);
  ros::Time wait_time;
  
  ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);
  
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
	  ROS_INFO("Waiting for the move_base action server to come up");
  }

  //Set some position to test robot navigation

  //Airlab Stand
  rnd_pos[0].target_pose.header.frame_id = "map";
  rnd_pos[0].target_pose.header.stamp = ros::Time::now();
  rnd_pos[0].target_pose.pose.position.x = -3.97;
  rnd_pos[0].target_pose.pose.position.y = -6.64;
  rnd_pos[0].target_pose.pose.orientation.z = -0.69;
  rnd_pos[0].target_pose.pose.orientation.w = 0.72;
  //Google Stand
  rnd_pos[1].target_pose.header.frame_id = "map";
  rnd_pos[1].target_pose.header.stamp = ros::Time::now();
  rnd_pos[1].target_pose.pose.position.x = -4.41;
  rnd_pos[1].target_pose.pose.position.y = 5.35;
  rnd_pos[1].target_pose.pose.orientation.z = 0.71;
  rnd_pos[1].target_pose.pose.orientation.w = 0.7;
  //Polimi Stand
  rnd_pos[2].target_pose.header.frame_id = "map";
  rnd_pos[2].target_pose.header.stamp = ros::Time::now();
  rnd_pos[2].target_pose.pose.position.x = 3.34;
  rnd_pos[2].target_pose.pose.position.y = -4.56;
  rnd_pos[2].target_pose.pose.orientation.z = -0.71;
  rnd_pos[2].target_pose.pose.orientation.w = 0.7;

  //Set random goal
  srand ( time(NULL) );
  int goal_number = rand() % 2+1;
  goal = rnd_pos[0];


  backtracking=true;
  CheckUserPresence(Period);

  ROS_INFO("E2: Control Loop...");

  while (ros::ok())
  {

	  if((ros::Time::now() > ( last_user_detection.target_pose.header.stamp + ros::Duration(TIME_CHECK_USER))) && path_planned) // Default user checking
	  {
		  ROS_INFO("E2: Aborting current goal. Who is following me ? Let's check...");
		  ac.cancelAllGoals();
		  path_planned = false;

		  navigation_status.assign("default_backtracking");
		  CheckUserPresence(Period);
	  }
	  else if(user_recognized && !path_planned)
	  {
			ROS_INFO("E2: I'm going home dude");
			ac.sendGoal(goal);
			path_planned = true;
			user_recognized = false;
			navigation_status.assign("navigating");
	  }
	  else if (ros::Time::now() > ( last_object_detection + ros::Duration(TIME_CHECK_OBJECT)))
	  {
		  ROS_INFO("E2: Aborting current goal. No object detected in 10s...check for user");
		  ac.cancelAllGoals();
		  path_planned = false;

		  navigation_status.assign("check_user");
		  CheckUserPresence(Period);
	  }

	  // Check if the robot succeded it's task
	  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	  {
		  ROS_INFO("Hooray, we reache the Goal ! ");
		  navigation_status.assign("completed");
		  break;
	  }
	  else
		ROS_DEBUG("The woods are lovely, dark and deep. But i have promise to keep, and miles to go before i sleep. F.");


	  msg.data = navigation_status;
	  Publisher_nav_status.publish(msg);

	  object_detected=false;

	  ros::spinOnce();
	  LoopRate.sleep();
  }

  CheckUserPresence(Period); // Get person in front of me

}

void ROSnode::Shutdown(void)
{
  ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
  
  /* 
   * TODO - Clean File used for training faces
   */
}


/*
 * This function takes odometry messages from the robot and update the current node position.
 */
void ROSnode::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_DEBUG("Robot Position x,y,z: [%f,%f,%f]: ", msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
	robot_pose.pose = msg->pose.pose;
}

void ROSnode::SensorCallback(const vrep_common::ProximitySensorData::ConstPtr& sens)
{

	double obj_distance = sens->detectedPoint.z;

	if (obj_distance >= MIN_USER_DISTANCE && obj_distance <= MAX_USER_DISTANCE) {

		ROS_DEBUG("Received a detection! Fuck yeah!");
		ROS_DEBUG("Distance from object %f", sens->detectedPoint.z);

		last_object_detection= ros::Time::now();
		object_detected=true;

	}

}

/*
 * This function is suscribed to face_recognition node and watch to see if there was a user recognition
 */
void ROSnode::FRCallback(const face_recognition::FaceRecognitionActionResult::ConstPtr& msg)
{

	if(msg->status.status == 3 && strcmp(msg->result.names[0].c_str(),user_name.c_str())==0)
	{

		ROS_INFO("Detected User: %s",msg->result.names[0].c_str());

		user_recognized=true;
		backtracking = false;

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
 * This func check if there's a regognition, if not it publish a message to face_recognition node to query a new search
 */
void ROSnode::CheckUserPresence(float Period, bool rotate, bool swing_neck)
{

	ros::Rate LoopRate(1.0/Period);
	ros::Time time_backtracking = ros::Time::now() + ros::Duration(10);

	while (ros::ok() && !user_recognized)
	{
		ROS_DEBUG("User not detected...Continue Looking");

		face_recognition::FRClientGoal fr_goal;
		fr_goal.order_id=0;
		fr_goal.order_argument="none";

		Publisher_fr_order.publish(fr_goal);

		if(rotate)
			RotateRobot();

		if(swing_neck)
			SwingNeck();

		if(ros::Time::now() > time_backtracking && !backtracking)
			BacktrackUser();

	    ros::spinOnce();
	    LoopRate.sleep();

	}

}

/*
 * Tell the robot to start rotating on place. Used to check if there's a user near the robot
 */
void ROSnode::RotateRobot(void)
{
	ROS_DEBUG("Rotating ROBOT");
	geometry_msgs::Twist rotation;

	rotation.linear.x = 0;
	rotation.angular.z = 0.5;

	Publisher_base_control.publish(rotation);
}

/*
 * Tell the robot to rotate neck
 */
void ROSnode::SwingNeck(void)
{
	ROS_INFO("Rotating Neck");
	geometry_msgs::Twist rotation;

	rotation.linear.x = 0;
	rotation.angular.z = 0.1;

	ros::Time start_swing = ros::Time::now();


	while(ros::Time::now() < start_swing+ros::Duration(2))
		Publisher_neck_control.publish(rotation);

	rotation.angular.z *= 0;
	Publisher_neck_control.publish(rotation);

	rotation.angular.z = -0.1;
	start_swing = ros::Time::now();

	while(ros::Time::now() < start_swing+ros::Duration(2))
		Publisher_neck_control.publish(rotation);

	rotation.angular.z *= 0;
	Publisher_neck_control.publish(rotation);

	rotation.angular.z *= 0.1;
	start_swing = ros::Time::now();

	while(ros::Time::now() < start_swing+ros::Duration(2))
		Publisher_neck_control.publish(rotation);

	rotation.angular.z *= 0;
	Publisher_neck_control.publish(rotation);
}
/*
 * Stop the robot motors
 */
void ROSnode::StopRobot(void)
{
	ROS_INFO("Engine Shutdown. Motor offline. ROBOT Stopped");
	geometry_msgs::Twist stop;

	stop.linear.x = 0;
	stop.angular.z = 0;

	Publisher_base_control.publish(stop);
}

/*
 * If there's no user near the robot this function is invoked to go pack towards the last user detection and find back the user
 */
void ROSnode::BacktrackUser(void)
{
	ROS_INFO("E2: Start backtracking user... ");

	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac_2("move_base", true);

	//wait for the action server to come up
	while (!ac_2.waitForServer(ros::Duration(5.0))) {
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	ac_2.sendGoal(last_user_detection);
	user_recognized=false;
	backtracking=true;

	CheckUserPresence(RunPeriod,false,true);

}
//-----------------------------------------------------------------
// End node functions
//-----------------------------------------------------------------


// Main function
int main(int argc, char **argv)
{

	// Inizialize the node
	ros::init(argc, argv, NAME_OF_THIS_NODE);
  
	ROSnode navigator;

	// Main operation of the node
	navigator.Prepare();

	// Navigation control Loop
	navigator.RunPeriodically(navigator.RunPeriod);

	// The navigation is over. Shutdown the node
	navigator.Shutdown();

	navigator.StopRobot();

	// TODO - Espeak Voice

	return (0);
}
