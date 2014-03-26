/*************************************************************
 *  Basic differential controller used by vrep simulator
 *
 *  by Lorenzo Ripani - AIRLab (Politecnico di Milano)
 *  email: ripani.lorenzo@gmail.com
 *
 *  version 0.1 - 12/2014
 *************************************************************/

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

// Data structures:
#include "vrep_common/VrepInfo.h"
#include "vrep_common/v_repConst.h"
#include "vrep_common/JointSetStateData.h"

// API services:
#include "vrep_common/simRosEnablePublisher.h"
#include "vrep_common/simRosEnableSubscriber.h"

#define RATE 50 // Hz
#define ROBOT_WIDTH 0.6

// Global variables (modified by topic subscribers):
bool simulationRunning=true;
float simulationTime=0.0f;

int leftMotorHandle,rightMotorHandle,neckMotorHandle;
float LeftMotorSpeed,RightMotorSpeed,neckMotorSpeed;
ros::Publisher motorSpeedPub;
vrep_common::JointSetStateData motorSpeeds,neckSpeed;


/*
 * Send messages to vrep simulator, for neck rotation
 */
void neckCallback(const geometry_msgs::Twist::ConstPtr& msg){
	ROS_INFO("Received neck velocity command:%f", msg->angular.z);

	neckMotorSpeed = msg->angular.z * 5;

	neckSpeed.handles.data.push_back(neckMotorHandle);
	neckSpeed.setModes.data.push_back(2);

	neckSpeed.values.data.push_back(neckMotorSpeed);
	motorSpeedPub.publish(neckSpeed);
}

/*
 * Send messages to vrep simulator in order to move robot base
 */
void controllerCallback(const geometry_msgs::Twist::ConstPtr& msg){

	ROS_DEBUG("Received Twist Message");

	LeftMotorSpeed=0;
	RightMotorSpeed=0;

	// Control loop e send back to simulator
	LeftMotorSpeed = msg->linear.x - msg->angular.z * (ROBOT_WIDTH/2);
	RightMotorSpeed = msg->linear.x + msg->angular.z * (ROBOT_WIDTH/2);

	LeftMotorSpeed *=24;
	RightMotorSpeed *=24;


	motorSpeeds.handles.data.push_back(leftMotorHandle);
	motorSpeeds.handles.data.push_back(rightMotorHandle);
	motorSpeeds.setModes.data.push_back(2); // 2 is the speed mode
	motorSpeeds.setModes.data.push_back(2);
	motorSpeeds.values.data.push_back(LeftMotorSpeed);
	motorSpeeds.values.data.push_back(RightMotorSpeed);
	motorSpeedPub.publish(motorSpeeds);

}

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info){
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}


//===================================================
//	Main Code
//===================================================
int main(int argc,char* argv[]){

	if (argc>=3){
		leftMotorHandle=atoi(argv[1]);
		rightMotorHandle=atoi(argv[2]);
		neckMotorHandle=atoi(argv[3]);
	}else{
		printf("Indicate following arguments: 'leftMotorHandle rightMotorHandle NeckMotor'!\n");
		sleep(5000);
		return 0;
	}

	// Create a ROS node :
	int _argc = 0;
	char** _argv = NULL;
	struct timeval tv;
	unsigned int timeVal=0;

	if (gettimeofday(&tv,NULL)==0)
		timeVal=(tv.tv_sec*1000+tv.tv_usec/1000)&0x00ffffff;

	std::string nodeName("e2_base_controller");
	ros::init(_argc,_argv,nodeName);

	if(!ros::master::check())
		return(0);


	ros::NodeHandle node("~");
	printf("E2 Base Controller started.");


	ros::Subscriber subInfo=node.subscribe("/vrep/info",1,infoCallback);

	//  Let's tell V-REP to subscribe to the motor speed topic
	ros::ServiceClient client_enableSubscriber=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
	vrep_common::simRosEnableSubscriber srv_enableSubscriber,srv_enableSubscriber_neck;

	srv_enableSubscriber.request.topicName="/"+nodeName+"/joints"; // the topic name
	srv_enableSubscriber.request.queueSize=1; // the subscriber queue size (on V-REP side)
	srv_enableSubscriber.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type


	if ( client_enableSubscriber.call(srv_enableSubscriber)&&(srv_enableSubscriber.response.subscriberID!=-1) )
	{
		// Prepare a publisher of those motor speeds:
		motorSpeedPub=node.advertise<vrep_common::JointSetStateData>("joints",1);

		ros::Subscriber twist_sub = node.subscribe("/cmd_vel", RATE, controllerCallback);
		ros::Subscriber neck_twist = node.subscribe("/e2/neck", RATE, neckCallback);

		ros::Rate r(RATE);

		while (ros::ok()&&simulationRunning)
		{

			// handle ROS messages:
			ros::spinOnce();

			// sleep a bit:
			r.sleep();
		}

	}



}




