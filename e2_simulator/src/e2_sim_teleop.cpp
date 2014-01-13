/*************************************************************
 * This node control simulated robot on vrep space using
 * the keyboard.
 *
 * by Lorenzo Ripani - AIRLab (Politecnico di Milano)
 * email: ripani.lorenzo@gmail.com
 *
 * version 0.1 - 12/2014
 *************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "vrep_common/v_repConst.h"

// Used data structures:
#include "vrep_common/ProximitySensorData.h"
#include "vrep_common/VrepInfo.h"
#include "vrep_common/JointSetStateData.h"

// Used API services:
#include "vrep_common/simRosEnablePublisher.h"
#include "vrep_common/simRosEnableSubscriber.h"

// Teleop
#include <signal.h>
#include <termios.h>
#include <geometry_msgs/Twist.h>

#define RATE 5000

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_SPACE 0x20
#define KEYCODE_MINUS 0xBD
#define KEYCODE_PLUS 0xBB

// Global variables
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;

int kfd = 0;
struct termios cooked, raw;


// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info){
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

// Get messages from sensor
void sensorCallback(const vrep_common::ProximitySensorData::ConstPtr& sens)
{
	sensorTrigger=true;
}

// Main code:
int main(int argc,char* argv[]){

	int leftMotorHandle;
	int rightMotorHandle;
	int sensorHandle;

	if (argc>=3)
	{
		leftMotorHandle=atoi(argv[1]);
		rightMotorHandle=atoi(argv[2]);
		sensorHandle=atoi(argv[3]);
	}else{
		printf("Indicate following arguments: 'leftMotorHandle rightMotorHandle sensorHandle'!\n");
		sleep(5000);
		return 0;
	}

	// Create a ROS node. The name has a random component: 
	int _argc = 0;
	char** _argv = NULL;
	struct timeval tv;
	unsigned int timeVal=0;

	if (gettimeofday(&tv,NULL)==0)
		timeVal=(tv.tv_sec*1000+tv.tv_usec/1000)&0x00ffffff;

	std::string nodeName("e2");

	ros::init(_argc,_argv,nodeName.c_str());

	if(!ros::master::check())
		return(0);
	

	ros::NodeHandle node("~");	
	ROS_INFO("E2 just started with node name %s\n",nodeName.c_str());

	ros::Subscriber subInfo=node.subscribe("/vrep/info",1,infoCallback);

	ros::ServiceClient client_enablePublisher=node.serviceClient<vrep_common::simRosEnablePublisher>("/vrep/simRosEnablePublisher");
	vrep_common::simRosEnablePublisher srv_enablePublisher;
	srv_enablePublisher.request.topicName="proxData"; // the requested topic name
	srv_enablePublisher.request.queueSize=1; // the requested publisher queue size (on V-REP side)
	srv_enablePublisher.request.streamCmd=simros_strmcmd_read_proximity_sensor; // the requested publisher type
	srv_enablePublisher.request.auxInt1=sensorHandle; // some additional information the publisher needs (what proximity sensor)

	if ( client_enablePublisher.call(srv_enablePublisher)&&(srv_enablePublisher.response.effectiveTopicName.length()!=0) )
	{
		std::string topicName("/vrep/");
		topicName+=srv_enablePublisher.response.effectiveTopicName; // Make sure to use the returned topic name, not the requested one (can be same)
		ros::Subscriber sub=node.subscribe(topicName.c_str(),1,sensorCallback);

		ros::ServiceClient client_enableSubscriber=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
		vrep_common::simRosEnableSubscriber srv_enableSubscriber;

		srv_enableSubscriber.request.topicName="/"+nodeName+"/wheels"; // the topic name
		srv_enableSubscriber.request.queueSize=1; // the subscriber queue size (on V-REP side)
		srv_enableSubscriber.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type

		if ( client_enableSubscriber.call(srv_enableSubscriber)&&(srv_enableSubscriber.response.subscriberID!=-1) )
		{
			ros::Publisher motorSpeedPub=node.advertise<vrep_common::JointSetStateData>("wheels",1);

			float driveBackStartTime=-99.0f;
			vrep_common::JointSetStateData motorSpeeds;


			// TASTIERA
			// get the console in raw mode
			tcgetattr(kfd, &cooked);
			memcpy(&raw, &cooked, sizeof(struct termios));
			raw.c_lflag &=~ (ICANON | ECHO);
			// Setting a new line, then end of file
			raw.c_cc[VEOL] = 1;
			raw.c_cc[VEOF] = 2;
			tcsetattr(kfd, TCSANOW, &raw);

			puts("Reading from keyboard");
			puts("---------------------------");
			puts("Use arrow keys to move E2 :");
			char c;
			float standardVelocity=10;

			while (ros::ok()&&simulationRunning)
			{

				// this is the control loop (very simple, just as an example)

				float desiredLeftMotorSpeed;
				float desiredRightMotorSpeed;

				// get the next event from the keyboard
				if(read(kfd, &c, 1) < 0)
				{
					perror("read():");
					exit(-1);
				}

				ROS_DEBUG("value: 0x%02X\n", c);

				switch(c)
				{
				      case KEYCODE_L:
				        ROS_DEBUG("LEFT");
						desiredLeftMotorSpeed=-standardVelocity*0.25;
						desiredRightMotorSpeed=standardVelocity*0.25;
				        break;
				      case KEYCODE_R:
				        ROS_DEBUG("RIGHT");
						desiredLeftMotorSpeed=standardVelocity*0.25;
						desiredRightMotorSpeed=-standardVelocity*0.25;
				        break;
				      case KEYCODE_U:
				        ROS_DEBUG("UP");
						desiredLeftMotorSpeed=standardVelocity;
						desiredRightMotorSpeed=standardVelocity;
				        break;
				      case KEYCODE_D:
				        ROS_DEBUG("DOWN");
						desiredLeftMotorSpeed=-standardVelocity;
						desiredRightMotorSpeed=-standardVelocity;
				        break;
				      case KEYCODE_PLUS:
				        ROS_DEBUG("INCREASE VELOCITY");
				        standardVelocity+=1;
				        desiredLeftMotorSpeed=standardVelocity;
				        desiredRightMotorSpeed=standardVelocity;
				        break;
				      case KEYCODE_MINUS:
				        ROS_DEBUG("DECREASE VELOCITY");
				        standardVelocity-=1;
   				        desiredLeftMotorSpeed=standardVelocity;
   				        desiredRightMotorSpeed=standardVelocity;
   				        break;
				}

				// publish the motor speeds:
				motorSpeeds.handles.data.push_back(leftMotorHandle);
				motorSpeeds.handles.data.push_back(rightMotorHandle);
				motorSpeeds.setModes.data.push_back(2); // 2 is the speed mode
				motorSpeeds.setModes.data.push_back(2);
				motorSpeeds.values.data.push_back(desiredLeftMotorSpeed);
				motorSpeeds.values.data.push_back(desiredRightMotorSpeed);
				motorSpeedPub.publish(motorSpeeds);

				// handle ROS messages:
				ros::spinOnce();

				// Sleep a bit
				usleep(RATE);
			}

		}
	}

	ros::shutdown();
	ROS_INFO("Node just ended....");

	return(0);

}



