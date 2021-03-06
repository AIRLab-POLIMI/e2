/*
 * face_rec_client.cpp - AIRLab (Politecnico di Milano)
 *
 *  Author:  Lorenzo Ripani
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 27/feb/2014
 *
 */
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <face_recognition/FaceRecognitionAction.h>
#include <signal.h>

face_recognition::FaceRecognitionGoal goal; //Goal message
actionlib::SimpleActionClient<face_recognition::FaceRecognitionAction> * ac; //action lib client

//=================================================
// Called once when the goal completes
//=================================================
void doneCb(const actionlib::SimpleClientGoalState& state, 	const face_recognition::FaceRecognitionResultConstPtr& result)
{
	ROS_INFO("Goal [%i] Finished in state [%s]", result->order_id,state.toString().c_str());
	if(state.toString() != "SUCCEEDED") return;
	if( result->order_id==0)
		ROS_INFO("%s was recognized with confidence %f at distance %f", result->names[0].c_str(),result->confidence[0],result->distance[0]);
	if( result->order_id==2)
		ROS_INFO("Pictures of %s were successfully added to the training images",result->names[0].c_str());
}
//=================================================
// Called once when the goal becomes active
//=================================================
void activeCb()
{
	ROS_INFO("Goal just went active");
}

//=================================================
// Called every time feedback is received for the goal
//=================================================
void feedbackCb(const face_recognition::FaceRecognitionFeedbackConstPtr& feedback)
{
	ROS_INFO("Received feedback from Goal [%d] ", feedback->order_id);
	if(feedback->order_id==1 )
		ROS_INFO("%s was recognized with confidence %f at distance %f", feedback->names[0].c_str(),feedback->confidence[0],feedback->distance[0]);
	if( feedback->order_id==2)
		ROS_INFO("A picture of %s was successfully added to the training images",feedback->names[0].c_str());
}
//=================================================
// called for every FRClientGoal message received by the client.
// Client processes each message and sends the corresponding
// goal to the server and registers feedback and result and status call back functions.
//=================================================
void frclientCallback(const face_recognition::FaceRecognitionGoalConstPtr& msg)
{
	ROS_INFO("request for sending goal [%i] is received", msg->order_id);
	goal.order_id = msg->order_id;
	goal.order_argument = msg->order_argument;
	ac->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
}
//=================================================
//shut down
//=================================================
void exit_handler(int s)
{
	delete(ac);
	ros::shutdown();
}

//=================================================
// Main
//=================================================
int main (int argc, char **argv)
{
	ros::init(argc, argv, "face_recognition_client");
	ros::NodeHandle n;

	ac = new actionlib::SimpleActionClient<face_recognition::FaceRecognitionAction>("face_recognition", true);
	//for proper shutdown exit_handler is used
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = exit_handler;

	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	//wait for the server
	ac->waitForServer();
	//subscribe to the topic of interest
	ros::Subscriber sub = n.subscribe("fr_order", 1, frclientCallback);

	ros::spin();
	return 0;

}
