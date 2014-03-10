/*
 * neck.cpp - AIRLab (Politecnico di Milano)
 *
 *	This node receive messages on default topic and controls neck moviments for e2 robot
 *
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 28/feb/2014
 *
 */

#include "ros/ros.h"
#include "NeckInterface.h"
#include "E2_Pololu_Interface.h"

#define ROS_NODE_NAME	"e2_neck_controller"

using namespace std;

E2_Pololu_Interface* pE2PololuInterface ;

void menuChoice1( E2_Pololu_Interface* e2PololuInterface );
void menuChoice2( E2_Pololu_Interface* e2PololuInterface );
void menuChoice3( E2_Pololu_Interface* e2PololuInterface );
void menuChoice4( E2_Pololu_Interface* e2PololuInterface );
void menuChoice5( E2_Pololu_Interface* e2PololuInterface );
void menuChoice6( E2_Pololu_Interface* e2PololuInterface );
void menuChoice7( E2_Pololu_Interface* e2PololuInterface );
void menuChoice8( E2_Pololu_Interface* e2PololuInterface );
void menuChoice9( E2_Pololu_Interface* e2PololuInterface );


//=====================================
// Main Code
//=====================================
int main(int argc, char **argv)
{
	ros::init(argc, argv, ROS_NODE_NAME);

    //pE2PololuInterface = new E2_Pololu_Interface();

	ROS_INFO("["ROS_NODE_NAME"]:: Node Started");

	// Initialize main  class
	NeckInterface neck(ros::this_node::getName());

	ros::spin();

	delete pE2PololuInterface;
	return 0;


}

/*
void NeckCb(e2_neck_controller::NeckAction::ConstPtr &msg)
{
	int choice= msg->action_id;
	ROS_INFO("["ROS_NODE_NAME"]:: Received Action %d ",choice);

	switch(choice)
	{
		case '1':
			menuChoice1( pE2PololuInterface );
			break;
		case '2':
			menuChoice2( pE2PololuInterface );
			break;
		case '3':
			menuChoice3( pE2PololuInterface );
			break;
		case '4':
			menuChoice4( pE2PololuInterface );
			break;
		case '5':
			menuChoice5( pE2PololuInterface );
			break;
		case '6':
			menuChoice6( pE2PololuInterface );
			break;
		case '7':
			menuChoice7( pE2PololuInterface );
			break;
		case '8':
			menuChoice8( pE2PololuInterface );
			break;
		case '9':
			menuChoice9( pE2PololuInterface );
			break;
		case '10':
			quit = true;
			break;
	}
}
*/

// ========================================================================
// Menu Choices (examples of how to call E2_Pololu_Interface's functions)
// ========================================================================
void menuChoice1( E2_Pololu_Interface* e2PololuInterface )
{
	printf("1 - Reach Straight Neck Position\n\n");

#ifdef _DEVELOPMENT_MODE_
	e2PololuInterface->runSubroutine( e2PololuInterface->LEDON );
#else
	e2PololuInterface->runNeckSubroutine( e2PololuInterface->STRAIGHTNECK );
#endif
}

void menuChoice2( E2_Pololu_Interface* e2PololuInterface )
{
	printf("2 - Invitation Left\n\n");

#ifdef _DEVELOPMENT_MODE_
	e2PololuInterface->runSubroutine( e2PololuInterface->LEDOFF );
#else
	e2PololuInterface->runNeckSubroutine( e2PololuInterface->INVITATIONLEFT );
#endif
}

void menuChoice3( E2_Pololu_Interface* e2PololuInterface )
{
	printf("3 - Invitation Right\n\n");

#ifdef _DEVELOPMENT_MODE_
	e2PololuInterface->runSubroutine( e2PololuInterface->BLINKSLOW10SECONDS );
#else
	e2PololuInterface->runNeckSubroutine( e2PololuInterface->INVITATIONRIGHT );
#endif
}

void menuChoice4( E2_Pololu_Interface* e2PololuInterface )
{
	printf("4 - Give a bow\n\n");

#ifdef _DEVELOPMENT_MODE_
	e2PololuInterface->runSubroutine( e2PololuInterface->LEDBLINKFASTNSECONDS, 5 );
#else
	e2PololuInterface->runNeckSubroutine( e2PololuInterface->GIVE_A_BOW );
#endif
}

void menuChoice5( E2_Pololu_Interface* e2PololuInterface )
{
	printf("5 - Express Surprise\n\n");

#ifdef _DEVELOPMENT_MODE_
	e2PololuInterface->runSubroutine( e2PololuInterface->LEDBLINKFASTNSECONDS, 1 );
#else
	e2PololuInterface->runNeckSubroutine( e2PololuInterface->SURPRISEBEHAVIOUR );
#endif
}

void menuChoice6( E2_Pololu_Interface* e2PololuInterface )
{
	printf("6 - Express Surprise\n\n");

#ifdef _DEVELOPMENT_MODE_
	e2PololuInterface->runSubroutine( e2PololuInterface->LEDBLINKSLOWNSECONDS, 5 );
#else
	e2PololuInterface->runNeckSubroutine( e2PololuInterface->SURPRISEBEHAVIOUR );
#endif
}

void menuChoice7( E2_Pololu_Interface* e2PololuInterface )
{
	printf("7 - Express Surprise\n\n");

#ifdef _DEVELOPMENT_MODE_
	e2PololuInterface->runSubroutine( e2PololuInterface->LEDBLINKSLOWNSECONDS, 1 );
#else
	e2PololuInterface->runNeckSubroutine( e2PololuInterface->SURPRISEBEHAVIOUR );
#endif
}

void menuChoice8( E2_Pololu_Interface* e2PololuInterface )
{
	printf("8 - Express Surprise\n\n");

#ifdef _DEVELOPMENT_MODE_
	e2PololuInterface->runSubroutine( e2PololuInterface->SETLED, 1 );
#else
	e2PololuInterface->runNeckSubroutine( e2PololuInterface->SURPRISEBEHAVIOUR );
#endif
}

void menuChoice9( E2_Pololu_Interface* e2PololuInterface )
{
	printf("9 - Express Surprise\n\n");

#ifdef _DEVELOPMENT_MODE_
	e2PololuInterface->runSubroutine( e2PololuInterface->SETLED, 0 );
#else
	e2PololuInterface->runNeckSubroutine( e2PololuInterface->SURPRISEBEHAVIOUR );
#endif
}


