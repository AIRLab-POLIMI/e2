/*
 * NeckInterface.cpp - AIRLab (Politecnico di Milano)
 * 
 * description
 *
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 10/mar/2014
 *
 */

#include "NeckInterface.h"

// ========================================================================
// Default constructor
// ========================================================================
NeckInterface::NeckInterface(string name) : nh_(),as_(nh_, name, boost::bind(&NeckInterface::executeCB, this, _1), false)
{
	goal_id_ = -99;
	as_.start();			//starting the actionlib server

	pE2PololuInterface = new E2_Pololu_Interface();

	ROS_INFO("[INeck]:: Neck interface ready");

}

NeckInterface::~NeckInterface()
{
	delete pE2PololuInterface;
	ROS_INFO("[INeck]:: Neck interface ended");
}

// ========================================================================
//	Execute action using actionlib
// ========================================================================
void NeckInterface::executeCB(const e2_neck_controller::NeckGoalConstPtr &goal)
{
	// Check if current goal is still active
	if (as_.isPreemptRequested())
	{
		ROS_INFO("[INeck]:: There's an active action. Abort");
		as_.setPreempted();
		return;
	}
	goal_id_ = goal->action_id ;

	//check if the name of the person has been provided for the add-face-images goal
	if (goal_id_ > 10 || goal_id_ < 0 )
	{
		ROS_INFO("[INeck]:: %d is not a valid action. Abort",goal_id_);
		as_.setPreempted();
		return;
	}
	ROS_INFO("[INeck]:: Received action %d ",goal_id_);

	switch(goal_id_)
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
	}

	usleep(ACTION_TIME);
	ROS_INFO("[INeck]:: Action completed");
	as_.setSucceeded();

}

// ========================================================================
// Menu Choices (examples of how to call E2_Pololu_Interface's functions)
// ========================================================================
void NeckInterface::menuChoice1( E2_Pololu_Interface* e2PololuInterface )
{
	ROS_INFO("[INeck]:: 1 - Reach Straight Neck Position");

	#ifdef _DEVELOPMENT_MODE_
		e2PololuInterface->runSubroutine( e2PololuInterface->LEDON );
	#else
		e2PololuInterface->runNeckSubroutine( e2PololuInterface->STRAIGHTNECK );
	#endif
}

void NeckInterface::menuChoice2( E2_Pololu_Interface* e2PololuInterface )
{
	ROS_INFO("[INeck]:: 2 - Invitation Left");

	#ifdef _DEVELOPMENT_MODE_
		e2PololuInterface->runSubroutine( e2PololuInterface->LEDOFF );
	#else
		e2PololuInterface->runNeckSubroutine( e2PololuInterface->INVITATIONLEFT );
	#endif
}

void NeckInterface::menuChoice3( E2_Pololu_Interface* e2PololuInterface )
{
	ROS_INFO("[INeck]:: 3 - Invitation Right");

	#ifdef _DEVELOPMENT_MODE_
		e2PololuInterface->runSubroutine( e2PololuInterface->BLINKSLOW10SECONDS );
	#else
		e2PololuInterface->runNeckSubroutine( e2PololuInterface->INVITATIONRIGHT );
	#endif
}

void NeckInterface::menuChoice4( E2_Pololu_Interface* e2PololuInterface )
{
	ROS_INFO("[INeck]:: 4 - Give a bow");

	#ifdef _DEVELOPMENT_MODE_
		e2PololuInterface->runSubroutine( e2PololuInterface->LEDBLINKFASTNSECONDS, 5 );
	#else
		e2PololuInterface->runNeckSubroutine( e2PololuInterface->GIVE_A_BOW );
	#endif
}

void NeckInterface::menuChoice5( E2_Pololu_Interface* e2PololuInterface )
{
	ROS_INFO("[INeck]:: 5 - Express Surprise");

	#ifdef _DEVELOPMENT_MODE_
		e2PololuInterface->runSubroutine( e2PololuInterface->LEDBLINKFASTNSECONDS, 1 );
	#else
		e2PololuInterface->runNeckSubroutine( e2PololuInterface->SURPRISEBEHAVIOUR );
	#endif
}

void NeckInterface::menuChoice6( E2_Pololu_Interface* e2PololuInterface )
{
	ROS_INFO("[INeck]:: 6 - Express Surprise");

	#ifdef _DEVELOPMENT_MODE_
		e2PololuInterface->runSubroutine( e2PololuInterface->LEDBLINKSLOWNSECONDS, 5 );
	#else
		e2PololuInterface->runNeckSubroutine( e2PololuInterface->SURPRISEBEHAVIOUR );
	#endif
}

void NeckInterface::menuChoice7( E2_Pololu_Interface* e2PololuInterface )
{
	ROS_INFO("[INeck]:: 7 - Express Surprise");

	#ifdef _DEVELOPMENT_MODE_
		e2PololuInterface->runSubroutine( e2PololuInterface->LEDBLINKSLOWNSECONDS, 1 );
	#else
		e2PololuInterface->runNeckSubroutine( e2PololuInterface->SURPRISEBEHAVIOUR );
	#endif
}

void NeckInterface::menuChoice8( E2_Pololu_Interface* e2PololuInterface )
{
	ROS_INFO("[INeck]:: 8 - Express Surprise");

	#ifdef _DEVELOPMENT_MODE_
		e2PololuInterface->runSubroutine( e2PololuInterface->SETLED, 1 );
	#else
		e2PololuInterface->runNeckSubroutine( e2PololuInterface->SURPRISEBEHAVIOUR );
	#endif
}

void NeckInterface::menuChoice9( E2_Pololu_Interface* e2PololuInterface )
{
	ROS_INFO("[INeck]:: 9 - Express Surprise");

	#ifdef _DEVELOPMENT_MODE_
		e2PololuInterface->runSubroutine( e2PololuInterface->SETLED, 0 );
	#else
		e2PololuInterface->runNeckSubroutine( e2PololuInterface->SURPRISEBEHAVIOUR );
	#endif
}
