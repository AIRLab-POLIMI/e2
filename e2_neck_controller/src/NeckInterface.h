/*
 * NeckInterface.h - AIRLab (Politecnico di Milano)
 *
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 10/mar/2014
 *
 */

#ifndef NECKINTERFACE_H_
#define NECKINTERFACE_H_

#include <actionlib/server/simple_action_server.h>

#include "E2_Pololu_Interface.h"

#include "e2_neck_controller/NeckGoal.h"
#include "e2_neck_controller/NeckResult.h"
#include "e2_neck_controller/NeckAction.h"
#include "e2_neck_controller/NeckActionFeedback.h"

#define ACTION_TIME 10000000							// Define time before action is completed in microsec

using namespace std;

class NeckInterface
{
	public:

		NeckInterface(string name);
		~NeckInterface();

		void executeCB(const e2_neck_controller::NeckGoalConstPtr &goal);

	private:
		int goal_id_;
		E2_Pololu_Interface* pE2PololuInterface ;

	protected:
		ros::NodeHandle nh_;
		e2_neck_controller::NeckResult result_;
		e2_neck_controller::NeckFeedback feedback_;
		actionlib::SimpleActionServer<e2_neck_controller::NeckAction> as_;
};

#endif /* NECKINTERFACE_H_ */
