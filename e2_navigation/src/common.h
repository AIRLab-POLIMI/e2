
/*
 * common.h - AIRLab (Politecnico di Milano)
 * 
 * description
 *
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 27/feb/2014
 *
 */

#ifndef COMMON_H_
#define COMMON_H_

#include "ros/ros.h"
#include "ros/package.h"

#include <fstream>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef move_base_msgs::MoveBaseGoal MBGoal;

using namespace std;

#endif /* COMMON_H_ */
