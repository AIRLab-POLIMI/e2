/*************************************************************
 * This node locate the robot and update it's pose respect
 * positions markers.
 *
 * by Lorenzo Ripani - AIRLab (Politecnico di Milano)
 * email: ripani.lorenzo@gmail.com
 *
 * version 0.1 - 12/2013
 *************************************************************/

#include "ros/ros.h"
#include "ros/package.h"

#include "ar_track_alvar/AlvarMarker.h"
#include "ar_track_alvar/AlvarMarkers.h"

// Messages include
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"

#include <tf/transform_listener.h>
#include "yaml.h"
#include <yaml-cpp/yaml.h>

// Operation include
#include <iostream>
#include <iterator>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>

// Default definitions
#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "e2_localization"


// Yaml Structures
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

// now the extraction operators for these types
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

//=============================================
//		Localization class
//=============================================
class Localization
{

  private:
	ros::NodeHandle Handle;
	ros::Publisher robot_pose;
	tf::TransformListener listener;
	ros::Subscriber ar_marker_pose;

	//geometry_msgs::PoseWithCovarianceStamped pose;
	geometry_msgs::PoseStamped pose;

  public:
    double RunPeriod;
	Marker *map_marker;
	std::string mapFrame;
	std::string robotFrame;

    void Prepare(void);
    void MarkerCB(const ar_track_alvar::AlvarMarkers &msg);

};

//=============================================
//		End node class
//=============================================

/*
 * Default node inizialization
 */
void Localization::Prepare(void)
{
	ros::NodeHandle HandleP("~");

	RunPeriod = RUN_PERIOD_DEFAULT;

	robot_pose = Handle.advertise<geometry_msgs::PoseStamped>("/e2/localization", 1);
	ar_marker_pose = Handle.subscribe("/ar_pose_marker", 1,&Localization::MarkerCB,this);
	//robot_pose = Handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/e2/localization", 1);

	HandleP.param<std::string>("map_frame", mapFrame, "map");
	HandleP.param<std::string>("robot_frame", robotFrame, "base_footprint");

	ROS_INFO("Map frame: %s",mapFrame.c_str());
	ROS_INFO("Robot frame: %s",robotFrame.c_str());
	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());

	std::string marker_file = ros::package::getPath("e2_navigation")+"/config/marker_positions.yaml";
	std::ifstream fin(marker_file.c_str());

	YAML::Node doc;
	YAML::Parser parser(fin);

	parser.GetNextDocument(doc);
	map_marker = new Marker [doc.size()];

	// Load Markers from map file
	for(unsigned i=0;i<doc.size();i++) {

		doc[i] >> map_marker[i];
		//DEBUG
		//std::cout << markers[i].name << "," << markers[i].id << ", " << markers[i].position.x << "\n";
	}
}

/*
 * Receive messages from Alvar marker node and update robot position
 */
void Localization::MarkerCB(const ar_track_alvar::AlvarMarkers &msg)
{

	int marker_id;
	std::stringstream real_marker_frame,detected_marker_frame;
	std::vector<ar_track_alvar::AlvarMarker> markers = msg.markers;

	geometry_msgs::Pose real_marker;
	tf::StampedTransform transform_detected_marker,transform_real_marker,transform_temp,transform_det_map;

	for(std::vector<ar_track_alvar::AlvarMarker>::iterator it = markers.begin(); it != markers.end(); ++it) {

		ROS_DEBUG("Marker Message");
		ROS_DEBUG("Marker ID: %i",it->id);
		ROS_DEBUG("X: %f",it->pose.pose.position.x);
		ROS_DEBUG("Y: %f",it->pose.pose.position.y);

		marker_id = it->id;

		real_marker_frame.str("");
		detected_marker_frame.str("");

		real_marker_frame << "marker_" << marker_id;
		detected_marker_frame << "ar_marker_" << marker_id ;

	 }

    try
    {

    	tf::Quaternion real_marker_quat;
        tf::Vector3 real_marker_pose;

        if(map_marker[marker_id].id != marker_id)
    	{
    		ROS_INFO("Problem to find a valid marker on the map.");
    		exit(0);
    	}
    	else
    	{
    		real_marker_pose.setX(map_marker[marker_id].position.x);
    		real_marker_pose.setY(map_marker[marker_id].position.y);
    		real_marker_pose.setZ(0);
    	}

        ROS_DEBUG("Real Marker Position");
        ROS_DEBUG("X: %f",real_marker_pose.getX());
        ROS_DEBUG("Y: %f",real_marker_pose.getY());

    	listener.lookupTransform(mapFrame,detected_marker_frame.str(),ros::Time(0), transform_det_map);

        real_marker_pose.setZ(1.5);													// Set some Z value , just for rviz visalization
        transform_det_map.setOrigin(real_marker_pose);

    	listener.lookupTransform(detected_marker_frame.str(),robotFrame,ros::Time(0), transform_detected_marker);

        tf::Transform transform_final;
        transform_final.mult(transform_det_map,transform_detected_marker);

        real_marker_quat = transform_final.getRotation();
        real_marker_pose = transform_final.getOrigin();

        tf::Quaternion yaw;
        yaw.setEuler(tf::getYaw(real_marker_quat), 0, 0);

    	// Fill in the pose message - the covriance is currently not used
    	pose.header.stamp = ros::Time::now();
    	pose.header.frame_id = mapFrame;
    	pose.pose.position.x = real_marker_pose.getX();
    	pose.pose.position.y = real_marker_pose.getY();
    	pose.pose.position.z = 0; // For now remove the z component
    	pose.pose.orientation.x = yaw.getX();
    	pose.pose.orientation.y = yaw.getY();
    	pose.pose.orientation.z = yaw.getZ();
    	pose.pose.orientation.w = yaw.getW();
    	float covariance[] = {
			1e-3, 0, 0, 0, 0, 0,
			0, 1e-3, 0, 0, 0, 0,
			0, 0, 1e-3, 0, 0, 0,
			0, 0, 0, 1e-3, 0, 0,
			0, 0, 0, 0, 1e-3, 0,
			0, 0, 0, 0, 0, 1e-3
    	};

/*
    	for (unsigned int i = 0; i < pose.pose.covariance.size(); i++) {
    		pose.pose.covariance[i] = covariance[i];
    	}
    	ROS_DEBUG("ROBOT POSE");
    	ROS_DEBUG("X: %f",pose.pose.position.x );
    	ROS_DEBUG("Y: %f",pose.pose.position.y );

    	ROS_DEBUG("ROBOT Orientation");
    	ROS_DEBUG("Z: %f",pose.pose.orientation.z );
    	ROS_DEBUG("W: %f",pose.pose.orientation.w );
*/

    	robot_pose.publish(pose);

	}
    catch (tf::TransformException ex)
    {
    	ROS_ERROR("%s",ex.what());
	}

}

//-----------------------------------------------------------------
// End node functions
//-----------------------------------------------------------------


// Main function
int main(int argc, char **argv)
{

	// Inizialize the node
	ros::init(argc, argv, NAME_OF_THIS_NODE);

	Localization node;
	ros::Rate LoopRate(1.0/RUN_PERIOD_DEFAULT);

	// Main operation of the node
	node.Prepare();

	while(ros::ok())
	{
		ros::spinOnce();
		LoopRate.sleep();
	}

	return (0);

}
