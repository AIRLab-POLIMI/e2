/*************************************************************
 * This node locate the robot and update it's pose respect
 * positions markers.
 *
 * by Lorenzo Ripani - AIRLab (Politecnico di Milano)
 * email: ripani.lorenzo@gmail.com
 *
 * version 0.1 - 01/2014
 *************************************************************/


#include <ros/ros.h>
#include "ros/package.h"
#include "localization_lib.cpp"

// Messages include
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"

#include <tf/transform_listener.h>

#include "yaml.h"
#include <yaml-cpp/yaml.h>


using namespace std;

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

class Localization
{

	public:
	string marker_file;
	Marker *map_marker;

	Localization(std::string name): ll(),nh_()
	{
		alvar_sub_ = nh_.subscribe("/ar_pose_marker", 1,&Localization::executeCB, this);
		marker_file = ros::package::getPath("e2_localization")+"/config/marker_positions.yaml";
	}


	// Destructor
	~Localization(void)
	{

	}

	void executeCB(const ar_track_alvar::AlvarMarkers &msg)
	{
		ll.getCameraMarker(msg);
		ll.showInfo();

	}


protected:
	LocalizationLib ll;
	ros::NodeHandle nh_;
	ros::Subscriber alvar_sub_;

};


//--------------------------------------------------------------------------------------------
//		Main code
//--------------------------------------------------------------------------------------------
int main(int argc, char** argv) {
	ros::init(argc, argv, "localization");
	Localization localization(ros::this_node::getName());
	ros::spin();
	return 0;
}

