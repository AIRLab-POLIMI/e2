/*************************************************************
 * This node locate the robot and update it's pose respect
 * positions markers.
 *
 * by Lorenzo Ripani - AIRLab (Politecnico di Milano)
 * email: ripani.lorenzo@gmail.com
 *
 * version 0.1 - 01/2014
 *************************************************************/
#include <ar_track_alvar/AlvarMarker.h>
#include <ar_track_alvar/AlvarMarkers.h>

using namespace std;

class LocalizationLib
{
	public:

		string mapFrame;
		string robotFrame;
		stringstream real_marker_frame,detected_marker_frame;

		int marker_id;

		geometry_msgs::PoseStamped robot_pose;
		geometry_msgs::PoseStamped real_marker;
		geometry_msgs::PoseStamped camera_marker;

		//Functions:
		void showInfo();

		void getRealMarker();
		void getCameraMarker(const ar_track_alvar::AlvarMarkers &msg);

		void prepareMessage();

		void getCameraTransform(char *frame);
		void calculateRealPosition();


		LocalizationLib()
		{
		}

		~LocalizationLib(void)
		{
		}

};

void LocalizationLib::getCameraMarker(const ar_track_alvar::AlvarMarkers &msg)
{
	ROS_INFO(" * Get Marker Pose from Camera.");


	std::vector<ar_track_alvar::AlvarMarker> markers = msg.markers;

	for(std::vector<ar_track_alvar::AlvarMarker>::iterator it = markers.begin(); it != markers.end(); ++it)
	{

		ROS_DEBUG("Marker Message");
		ROS_DEBUG("Marker ID: %i",it->id);
		ROS_DEBUG("X: %f",it->pose.pose.position.x);
		ROS_DEBUG("Y: %f",it->pose.pose.position.y);

		marker_id = it->id;
		camera_marker = it->pose;

		real_marker_frame.str("");
		detected_marker_frame.str("");

		real_marker_frame << "marker_" << marker_id;
		detected_marker_frame << "ar_marker_" << marker_id ;

	}

}

void LocalizationLib::getCameraTransform(char *maker_frame)
{

}

void LocalizationLib::prepareMessage()
{

}
void LocalizationLib::showInfo()
{
	ROS_INFO("***********************************************************");
	ROS_INFO("***********************************************************");

	ROS_INFO("Camera Marker pos[x,y,z] : %f - %f - %f",camera_marker.pose.position.x,camera_marker.pose.position.y,camera_marker.pose.position.z);
	ROS_INFO("Camera Marker or[z] : %f ",camera_marker.pose.orientation.z);

	ROS_INFO("***********************************************************");
	ROS_INFO("***********************************************************");

}
