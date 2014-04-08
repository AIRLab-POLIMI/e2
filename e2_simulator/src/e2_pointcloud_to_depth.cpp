/*
 * e2_pointcloud_to_depth.cpp
 * 
 * convert pointcloud messages to depth image. Used for simulation purpose
 *
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 08/apr/2014
 *
 */

#define ROS_NODE_RATE	50
#define ROS_NODE_NAME	"pointcloud_to_depth"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <pcl_ros/io/pcd_io.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <string>

using namespace sensor_msgs;

class PointCloudToImage
{

public:
	void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
	{
		if ((cloud->width * cloud->height) == 0)
		  return; //return if the cloud is not dense!
		try
		{
			pcl::toROSMsg (*cloud, image_); //convert the cloud

		}
		catch (std::runtime_error e)
		{
			ROS_ERROR_STREAM("Error in converting cloud to image message: " << e.what());
		}
		image_pub_.publish (image_); //publish our cloud image
	}

	PointCloudToImage () : cloud_topic_("/camera/points"),image_topic_("/camera/image_raw")
	{
		sub_ = nh_.subscribe (cloud_topic_, 30, &PointCloudToImage::cloud_cb, this);
		image_pub_ = nh_.advertise<sensor_msgs::Image> (image_topic_, 30);

		//print some info about the node
		std::string r_ct = nh_.resolveName (cloud_topic_);
		std::string r_it = nh_.resolveName (image_topic_);
		ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct );
		ROS_INFO_STREAM("Publishing image on topic " << r_it );
	}

private:
	  ros::NodeHandle nh_;
	  sensor_msgs::Image image_; //cache the image message
	  std::string cloud_topic_; //default input
	  std::string image_topic_; //default output
	  ros::Subscriber sub_; //cloud subscriber
	  ros::Publisher image_pub_; //image message publisher
};

int main (int argc, char **argv)
{
	  ros::init (argc, argv, "convert_pointcloud_to_image");
	  PointCloudToImage pci; //this loads up the node

	  ROS_INFO("["ROS_NODE_NAME"]:: Node started");
	  ros::Rate r(ROS_NODE_RATE);

	  while(ros::ok())
	  {
		  ros::spinOnce();
		  r.sleep();
	  }

}
