#include <stdio.h>
#include <string.h>

//Serial Communication libraries
#include "SerialCommunicationSonar.h"
#include "CharCircularBuffer.h"
#include "ReadSonar.h"
#include "SonarData.h"

//Ros Libraries
#include "ros/ros.h"
#include "ros/package.h"
#include "sensor_msgs/Range.h"

//Ros Messages

using namespace std;

//Variables
//==============================================================================
//==============================================================================
ReadSonar* readSonar;

//Control Variables


//Protoypes
//==============================================================================
//==============================================================================


//Functions
//==============================================================================
//==============================================================================
int main(int argc, char **argv)
{
	//Initialization
	readSonar = 0;
	float	to_meter = 0.1;
	int max_line_err=5;
	
	//Ros node initialization
	ros::init(argc, argv, "sonar");
  ros::NodeHandle nh;
	
	//Read configuration
	
	string device = string("/dev/ttyUSB0");
	
	ros::Publisher sonar_pub_ = nh.advertise<sensor_msgs::Range>("e2_sonar", 1000);

	//Open Sonar
	try
	{
		ROS_INFO("[SONAR]::Create communication object");
		readSonar = new ReadSonar(device,to_meter);
		readSonar->sendRun();
	}
	catch (ReadSonarDeviceException &e)
	{
		ROS_ERROR("[SONAR]::Error open device");
		exit(-1);
	}

  ros::Rate r(200);
	while(ros::ok())	//ROS LOOP
  {
		if(!readSonar)
		{
			#ifdef EXPERTS_DEBUG
			ROS_ERROR("[SONAR]::ReadSonar NULL");
			#endif
			exit(-1);
		}
		
		int meas_progress=0;
		int line_readed=0;
		do
		{
			if(readSonar->readData()==0)
			{
				unsigned int n_line;
				n_line=readSonar->getLineToParseNum();
				line_readed+=n_line;
				for(unsigned int i=0;i<n_line;i++)
				{
					#ifdef EXPERTS_DEBUG
					ROS_INFO("[SONAR]:: Parse %d/%d lines",i,n_line);
					#endif
				
					switch(readSonar->parseLine())
					{
						case ReadSonar::parse_err:
							meas_progress=0;
							break;
						case ReadSonar::parse_meas_b1:
							if(meas_progress==0)meas_progress++;
							break;
						case ReadSonar::parse_meas_b2:
							if(meas_progress==1)meas_progress++;
							break;
						case ReadSonar::parse_dbg:
							meas_progress=0;
							break;
						case ReadSonar::parse_ok:
							meas_progress=0;
							break;
						case ReadSonar::parse_response_err:
							meas_progress=0;
							break;
						default:
							meas_progress=0;
							break;
					}
				
					if(meas_progress==2)
					{
						meas_progress=0;
						
						for (unsigned int i=0;i<8;i++)
						{
							//Get Data
							float sonarRawData = readSonar->getMeasure(i);
							
							ROS_INFO("[SONAR]::SonarData:%d  -  %f", i, sonarRawData);

							std::ostringstream stream;
							stream << "sonar_" << i;

							sensor_msgs::Range msg;
							msg.header.frame_id = stream.str();
							msg.field_of_view= 0.1; // x-axis only
							msg.min_range = 0;
							msg.max_range= 6;
							msg.range = sonarRawData/100;

							sonar_pub_.publish(msg);

						}					
					}
				}
			}
			
			else
			{
				#ifdef EXPERTS_DEBUG
				ROS_ERROR("[SONAR]::Sonar Read Error");
				#endif
				
				exit(-1);
			}
	
		} while(line_readed<=max_line_err);
		
		ros::spinOnce();
		r.sleep();
	}    
}