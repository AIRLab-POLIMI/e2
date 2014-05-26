/*======================================================================
* Authors:	cristianmandelli@gmail.com 
* 	      	deborahzamponi@gmail.com
* Data: 15/10/2011
* Description: get data from sonar belt and publish data on sonar topic
======================================================================*/
#include <stdio.h>
#include <string.h>
//ROS
#include "ros/ros.h"
#include "ros/package.h"
//Messages
#include <sonar/SonarData.h>
//Other libs
#include "SerialCommunicationSonar.h"
#include "CharCircularBuffer.h"
#include "ReadSonar.h"
#include "SonarData.h"

using namespace std;

//Global Scope
//======================================================================
//======================================================================
ReadSonar* readSonar;
//Messages
ros::Publisher pubSonarData;	
sonar::SonarData sonarData;

//Prototypes
//======================================================================
//======================================================================


//Main
//======================================================================
//======================================================================
int main(int argc, char **argv)
{
	//Ros initialization
	ros::init(argc, argv, "sonar");
	ros::NodeHandle nh;
	
	//Message publisher
	pubSonarData = nh.advertise<sonar::SonarData>("sonarData", 100);
	
	//Variables
	readSonar = 0;
	float	to_meter = 0.1;
	int max_line_err= 5;

	
	//Open sonar connection on device port
	string device = string("/dev/ttyUSB0");
	try
	{
		ROS_INFO("[SONAR]::Opening communication...");
		readSonar = new ReadSonar(device,to_meter);
		readSonar->sendRun();
	}
	catch (ReadSonarDeviceException &e)
	{
		ROS_ERROR("[SONAR]::Error opening connection to the device");
		exit(-1);
	}
	
	//RosLoop
	ros::Rate r(5);
	while(ros::ok())
	{
		
		int meas_progress=0;
		int line_readed=0;
		unsigned int n_line;
		//Read sonar belt data	
		//VERIFICARE SE IL DO-WHILE E'  EFFETTIVAMENTE NECESSARIO O SE 
		//PUO' ESSERE USATO IL LOOP ROS PER LO STESSO SCOPO... COSI'
		//SEMPBRA CHE LE LETTURE AVVEGANO ALL'INTERNO DI UN UNICO CICLO
		//ROS E NON IN DIVERSI E QUINDI OGNI 1/5 DI SECONDO.
		
		do
		{
			if(readSonar->readData()==0)
			{
				n_line=readSonar->getLineToParseNum();
				line_readed+=n_line;
				for(unsigned int i=0;i<n_line;i++)
				{

					//ROS_INFO("[SONAR]:: Parse %d/%d lines",i,n_line);
				
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
						
						for(int i = 0; i <8; i++)
						{
							ROS_INFO("Sonar %d value: %d",i, (int)readSonar->getMeasure(i));
						}
						
						//Compile and send message
						sonarData.sonarA.data = (int)readSonar->getMeasure(0);
						sonarData.sonarB.data = (int)readSonar->getMeasure(1);
						sonarData.sonarC.data = (int)readSonar->getMeasure(2);
						sonarData.sonarD.data = (int)readSonar->getMeasure(3);
						sonarData.sonarE.data = (int)readSonar->getMeasure(4);
						sonarData.sonarF.data = (int)readSonar->getMeasure(5);
						sonarData.sonarG.data = (int)readSonar->getMeasure(6);
						sonarData.sonarH.data = (int)readSonar->getMeasure(7);
						
						//Publish current message
						pubSonarData.publish(sonarData);
					}
				}
			}
			else
			{
				ROS_ERROR("[SONAR]::Sonar Error");
				exit(-1);
			}
		} while(line_readed<=max_line_err);
		
		ros::spinOnce();
		r.sleep();
	}
}



//Functions
//======================================================================
//======================================================================

