/*************************************************************
 * This node is the brain of navigation system. Use the mrt
 * for fuzzy decision.
 *
 * by Lorenzo Ripani - AIRLab (Politecnico di Milano)
 * email: ripani.lorenzo@gmail.com
 *
 * version 0.1 - 01/2014
 *************************************************************/

#include "ros/ros.h"
#include <ros/package.h>

#include <brian.h>
#include <getFuzzy.h>
#include <interf_obj.h>

#define NAME_OF_THIS_NODE "e2_brain"

using namespace std;


int main(int argc, char **argv)
{
    //Liste per lo "scambio" dati con brian
	crisp_data_list* cdl;    //lista in ingresso
	command_list * cl;       //lista in uscita

    ROS_INFO("********** Starting Mr. Brian **********");

    string ctofFilename = ros::package::getPath("e2_navigation")+"/e2_brian_config/ctof.txt";
    string shape_ctofFilename = ros::package::getPath("e2_navigation")+"/e2_brian_config/shape_ctof.txt";
    string PredicateFilename = ros::package::getPath("e2_navigation")+"/e2_brian_config/predicate.ini";
    string PredicateActionsFilename = ros::package::getPath("e2_navigation")+"/e2_brian_config/predicateActions.ini";
    string CandoFilename = ros::package::getPath("e2_navigation")+"/e2_brian_config/cando.ini";
    string behaviourFilename = ros::package::getPath("e2_navigation")+"/e2_brian_config/behaviour.txt";
    string wantFilename = ros::package::getPath("e2_navigation")+"/e2_brian_config/want.txt";
    string s_ftocFilename = ros::package::getPath("e2_navigation")+"/e2_brian_config/s_ftoc.txt";
    string s_shapeFilename = ros::package::getPath("e2_navigation")+"/e2_brian_config/s_shape.txt";

    MrBrian *brian = new MrBrian((char*)ctofFilename.c_str(),
    							 (char*)shape_ctofFilename.c_str(),
                                 (char*)PredicateFilename.c_str(),
                                 (char*)PredicateActionsFilename.c_str(),
                                 (char*)CandoFilename.c_str(),
                                 (char*)behaviourFilename.c_str(),
                                 (char*)wantFilename.c_str(),
                                 (char*)s_ftocFilename.c_str(),
                                 (char*)s_shapeFilename.c_str());

    ROS_INFO("********** End inizialization Mr. Brian **********");

    ros::init(argc, argv, NAME_OF_THIS_NODE);

    ros::Time::init();
	ros::Rate r(30);

	while(ros::ok())
    {

		//Get Input data list
        cdl = (brian->getFuzzy())->get_crisp_data_list();
        //Clear input data list
        cdl->clear();

        //==========================================================================
        //						Reading message from INPUT
        //==========================================================================
        //With ROS this step is automatic with messages callbacks

        //==========================================================================
        //						Updating INPUT data
        //==========================================================================
        //Read User_Tracker data
        if(true)
        {
        	cdl->add(new crisp_data("UserDetected", true, 1));
        	//cdl->add(new crisp_data("UserInterested", true, 1));
        	//cdl->add(new crisp_data("UserInterested", 20, 1));
        	cdl->add(new crisp_data("UserDistance", 1700, 1));
        	//cdl->add(new crisp_data("UserXPosition", userXPosition, 1));
        }
        else
        {
        	cdl->add(new crisp_data("UserDetected", false, 1));
        }

        //Read Head_Analyzer data
        //Read Motor Data
        //==========================================================================
        //						Brian algorithm
        //==========================================================================

        ROS_INFO("Brian running...");
        brian->run();
        brian->debug();

        //==========================================================================
        //                      Updating OUTPUT data
        //==========================================================================
        //Read output data
        cl = (brian->getFuzzy())->get_command_singleton_list();
        if (cl!=0)
        {
        	ROS_INFO("Brain saids...");
        	command_list::iterator it;

        	for (it = cl->begin(); it != cl->end(); it++)
        	{
        		string temp = it->first;


        		/*
        		if (temp.compare("TanSpeed") == 0)
        		{
        			//ROS_ERROR("TanSpeed: %d", (int) it->second->get_set_point());
        			int temp = (int)it->second->get_set_point();
        			if(temp >= OUTtanSpeed - MAX_WHEEL_SPEED_GAP && temp <= OUTtanSpeed + MAX_WHEEL_SPEED_GAP)
							{
									OUTtanSpeed = OUTtanSpeed;
									temp = OUTtanSpeed;
									WheelSpeedMessages = false;
							}
							else
        			OUTtanSpeed = temp;
        			WheelSpeedMessages = true;
        		}

        		if (temp.compare("RotSpeed") == 0)
        		{
        			//ROS_ERROR("TanSpeed: %d", (int) it->second->get_set_point());
        			OUTrotSpeed = (int)it->second->get_set_point();
        		}
        		*/
        	}
        }

        //Clear output data
        cl->clear();

        //==========================================================================
        //						Compiling message to Motor and other modules
        //==========================================================================
        //compileMessages();
        //if(WheelSpeedMessages)
        //pubWheelData.publish(wheelDataMSG);

        ros::spinOnce();
        r.sleep();

    }
}






