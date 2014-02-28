#include <iostream>
#include "MotorExpert.h"

//==================================================================
//              Costruttore
//==================================================================
MotorExpert::MotorExpert()
{
    SetWheelDistance( 330.0 );
    wait_to_read_compass = 0;
};



//==================================================================
//              Distruttore
//==================================================================
MotorExpert::~MotorExpert()
{
};



//==================================================================
//   Computa i dati da inviare ai motori a partire dalle velocità
//==================================================================
void MotorExpert::actuations(int tanSpeed, int rotSpeed)
{
		printf("[MOTOR]:: Compute Actuation");
    ComputeActuations((float)tanSpeed, (float)rotSpeed);
    printf("[MOTOR]:: Write Actuation");
    WriteActuations();
    //printf("[MOTOR]:: Read Encoders");
    //ReadEncoders();
    //ComputeOdometry();
};



   
void MotorExpert::getOdometryData()
{
  FloatDataListType::iterator it;
  cout<<"Odometry Data"<<endl;
  //ReadEncoders();
  ComputeOdometry();
  for (it = mOdometryData.begin(); it != mOdometryData.end(); it++)
    {
			cout<<"OdometryData: NAME:"<<it->first<<"   VALUE:"<<it->second<<endl;
    } 
};


/*void MotorExpert::AddOdometryData(string name, float value)
{
  cout<<"pippo";
}

void MotorExpert::AddCompassData(string name, int value)
{
  cout<<"pluto";
}
*/
