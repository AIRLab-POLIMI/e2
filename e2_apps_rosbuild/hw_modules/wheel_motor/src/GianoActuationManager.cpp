#include "GianoActuationManager.h"
#include "MotorConstants.h"
#include <stdio.h>
#include <math.h>

#define K_FACTOR 0.75

GianoActuationManager::GianoActuationManager()
{}

GianoActuationManager::~GianoActuationManager()
{}


void GianoActuationManager::ComputeActuations(float tanSpeed, float rotSpeed)
{
  mActuationValues.clear();
  float tan_speed = tanSpeed;
  float rot_speed = rotSpeed * K_FACTOR;
  
  float dx = (tan_speed - rot_speed);
  float sx = (tan_speed + rot_speed);  

  float max_speed = max(fabs(dx),fabs(sx));
  //printf("### - MAX/Sx/Dx:%f-%f-%f\n", max_speed,sx,dx);
  if (max_speed > 100.0)
  {
    dx = dx*100.0/max_speed;
    sx = sx*100.0/max_speed;
  }

  static float spSX=0,spDX=0,actSX=0,actDX=0; 

  //ATTENZIONE !!! INVERTITI X MOTORE MONTATO AL CONTRARIO
  spDX =  (-sx); 
  spSX =  (-dx); 

  float delta;
  delta = spDX - actDX;
  actDX += delta/3;
  
  delta = spSX - actSX;
  actSX += delta/3;
 
  printf("\n%$# - SX/DX (RealValue) :%d-%d",  (int)actSX, (int)actDX);
  mActuationValues[SPEED_MOTOR_DX] = (int) actDX;
  mActuationValues[SPEED_MOTOR_SX] = (int) actSX;
    
  //ATTENZIONE !!! INVERTITI X MOTORE MONTATO AL CONTRARIO
  mActuationValues[SPEED_MOTOR_DX] = (int)(-sx);
  mActuationValues[SPEED_MOTOR_SX] = (int)(-dx);

  //printf("\n%$# - MAX/Sx/Dx:%f-%f-%f\n", max_speed,sx,dx);
  //printf("\n%$# - T/R:%f-%f\n",  mCommandData[TAN_SPEED] , mCommandData[ROT_SPEED] );
  //printf("\n%$# - S/D:%d-%d\n",  mActuationValues[SPEED_MOTOR_SX] , mActuationValues[SPEED_MOTOR_DX] );
 
  if (mCommandData.find(KICK) != mCommandData.end())
    {
      mActuationValues[KICK] = (int)mCommandData[KICK];
    }
	
}

void GianoActuationManager::ComputeOdometry()
{
  float mm_sx = mEncoderValues[ENCODER_MOTOR_SX] * mCoefficientTicTomm_Sx;
  float mm_dx = mEncoderValues[ENCODER_MOTOR_DX] * mCoefficientTicTomm_Dx;
  float movement = ( mm_sx + mm_dx ) / 2.0;
  printf("movement: left = %f, right = %f, used: %f [mm]\n", mm_sx, mm_dx, movement );

  float rotation = ( mm_sx - mm_dx ) / mWheelDistance;
  printf("rotation = %f radians, with wheeldistance = %f mm\n", rotation, mWheelDistance );

  if( movement > 0 )
  {
     mOdometryData[ROBOT_MOVEMENT] = movement; // in mm
  }
  else
  { 
     mOdometryData[ROBOT_MOVEMENT] = 0.0;  // in mm
  }
  mOdometryData[ROBOT_DIRECTION] = rotation / 2; // radiants 
  mOdometryData[ROBOT_ORIENTATION] = rotation * (180/M_PI);   // degrees
  
  mOdometryData[ACT_TAN_SPEED] = (float)(mEncoderValues[SPEED_MOTOR_SX] + mEncoderValues[SPEED_MOTOR_DX]) / 2;
  mOdometryData[ACT_ROT_SPEED] = (float)(mEncoderValues[SPEED_MOTOR_SX] - mEncoderValues[SPEED_MOTOR_DX]) / 2;
  printf("speed: tan = %f, rot = %f\n", mOdometryData[ACT_TAN_SPEED], mOdometryData[ACT_ROT_SPEED] );
}

void GianoActuationManager::SetWheelDistance( float wheeldistance )
{
   mWheelDistance = wheeldistance;
}

void GianoActuationManager::SetConversionCoefficients( float coefficient_sx, float coefficient_dx )
{
   mCoefficientTicTomm_Sx = coefficient_sx;
   mCoefficientTicTomm_Dx = coefficient_dx;
}

