#include "SerialCommunicationAIRBoard.h"
#include "MotorConstants.h"

#include <assert.h>
#include <sstream>
#include <cstring>


#include <stdio.h>


using namespace std;

const int WRITING_DELAY = 0;

SerialCommunicationAIRBoard::SerialCommunicationAIRBoard()
{
#ifdef rabbiati
  mBaudRate = B115200;
#else
  mBaudRate = B115200;
#endif
  mModemDevice = strdup("/dev/ttyS0");
  mHardwareFlowControl = false;
  InitCommunication();
  StartMotors();
  
  last_speed_motor_0 = 200;
  last_speed_motor_1 = 200;
}

SerialCommunicationAIRBoard::~SerialCommunicationAIRBoard()
{
  StopMotors();
  CloseCommunication();
}

void SerialCommunicationAIRBoard::StartMotors()
{
  SetMotorsSpeed(0,0);
  usleep(WRITING_DELAY);
  Write("a2\r");
}

void SerialCommunicationAIRBoard::StopMotors()
{
  SetMotorsSpeed(0,0);
  usleep(WRITING_DELAY);
  Write("b2\r");
}

void SerialCommunicationAIRBoard::WriteActuations()
{
  Write("a2\r");
  SetMotorsSpeed(mActuationValues[SPEED_MOTOR_SX],mActuationValues[SPEED_MOTOR_DX]);

  if (mActuationValues.find(KICK) != mActuationValues.end())
    {
      usleep(WRITING_DELAY);
      Kick(mActuationValues[KICK]);
    }

}

void SerialCommunicationAIRBoard::ReadEncoders()
{
  int n;


  usleep(WRITING_DELAY);
  Write(READ_SPEEDS);
  ReadNumber(&n);
  mEncoderValues[SPEED_MOTOR_SX] = n;
  //printf( "\nlettura: mEncoderValues[SPEED_MOTOR_SX] = %d", mEncoderValues[SPEED_MOTOR_SX] );
  ReadNumber(&n);
  mEncoderValues[SPEED_MOTOR_DX] = n;
  //printf( "\nlettura: mEncoderValues[SPEED_MOTOR_DX] = %d", mEncoderValues[SPEED_MOTOR_DX] );

  usleep(WRITING_DELAY);
  Write(READ_ENCODERS);
  ReadNumber(&n);
  mEncoderValues[ENCODER_MOTOR_SX] = n;
  //printf( "\nlettura: mEncoderValues[ENCODER_MOTOR_SX] = %d", mEncoderValues[ENCODER_MOTOR_SX] );
  ReadNumber(&n);
  mEncoderValues[ENCODER_MOTOR_DX] = n;
  //printf( "\nlettura: mEncoderValues[ENCODER_MOTOR_DX] = %d", mEncoderValues[ENCODER_MOTOR_DX] );
}

void SerialCommunicationAIRBoard::SetMotorsSpeed(int speed_motor_0, int speed_motor_1)
{
  speed_motor_1 = -speed_motor_1;
  stringstream serial_message;
  string m0("m2");
  string m1("");
  int abs_speed_motor_0 = 0;
  int abs_speed_motor_1 = 0;
  
  if (speed_motor_0 >= 0)
    {
      m0.append("+");
    }
  else
    {
      m0.append("-");
    }

  if (speed_motor_1 >= 0)
    {
      m1.append("+");
    }
  else
    {
      m1.append("-");
    }



  abs_speed_motor_0 = abs(speed_motor_0);
  abs_speed_motor_1 = abs(speed_motor_1);

  if (abs_speed_motor_0 > 100)
    {
      abs_speed_motor_0 = 100;
    }
  else if (abs_speed_motor_0 < 10)
    {
      m0.append("00");
    }
  else if (abs_speed_motor_0 < 100)
    {
      m0.append("0");
    }

  if (abs_speed_motor_1 > 100)
    {
      abs_speed_motor_1 = 100;
    }
  else if (abs_speed_motor_1 < 10)
    {
      m1.append("00");
    }
  else if (abs_speed_motor_1 < 100)
    {
      m1.append("0");
    }

  serial_message.str(string());

  serial_message << m0 << abs_speed_motor_0 
                 << m1 << abs_speed_motor_1 
		 << "\r"; 
		 
  printf("### - %s\n",serial_message.str().data());
  
  if(( last_speed_motor_0 != speed_motor_0) ||
	(last_speed_motor_1 != speed_motor_1))

	Write(serial_message.str());
  
  last_speed_motor_0 = speed_motor_0;
  last_speed_motor_1 = speed_motor_1;  
  
}


void SerialCommunicationAIRBoard::Kick(int kicker)
{
  if (kicker == 0)
    {
      Write("k2\r");
    }
  else if (kicker == 1)
    {
      Write("k0\r");
    }
  else if (kicker == -1)
    {
      Write("k1\r");
    }

}
