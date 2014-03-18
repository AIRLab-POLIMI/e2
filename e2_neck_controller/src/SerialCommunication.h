#ifndef SerialCommunication_h
#define SerialCommunication_h

#include <unistd.h>
#include <termios.h>
#include <string>
#include <sstream>
#include <stdio.h>

//#define MOTOR_DEBUG 1
//#define POLOLU_DEBUG

#include "LowLevelCommunication.h"

#ifdef TIME
extern int getTime();
#endif
//#define MOTOR_DEBUG 1

using namespace std;

/**
 * This class is the interface for the serial communication towards the low-level 
 * controller.
 *
 * The SerialCommunication class is an abstract class that implements the 
 * LowLevelCommunication interface for the serial communication device, and it
 * gives the methods to write and read data of some types on the serial channel.
 *
 * @short	Interface for the serial communication
 * @author	Marcello Restelli (restelli@elet.polimi.it)
 */

class SerialCommunication : public LowLevelCommunication
{
 public:
  // LIFECYCLE

  /**
   * Default constructor
   */
  SerialCommunication();

  /**
   * Destructor
   */
  virtual ~SerialCommunication();

  // OPERATORS

  //Use compiler-generated copy constructor and assignment
  //SerialCommunication(const SerialCommunication& from);
  //SerialCommunication& operator=(const SerialCommunication& from);

  // OPERATIONS
  
  /**
   * This method initializes the serial communication channel.
   */
  void InitCommunication();
  
  /**
   * This method closes the communication channel.
   */
  void CloseCommunication();

  //
  //  void ReadEncoders(){};

  /**
   * This method writes a character on the serial channel
   *
   * @param c	The character that must be written on the serial channel
   */
  void Write(char c);

  /**
   * This method writes an integer on the serial channel
   *
   * @param n	The number that must be written on the serial channel
   */
  void Write(int n);

  /**
   * This method writes a string on the serial channel
   *
   * @param s	The string that must be written on the serial channel
   */
  void Write(string s);

  /**
   * This method reads an integer from the serial channel
   *
   * @param n	A pointer to the integer that has been read from the serial
   */
  void Read(int* n);

  /**
   * This method reads a 24-bit (3-byte) integer from the serial channel 
   *
   * @param n	A pointer to the integer that has been read from the serial
   */
  void Read24bitNumber(int* n);


  /**
   * This method reads a string representing an integer from the serial channel
   *
   * @param n	A pointer to the integer that has been read from the serial
   */
  void ReadNumber(int* n);

  /**
   * This method reads a string representing a float from the serial channel
   *
   * @param n	A pointer to the float that has been read from the serial
   */
  void ReadNumber(float* n);

  /**
   * This method reads a char from the serial channel
   *
   * @param c	A pointer to the char that has been read from the serial
   */
  void Read(char* c);

 protected:

  /**
   * The descriptor of the serial device
   */
  int fd; 

  /**
   * The transmission rate of the serial device
   */
  unsigned int mBaudRate;

  /**
   * A string that identifies the serial device
   */
  char* mModemDevice;

  /**
   * True if the control device need the output hardware flow control
   */
  bool mHardwareFlowControl;

 private:
  struct termios oldtio;
  struct termios newtio;
};


inline void SerialCommunication::Write(char c)
{
  write(fd,&c,1);
}


inline void SerialCommunication::Write(string s)
{
#ifdef MOTOR_DEBUG
  printf("\nWriting: %s\n",s.data());
#endif
#ifdef TIME
  int time = getTime();
#endif
  write(fd,s.c_str(),s.size());
#ifdef TIME
  time = getTime() - time;
  printf("\nWriting Time: %d\n",time); 
#endif
}


inline void SerialCommunication::Write(int n)
{
  write(fd,&n,sizeof(int));
}

inline void SerialCommunication::Read(int* n)
{
  read(fd,n,sizeof(int));
}

inline void SerialCommunication::Read24bitNumber(int* n)
{
  //:TODO:
  char bytes[3];
  for (int i = 0; i < 3; i++)
    {
      Read(&bytes[i]);
    }
  *n = (((int )bytes[0] << 16)) 
    | (((unsigned int) bytes[1]) << 8) 
    | ((unsigned int) bytes[2]);
#ifdef MOTOR_DEBUG
  printf("\nReading 24 bit number: %d\n",n);
#endif
}

inline void SerialCommunication::ReadNumber(int* n)
{
  stringstream received_message;

  char c = '\0';
  int flag=0;
  int number;

  while ( c != '\n' && c != ' ' && c != '\t' && !flag)
    {
      Read(&c);
#ifdef MOTOR_DEBUG
      printf("\nReading: %c\n",c);
#endif     
      if( (c >= '0' && c <= '9') || c == '-' || c == '+' )
	received_message << c;
      else if( c == '\n' || c == ' ' || c == '\t' )
      { }
      else
	flag=1;       
    }
  
  received_message >> number;
  if(flag || abs(number)>=5000)
    {  
      *n=0;
#ifdef MOTOR_DEBUG
      printf("\nGot: %d\n",*n);
#endif      
    }
  else
    {  
      *n = number;
#ifdef MOTOR_DEBUG
      printf("\nGot: %d\n",*n);
#endif      
    }
}


inline void SerialCommunication::ReadNumber(float* n)
{
  stringstream received_message;

  char c = '\0';

  while (c != '\n')
    {
#ifdef TIME
      int time = getTime();
#endif
      Read(&c);
#ifdef TIME
      time = getTime() - time;
      printf("\nReading Time: %d\n",time);
#endif
      if ((int)c != 6)
        {
#ifdef TIME
	  int time = getTime();
#endif
          received_message << c;
#ifdef TIME
	  time = getTime() - time;
	  printf("\nReading Time to Buffer: %d\n",time);
#endif
	}
    }
#ifdef TIME
  int time = getTime();
#endif
  received_message >> *n;
#ifdef TIME
  time = getTime() - time;
  printf("\nWriting to float: %d\n",time);
#endif
  
#ifdef MOTOR_DEBUG
  printf("\nReading string: %s\n",received_message.str().data());    
  printf("\nReading: %f\n",*n);
#endif      
}

inline void SerialCommunication::Read(char* c)
{
  read(fd,c,1);

#ifdef MOTOR_DEBUG
  printf("\nReading: %c\n",*c);
#endif   
}

#endif
