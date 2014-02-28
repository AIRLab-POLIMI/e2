#include "SerialCommunication.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>

using namespace std;

// Definizioni del modulo
#define STATUS_OFF 0
#define STATUS_ON 1
#define SX 0
#define DX 1
#define BOTH 2

#define FALSE 0
#define TRUE 1
#define _POSIX_SOURCE 1


SerialCommunication::SerialCommunication()
{}

SerialCommunication::~SerialCommunication()
{
  delete mModemDevice;
}


void SerialCommunication::InitCommunication(void)
{
  // Open modem device for reading and writing and not as controlling tty
  // because we don't want to get killed if linenoise sends CTRL-C.
  if ((fd = open(mModemDevice, O_RDWR | O_NOCTTY )) <0)
    {perror(mModemDevice); exit(-1); }

  tcgetattr(fd,&oldtio); /* save current serial port settings */
  bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */
  
  // BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
  //  CRTSCTS : output hardware flow control (only used if the cable has
  //            all necessary lines. See sect. 7 of Serial-HOWTO)
  //  CS8     : 8n1 (8bit,no parity,1 stopbit)
  //  CLOCAL  : local connection, no modem contol
  //  CREAD   : enable receiving characters
  if (mHardwareFlowControl)
    {
      newtio.c_cflag = mBaudRate | CRTSCTS | CS8 | CLOCAL | CREAD;
    }
  else
    {
      newtio.c_cflag = mBaudRate | CS8 | CLOCAL | CREAD;
    }
  
  //  IGNPAR  : ignore bytes with parity errors
  //  ICRNL   : map CR to NL (otherwise a CR input on the other computer
  //            will not terminate input)
  //  otherwise make device raw (no other input processing)
  newtio.c_iflag = IGNPAR | ICRNL;
  
  // Raw output.
  newtio.c_oflag = 0;
  
  //  ICANON  : enable canonical input
  //  disable all echo functionality, and don't send signals to calling program
  newtio.c_lflag = ICANON;
  
  //  initialize all control characters
  //  default values can be found in /usr/include/termios.h, and are given
  //  in the comments, but we don't need them here
  newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */
  newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
  newtio.c_cc[VERASE]   = 0;     /* del */
  newtio.c_cc[VKILL]    = 0;     /* @ */
  newtio.c_cc[VEOF]     = 4;     /* Ctrl-d */
  newtio.c_cc[VTIME]    = 0;     /* inter-character timer unused */
  newtio.c_cc[VMIN]     = 0;//1;     /* blocking read until 1 character arrives */
  newtio.c_cc[VSWTC]    = 0;     /* '\0' */
  newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */
  newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
  newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
  newtio.c_cc[VEOL]     = 0;     /* '\0' */
  newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
  newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
  newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
  newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
  newtio.c_cc[VEOL2]    = 0;     /* '\0' */

  //  now clean the modem line and activate the settings for the port
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd,TCSANOW,&newtio);
  
  // terminal settings done, now handle input
  printf("Terminal Started");
}


void SerialCommunication::CloseCommunication(void)
{
  // restore the old port settings
  tcsetattr(fd,TCSANOW,&oldtio);
  
  close (fd);
}


