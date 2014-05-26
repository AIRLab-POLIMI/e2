#ifndef READOSONAR_H_
#define READOSONAR_H_

#include <strings.h>  
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>  
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <sys/time.h>
#include <math.h>
#include <poll.h>
#include <exception>
#include <vector>
#include <string>
#include "WheelChairUtil.h"
#include "SerialCommunicationSonar.h"
#include "CharCircularBuffer.h"
#include "ReadSonarBase.h"

class ReadSonar : public SerialCommunicationSonar, public ReadSonarBase{
	int fd;  	//descrittore del file per leggere/scrivere sulla seriale
	termios oldtio,newtio;
	
	int sendStringCommand(char *cmd,int len);
public:
	ReadSonar(std::string sdev,float to_meter)
		throw (ReadSonarDeviceException);
	~ReadSonar();

	virtual bool isReady(); //to implement 
	
	virtual int readData(); //to implement

	virtual int sendRun();//to implement
	virtual int sendStop();//to implement
};

#endif
