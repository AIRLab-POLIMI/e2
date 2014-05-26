#include "ReadSonar.h"
#include "sstream"
#include <stdio.h>
#include <sstream>

//#define DEBUG
#define BAUDRATE 		B19200
#define TOUT			3000 //msec
#define MAX_C_RECV 		40
#define CHAR_PAUSE 		10000 //usec
#define MAX_BUF_CHAR	32767
	
using namespace std;


ReadSonar::ReadSonar(std::string sdev,float to_meter) throw (ReadSonarDeviceException)
:ReadSonarBase(to_meter){
	
	fd = open(sdev.c_str(), O_RDWR | O_NOCTTY );
	if(fd>=0){
		
		
		tcgetattr(fd,&oldtio);
		bzero(&newtio, sizeof(termios));
		newtio.c_cflag = BAUDRATE  | CS8 | CLOCAL | CREAD | CLOCAL ;
		newtio.c_iflag = IGNPAR;
		newtio.c_oflag = 0;
		newtio.c_lflag &= ~ICANON;
			 
		set_fd(fd);
		/*if(set_low_latency()!=0){
			fprintf(stderr,"******* readodo, low_latency error\n");
		}*/
		
		newtio.c_cc[VTIME]    = 0;
		newtio.c_cc[VMIN]     = 1;   
		tcflush(fd, TCIFLUSH);
		tcsetattr(fd,TCSANOW,&newtio);
/*		fds[0].fd=fd;
		fds[0].events=POLLIN;*/
		tcflush(fd, TCIOFLUSH);
		buffer=new CharCircularBuffer(MAX_BUF_CHAR,'\r');
				
	}
	else{
		throw ReadSonarDeviceException();
	}	
}

ReadSonar::~ReadSonar(){
	if(fd>=0)close(fd);
	if(buffer)delete buffer;	
}

bool ReadSonar::isReady(){
	return (fd>=0);
}

int ReadSonar::readData(){
	if(fd<0)return -1;

	int res;
	unsigned int count_c=0;
	
	#ifdef DEBUG 
	printf("\nSonar::Ciclo read\n");
	#endif
	res=0;
	do{
		switch(waitData(TOUT)){
			case SerialCommunicationSonar::wait_ok:
				res = read(fd,tmp_buf,max_buf_tmp);
				if(res<=0){
					fprintf(stderr,"(1) Sonar READ Error on fileno %d\n",fd);
					res=-1;
				}
				if(buffer->addNChar(tmp_buf,res)!=res){
					count_c	= count_c+res;			
					fprintf(stderr,"(1) Sonar BUF_FULL on fileno %d\n",fd);
					res=-1;
				}
				break;
			case SerialCommunicationSonar::wait_err: 
				fprintf(stderr,"(1) Sonar Error on fileno %d\n",fd);
				res=-1;
				break;
			case SerialCommunicationSonar::wait_tout: 
				fprintf(stderr,"(1) Sonar TOUT on fileno %d\n",fd);
				res=-1;
				break;
		}		
		if(res==-1)break;
		
	}while(count_c<MAX_C_RECV && buffer->getLineCount()<=0);
	
	if(res<=0)return -1;
	return 0;
}

int ReadSonar::sendRun(){
	tcflush(fd, TCIFLUSH);
	return sendStringCommand((char *)"run()\r",6);
}

int ReadSonar::sendStop(){
	return sendStringCommand((char *)"stop()\r",7);
}

int ReadSonar::sendStringCommand(char *cmd,int len){
	if(fd<0)return -1;
	for(int i=0;i<len;i++){
		if(write(fd,&cmd[i],1)!=1)return -1;
		usleep(CHAR_PAUSE);
	}
	return 0;
}
