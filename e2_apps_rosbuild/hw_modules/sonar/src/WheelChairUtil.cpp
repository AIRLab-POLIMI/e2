#include "WheelChairUtil.h"
#include <stdio.h>
#include <math.h>

using namespace std;

#define M2PI 6.28318530717959


bool stringToFloat(float& t, string s){
	std::istringstream iss(s);
	
	return !(iss >> std::dec >> t).fail();
}

double rad2deg(double d){
	return d*180/M_PI;
}


double deg2rad(double d){
	return d*M_PI/180;
}


void printAndDelete(char *s){
	printf("%s",s);
	delete [] s;
}

double simplifyAngleRad(double d){
	int x;
	//double M2PI=2*M_PI;
	x=(int)(d/M2PI);
	d=d-x*M2PI;
	if(d>M_PI)d-=M2PI;
	else if(d<-M_PI)d+=M2PI;
	return d;
}

double simplifyAngleDeg(double d){
	int x;
	x=(int)(d/360);
	d=d-x*360;
	if(d>180)d-=360;
	else if(d<-180)d+=360;
	return d;
}

double unwrapAngleRad(double source,double d){
	int x=(int)(source/M2PI);
	d=simplifyAngleRad(d);
	d=d+x*M2PI;
	if(d>source){
		if(fabsl(d-source)>M_PI){
			d=d-M2PI;
		}
	}
	else{
		if(fabsl(d-source)>M_PI){
			d=d+M2PI;
		}
	}
	return d;
}

double unwrapAngleDeg(double source,double d){
	int x=(int)(source/360);
	d=simplifyAngleDeg(d);
	d=d+x*360;
	if(d>source){
		if(fabsl(d-source)>180){
			d=d-360;
		}
	}
	else{
		if(fabsl(d-source)>180){
			d=d+360;
		}
	}
	return d;
}


void tokenize(const std::string& str,
                      std::vector<std::string>& tokens,
                      const std::string& delimiters ){
    // Skip delimiters at beginning.
    std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
    // Find first "non-delimiter".
    std::string::size_type pos     = str.find_first_of(delimiters, lastPos);

    while (std::string::npos != pos || std::string::npos != lastPos)
    {
        // Found a token, add it to the vector.
        tokens.push_back(str.substr(lastPos, pos - lastPos));
        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiters, pos);
        // Find next "non-delimiter"
        pos = str.find_first_of(delimiters, lastPos);
    }
}
