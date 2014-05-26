#ifndef SONAR_DATA_H_
#define SONAR_DATA_H_

#include <strings.h>  
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>  
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <exception>
#include <vector>
#include <string>
#include "WheelChairUtil.h"


class SonarData{
	std::string name;
	float semi_angle;
	float max_err_cart_conv;
	float min_meas;
	float max_meas;
	
	float meas_dist;
	
	public:
	SonarData(std::string name,float semi_angle,float max_err_cart_conv,
		float min_meas,float max_meas);
	~SonarData();
	void setMeasure(float m);
	
	inline std::string getName(){return name;};
	inline float getSemiAngle(){return semi_angle;};
	inline float getMeas(){return meas_dist;};
};

#endif
