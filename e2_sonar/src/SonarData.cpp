#include "SonarData.h"

SonarData::SonarData(std::string name,float semi_angle,float max_err_cart_conv,
		float min_meas,float max_meas){
	
	this->name=name;
	this->semi_angle=semi_angle;
	this->max_err_cart_conv=max_err_cart_conv;
	
	this->min_meas=min_meas;
	this->max_meas=max_meas;
	
	this->meas_dist=-1;
}

SonarData::~SonarData(){
}

void SonarData::setMeasure(float m){
	meas_dist=m;
}
