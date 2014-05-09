#ifndef WHEELCHAIRUTIL_H_
#define WHEELCHAIRUTIL_H_

#include <string>
#include <sstream>
#include <iostream>
#include <vector>

#ifdef DEBUG
	#define DBG(x,...) do{fprintf(stderr,x,##__VA_ARGS__);}while(0)
#else
	#define DBG(x,...)
#endif

bool stringToFloat(float& t, std::string s);

void printAndDelete(char *s);

double rad2deg(double d);
double deg2rad(double d);

double simplifyAngleRad(double d);
double simplifyAngleDeg(double d);

double unwrapAngleRad(double source,double d);
double unwrapAngleDeg(double source,double d);

void tokenize(const std::string& str,
                      std::vector<std::string>& tokens,
                      const std::string& delimiters = " ");



#endif
