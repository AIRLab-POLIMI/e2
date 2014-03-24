/*
 * yaml-operator.h - AIRLab (Politecnico di Milano)
 *
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 28/feb/2014
 *
 */

#include <yaml-cpp/yaml.h>

using namespace std ;

struct Vec3
{
   float x, y, z;
};

struct Vec2
{
   float z, w;
};

struct Marker
{
   int id;
   string name;
   Vec3 position;
   Vec2 orientation;
};

struct Speech
{
	string id;
	string text;
	int duration;
};


void operator >> (const YAML::Node& node, Vec3& v);	//Position
void operator >> (const YAML::Node& node, Vec2& v); //Orientation

void operator >> (const YAML::Node& node, Marker& marker);

void operator >> (const YAML::Node& node, Speech& speech);
