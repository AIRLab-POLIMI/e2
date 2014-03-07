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

struct Marker
{
   int id;
   string name;
   Vec3 position;
};

struct Speech
{
	string id;
	string text;
	int duration;
};


void operator >> (const YAML::Node& node, Vec3& v);

void operator >> (const YAML::Node& node, Marker& marker);

void operator >> (const YAML::Node& node, Speech& speech);
