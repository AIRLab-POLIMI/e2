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

struct Speech
{
	string id;
	string text;
	int neck_action;
	int face_action;

};

void operator >> (const YAML::Node& node, Speech& speech);
