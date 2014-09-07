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

namespace YAML {
template<>
struct convert<Speech> {
  static Node encode(const Speech& rhs) {
    Node node;
    node.push_back(rhs.id);
    node.push_back(rhs.text);
    node.push_back(rhs.neck_action);
    node.push_back(rhs.face_action);
    return node;
  }

  static bool decode(const Node& node, Speech& rhs) {

    rhs.id = node["id"].as<std::string>();
    rhs.text = node["text"].as<std::string>();
    rhs.neck_action = node["neck_action"].as<int>();
    rhs.face_action = node["face_action"].as<int>();
    return true;
  }
};
}
