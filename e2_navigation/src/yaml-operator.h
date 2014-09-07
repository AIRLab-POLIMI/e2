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
	int neck_action;
	int face_action;

};

namespace YAML {
//*************************
// Template Vec2
//*************************
template<>
struct convert<Vec2> {
  static Node encode(const Vec2& rhs) {
    Node node;
    node.push_back(rhs.z);
    node.push_back(rhs.w);
    return node;
  }

  static bool decode(const Node& node, Vec2& rhs) {
    rhs.z = node[0].as<float>();
    rhs.w = node[1].as<float>();
    return true;
  }
};
//*************************
// Template Vec3
//*************************
template<>
struct convert<Vec3> {
  static Node encode(const Vec3& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    return node;
  }

  static bool decode(const Node& node, Vec3& rhs) {
    rhs.x = node[0].as<float>();
    rhs.y = node[1].as<float>();
    rhs.z = node[2].as<float>();
    return true;
  }
};
//*************************
// Template Marker
//*************************
template<>
struct convert<Marker> {
  static Node encode(const Marker& rhs) {
    Node node;
    node.push_back(rhs.id);
    node.push_back(rhs.name);
    node.push_back(rhs.position);
    node.push_back(rhs.orientation);
    return node;
  }

  static bool decode(const Node& node, Marker& rhs) {

    rhs.id = node["id"].as<int>();
    rhs.name = node["name"].as<std::string>();
    rhs.position = node["position"].as<Vec3>();
    rhs.orientation = node["orientation"].as<Vec2>();
    return true;
  }
};
//*************************
// Template Speech
//*************************
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




void operator >> (const YAML::Node& node, Marker& marker);
