/*
 * e2_struct.h
 *
 *  Created on: 28/gen/2014
 *  Author:
 */

#ifndef E2_STRUCT_H_
#define E2_STRUCT_H_

//--------------------------------------------------------------------------------------------
//		Structure for yaml file reading (Stand map)
//--------------------------------------------------------------------------------------------
struct Vec3
{
   float x, y, z;
};

struct Marker
{
   int id;
   std::string name;
   Vec3 position;
};

void operator >> (const YAML::Node& node, Vec3& v) {
   node[0] >> v.x;
   node[1] >> v.y;
   node[2] >> v.z;
}

void operator >> (const YAML::Node& node, Marker& marker) {
   node["id"] >> marker.id;
   node["name"] >> marker.name;
   node["position"] >> marker.position;
}


#endif /* E2_STRUCT_H_ */
