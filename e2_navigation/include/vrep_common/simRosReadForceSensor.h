/* Auto-generated by genmsg_cpp for file /home/jackal/ros_workspace/src/vrep/vrep_common/srv/simRosReadForceSensor.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSREADFORCESENSOR_H
#define VREP_COMMON_SERVICE_SIMROSREADFORCESENSOR_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"



#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3.h"

namespace vrep_common
{
template <class ContainerAllocator>
struct simRosReadForceSensorRequest_ {
  typedef simRosReadForceSensorRequest_<ContainerAllocator> Type;

  simRosReadForceSensorRequest_()
  : handle(0)
  {
  }

  simRosReadForceSensorRequest_(const ContainerAllocator& _alloc)
  : handle(0)
  {
  }

  typedef int32_t _handle_type;
  int32_t handle;


  typedef boost::shared_ptr< ::vrep_common::simRosReadForceSensorRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosReadForceSensorRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosReadForceSensorRequest
typedef  ::vrep_common::simRosReadForceSensorRequest_<std::allocator<void> > simRosReadForceSensorRequest;

typedef boost::shared_ptr< ::vrep_common::simRosReadForceSensorRequest> simRosReadForceSensorRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosReadForceSensorRequest const> simRosReadForceSensorRequestConstPtr;


template <class ContainerAllocator>
struct simRosReadForceSensorResponse_ {
  typedef simRosReadForceSensorResponse_<ContainerAllocator> Type;

  simRosReadForceSensorResponse_()
  : result(0)
  , force()
  , torque()
  {
  }

  simRosReadForceSensorResponse_(const ContainerAllocator& _alloc)
  : result(0)
  , force(_alloc)
  , torque(_alloc)
  {
  }

  typedef int32_t _result_type;
  int32_t result;

  typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _force_type;
   ::geometry_msgs::Vector3_<ContainerAllocator>  force;

  typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _torque_type;
   ::geometry_msgs::Vector3_<ContainerAllocator>  torque;


  typedef boost::shared_ptr< ::vrep_common::simRosReadForceSensorResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosReadForceSensorResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosReadForceSensorResponse
typedef  ::vrep_common::simRosReadForceSensorResponse_<std::allocator<void> > simRosReadForceSensorResponse;

typedef boost::shared_ptr< ::vrep_common::simRosReadForceSensorResponse> simRosReadForceSensorResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosReadForceSensorResponse const> simRosReadForceSensorResponseConstPtr;

struct simRosReadForceSensor
{

typedef simRosReadForceSensorRequest Request;
typedef simRosReadForceSensorResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosReadForceSensor
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosReadForceSensorRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosReadForceSensorRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosReadForceSensorRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "92535b678299d2bdda959704e78c275e";
  }

  static const char* value(const  ::vrep_common::simRosReadForceSensorRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x92535b678299d2bdULL;
  static const uint64_t static_value2 = 0xda959704e78c275eULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosReadForceSensorRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosReadForceSensorRequest";
  }

  static const char* value(const  ::vrep_common::simRosReadForceSensorRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosReadForceSensorRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
int32 handle\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosReadForceSensorRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosReadForceSensorRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosReadForceSensorResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosReadForceSensorResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosReadForceSensorResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5e4b65925af0e441033ad70b707ce684";
  }

  static const char* value(const  ::vrep_common::simRosReadForceSensorResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5e4b65925af0e441ULL;
  static const uint64_t static_value2 = 0x033ad70b707ce684ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosReadForceSensorResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosReadForceSensorResponse";
  }

  static const char* value(const  ::vrep_common::simRosReadForceSensorResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosReadForceSensorResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 result\n\
geometry_msgs/Vector3 force\n\
geometry_msgs/Vector3 torque\n\
\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const  ::vrep_common::simRosReadForceSensorResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosReadForceSensorResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosReadForceSensorRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.handle);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosReadForceSensorRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosReadForceSensorResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
    stream.next(m.force);
    stream.next(m.torque);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosReadForceSensorResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosReadForceSensor> {
  static const char* value() 
  {
    return "f2a54a090baac0d8c5c08bb865b8cd2d";
  }

  static const char* value(const vrep_common::simRosReadForceSensor&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosReadForceSensor> {
  static const char* value() 
  {
    return "vrep_common/simRosReadForceSensor";
  }

  static const char* value(const vrep_common::simRosReadForceSensor&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosReadForceSensorRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f2a54a090baac0d8c5c08bb865b8cd2d";
  }

  static const char* value(const vrep_common::simRosReadForceSensorRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosReadForceSensorRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosReadForceSensor";
  }

  static const char* value(const vrep_common::simRosReadForceSensorRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosReadForceSensorResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f2a54a090baac0d8c5c08bb865b8cd2d";
  }

  static const char* value(const vrep_common::simRosReadForceSensorResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosReadForceSensorResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosReadForceSensor";
  }

  static const char* value(const vrep_common::simRosReadForceSensorResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSREADFORCESENSOR_H

