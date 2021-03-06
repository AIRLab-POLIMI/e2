/* Auto-generated by genmsg_cpp for file /home/jackal/ros_workspace/src/vrep/vrep_common/srv/simRosGetJointState.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSGETJOINTSTATE_H
#define VREP_COMMON_SERVICE_SIMROSGETJOINTSTATE_H
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



#include "sensor_msgs/JointState.h"

namespace vrep_common
{
template <class ContainerAllocator>
struct simRosGetJointStateRequest_ {
  typedef simRosGetJointStateRequest_<ContainerAllocator> Type;

  simRosGetJointStateRequest_()
  : handle(0)
  {
  }

  simRosGetJointStateRequest_(const ContainerAllocator& _alloc)
  : handle(0)
  {
  }

  typedef int32_t _handle_type;
  int32_t handle;


  typedef boost::shared_ptr< ::vrep_common::simRosGetJointStateRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosGetJointStateRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosGetJointStateRequest
typedef  ::vrep_common::simRosGetJointStateRequest_<std::allocator<void> > simRosGetJointStateRequest;

typedef boost::shared_ptr< ::vrep_common::simRosGetJointStateRequest> simRosGetJointStateRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosGetJointStateRequest const> simRosGetJointStateRequestConstPtr;


template <class ContainerAllocator>
struct simRosGetJointStateResponse_ {
  typedef simRosGetJointStateResponse_<ContainerAllocator> Type;

  simRosGetJointStateResponse_()
  : result(0)
  , state()
  {
  }

  simRosGetJointStateResponse_(const ContainerAllocator& _alloc)
  : result(0)
  , state(_alloc)
  {
  }

  typedef int32_t _result_type;
  int32_t result;

  typedef  ::sensor_msgs::JointState_<ContainerAllocator>  _state_type;
   ::sensor_msgs::JointState_<ContainerAllocator>  state;


  typedef boost::shared_ptr< ::vrep_common::simRosGetJointStateResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosGetJointStateResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosGetJointStateResponse
typedef  ::vrep_common::simRosGetJointStateResponse_<std::allocator<void> > simRosGetJointStateResponse;

typedef boost::shared_ptr< ::vrep_common::simRosGetJointStateResponse> simRosGetJointStateResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosGetJointStateResponse const> simRosGetJointStateResponseConstPtr;

struct simRosGetJointState
{

typedef simRosGetJointStateRequest Request;
typedef simRosGetJointStateResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosGetJointState
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetJointStateRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetJointStateRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosGetJointStateRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "92535b678299d2bdda959704e78c275e";
  }

  static const char* value(const  ::vrep_common::simRosGetJointStateRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x92535b678299d2bdULL;
  static const uint64_t static_value2 = 0xda959704e78c275eULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosGetJointStateRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetJointStateRequest";
  }

  static const char* value(const  ::vrep_common::simRosGetJointStateRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosGetJointStateRequest_<ContainerAllocator> > {
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

  static const char* value(const  ::vrep_common::simRosGetJointStateRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosGetJointStateRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetJointStateResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetJointStateResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosGetJointStateResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1d460a330c4107107a50f227d148b7d3";
  }

  static const char* value(const  ::vrep_common::simRosGetJointStateResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x1d460a330c410710ULL;
  static const uint64_t static_value2 = 0x7a50f227d148b7d3ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosGetJointStateResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetJointStateResponse";
  }

  static const char* value(const  ::vrep_common::simRosGetJointStateResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosGetJointStateResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 result\n\
sensor_msgs/JointState state\n\
\n\
\n\
================================================================================\n\
MSG: sensor_msgs/JointState\n\
# This is a message that holds data to describe the state of a set of torque controlled joints. \n\
#\n\
# The state of each joint (revolute or prismatic) is defined by:\n\
#  * the position of the joint (rad or m),\n\
#  * the velocity of the joint (rad/s or m/s) and \n\
#  * the effort that is applied in the joint (Nm or N).\n\
#\n\
# Each joint is uniquely identified by its name\n\
# The header specifies the time at which the joint states were recorded. All the joint states\n\
# in one message have to be recorded at the same time.\n\
#\n\
# This message consists of a multiple arrays, one for each part of the joint state. \n\
# The goal is to make each of the fields optional. When e.g. your joints have no\n\
# effort associated with them, you can leave the effort array empty. \n\
#\n\
# All arrays in this message should have the same size, or be empty.\n\
# This is the only way to uniquely associate the joint name with the correct\n\
# states.\n\
\n\
\n\
Header header\n\
\n\
string[] name\n\
float64[] position\n\
float64[] velocity\n\
float64[] effort\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosGetJointStateResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosGetJointStateRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.handle);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosGetJointStateRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosGetJointStateResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
    stream.next(m.state);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosGetJointStateResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosGetJointState> {
  static const char* value() 
  {
    return "e3e87944e85555d62f7a59817ea5aee6";
  }

  static const char* value(const vrep_common::simRosGetJointState&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosGetJointState> {
  static const char* value() 
  {
    return "vrep_common/simRosGetJointState";
  }

  static const char* value(const vrep_common::simRosGetJointState&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosGetJointStateRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e3e87944e85555d62f7a59817ea5aee6";
  }

  static const char* value(const vrep_common::simRosGetJointStateRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosGetJointStateRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetJointState";
  }

  static const char* value(const vrep_common::simRosGetJointStateRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosGetJointStateResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e3e87944e85555d62f7a59817ea5aee6";
  }

  static const char* value(const vrep_common::simRosGetJointStateResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosGetJointStateResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetJointState";
  }

  static const char* value(const vrep_common::simRosGetJointStateResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSGETJOINTSTATE_H

