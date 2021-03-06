/* Auto-generated by genmsg_cpp for file /home/jackal/ros_workspace/src/vrep/vrep_common/srv/simRosSetObjectQuaternion.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSSETOBJECTQUATERNION_H
#define VREP_COMMON_SERVICE_SIMROSSETOBJECTQUATERNION_H
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

#include "geometry_msgs/Quaternion.h"



namespace vrep_common
{
template <class ContainerAllocator>
struct simRosSetObjectQuaternionRequest_ {
  typedef simRosSetObjectQuaternionRequest_<ContainerAllocator> Type;

  simRosSetObjectQuaternionRequest_()
  : handle(0)
  , relativeToObjectHandle(0)
  , quaternion()
  {
  }

  simRosSetObjectQuaternionRequest_(const ContainerAllocator& _alloc)
  : handle(0)
  , relativeToObjectHandle(0)
  , quaternion(_alloc)
  {
  }

  typedef int32_t _handle_type;
  int32_t handle;

  typedef int32_t _relativeToObjectHandle_type;
  int32_t relativeToObjectHandle;

  typedef  ::geometry_msgs::Quaternion_<ContainerAllocator>  _quaternion_type;
   ::geometry_msgs::Quaternion_<ContainerAllocator>  quaternion;


  typedef boost::shared_ptr< ::vrep_common::simRosSetObjectQuaternionRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosSetObjectQuaternionRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosSetObjectQuaternionRequest
typedef  ::vrep_common::simRosSetObjectQuaternionRequest_<std::allocator<void> > simRosSetObjectQuaternionRequest;

typedef boost::shared_ptr< ::vrep_common::simRosSetObjectQuaternionRequest> simRosSetObjectQuaternionRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosSetObjectQuaternionRequest const> simRosSetObjectQuaternionRequestConstPtr;


template <class ContainerAllocator>
struct simRosSetObjectQuaternionResponse_ {
  typedef simRosSetObjectQuaternionResponse_<ContainerAllocator> Type;

  simRosSetObjectQuaternionResponse_()
  : result(0)
  {
  }

  simRosSetObjectQuaternionResponse_(const ContainerAllocator& _alloc)
  : result(0)
  {
  }

  typedef int32_t _result_type;
  int32_t result;


  typedef boost::shared_ptr< ::vrep_common::simRosSetObjectQuaternionResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosSetObjectQuaternionResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosSetObjectQuaternionResponse
typedef  ::vrep_common::simRosSetObjectQuaternionResponse_<std::allocator<void> > simRosSetObjectQuaternionResponse;

typedef boost::shared_ptr< ::vrep_common::simRosSetObjectQuaternionResponse> simRosSetObjectQuaternionResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosSetObjectQuaternionResponse const> simRosSetObjectQuaternionResponseConstPtr;

struct simRosSetObjectQuaternion
{

typedef simRosSetObjectQuaternionRequest Request;
typedef simRosSetObjectQuaternionResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosSetObjectQuaternion
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetObjectQuaternionRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetObjectQuaternionRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosSetObjectQuaternionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "9d74781e3cdb8dd84f786051a3a1c90b";
  }

  static const char* value(const  ::vrep_common::simRosSetObjectQuaternionRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x9d74781e3cdb8dd8ULL;
  static const uint64_t static_value2 = 0x4f786051a3a1c90bULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosSetObjectQuaternionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetObjectQuaternionRequest";
  }

  static const char* value(const  ::vrep_common::simRosSetObjectQuaternionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosSetObjectQuaternionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
int32 handle\n\
int32 relativeToObjectHandle\n\
geometry_msgs/Quaternion quaternion\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosSetObjectQuaternionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosSetObjectQuaternionRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetObjectQuaternionResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetObjectQuaternionResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosSetObjectQuaternionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "034a8e20d6a306665e3a5b340fab3f09";
  }

  static const char* value(const  ::vrep_common::simRosSetObjectQuaternionResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x034a8e20d6a30666ULL;
  static const uint64_t static_value2 = 0x5e3a5b340fab3f09ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosSetObjectQuaternionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetObjectQuaternionResponse";
  }

  static const char* value(const  ::vrep_common::simRosSetObjectQuaternionResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosSetObjectQuaternionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 result\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosSetObjectQuaternionResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosSetObjectQuaternionResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosSetObjectQuaternionRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.handle);
    stream.next(m.relativeToObjectHandle);
    stream.next(m.quaternion);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosSetObjectQuaternionRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosSetObjectQuaternionResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosSetObjectQuaternionResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosSetObjectQuaternion> {
  static const char* value() 
  {
    return "f5fe7b4813e58c37e0cb1c1585da009e";
  }

  static const char* value(const vrep_common::simRosSetObjectQuaternion&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosSetObjectQuaternion> {
  static const char* value() 
  {
    return "vrep_common/simRosSetObjectQuaternion";
  }

  static const char* value(const vrep_common::simRosSetObjectQuaternion&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosSetObjectQuaternionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f5fe7b4813e58c37e0cb1c1585da009e";
  }

  static const char* value(const vrep_common::simRosSetObjectQuaternionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosSetObjectQuaternionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetObjectQuaternion";
  }

  static const char* value(const vrep_common::simRosSetObjectQuaternionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosSetObjectQuaternionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f5fe7b4813e58c37e0cb1c1585da009e";
  }

  static const char* value(const vrep_common::simRosSetObjectQuaternionResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosSetObjectQuaternionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetObjectQuaternion";
  }

  static const char* value(const vrep_common::simRosSetObjectQuaternionResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSSETOBJECTQUATERNION_H

