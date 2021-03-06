/* Auto-generated by genmsg_cpp for file /home/jackal/ros_workspace/src/vrep/vrep_common/srv/simRosGetModelProperty.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSGETMODELPROPERTY_H
#define VREP_COMMON_SERVICE_SIMROSGETMODELPROPERTY_H
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




namespace vrep_common
{
template <class ContainerAllocator>
struct simRosGetModelPropertyRequest_ {
  typedef simRosGetModelPropertyRequest_<ContainerAllocator> Type;

  simRosGetModelPropertyRequest_()
  : handle(0)
  {
  }

  simRosGetModelPropertyRequest_(const ContainerAllocator& _alloc)
  : handle(0)
  {
  }

  typedef int32_t _handle_type;
  int32_t handle;


  typedef boost::shared_ptr< ::vrep_common::simRosGetModelPropertyRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosGetModelPropertyRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosGetModelPropertyRequest
typedef  ::vrep_common::simRosGetModelPropertyRequest_<std::allocator<void> > simRosGetModelPropertyRequest;

typedef boost::shared_ptr< ::vrep_common::simRosGetModelPropertyRequest> simRosGetModelPropertyRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosGetModelPropertyRequest const> simRosGetModelPropertyRequestConstPtr;


template <class ContainerAllocator>
struct simRosGetModelPropertyResponse_ {
  typedef simRosGetModelPropertyResponse_<ContainerAllocator> Type;

  simRosGetModelPropertyResponse_()
  : propertyValue(0)
  {
  }

  simRosGetModelPropertyResponse_(const ContainerAllocator& _alloc)
  : propertyValue(0)
  {
  }

  typedef int32_t _propertyValue_type;
  int32_t propertyValue;


  typedef boost::shared_ptr< ::vrep_common::simRosGetModelPropertyResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosGetModelPropertyResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosGetModelPropertyResponse
typedef  ::vrep_common::simRosGetModelPropertyResponse_<std::allocator<void> > simRosGetModelPropertyResponse;

typedef boost::shared_ptr< ::vrep_common::simRosGetModelPropertyResponse> simRosGetModelPropertyResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosGetModelPropertyResponse const> simRosGetModelPropertyResponseConstPtr;

struct simRosGetModelProperty
{

typedef simRosGetModelPropertyRequest Request;
typedef simRosGetModelPropertyResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosGetModelProperty
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetModelPropertyRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetModelPropertyRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosGetModelPropertyRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "92535b678299d2bdda959704e78c275e";
  }

  static const char* value(const  ::vrep_common::simRosGetModelPropertyRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x92535b678299d2bdULL;
  static const uint64_t static_value2 = 0xda959704e78c275eULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosGetModelPropertyRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetModelPropertyRequest";
  }

  static const char* value(const  ::vrep_common::simRosGetModelPropertyRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosGetModelPropertyRequest_<ContainerAllocator> > {
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

  static const char* value(const  ::vrep_common::simRosGetModelPropertyRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosGetModelPropertyRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetModelPropertyResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetModelPropertyResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosGetModelPropertyResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "a3ff7a3737260dba8d61537cf18cc60a";
  }

  static const char* value(const  ::vrep_common::simRosGetModelPropertyResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xa3ff7a3737260dbaULL;
  static const uint64_t static_value2 = 0x8d61537cf18cc60aULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosGetModelPropertyResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetModelPropertyResponse";
  }

  static const char* value(const  ::vrep_common::simRosGetModelPropertyResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosGetModelPropertyResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 propertyValue\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosGetModelPropertyResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosGetModelPropertyResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosGetModelPropertyRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.handle);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosGetModelPropertyRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosGetModelPropertyResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.propertyValue);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosGetModelPropertyResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosGetModelProperty> {
  static const char* value() 
  {
    return "a54fb13fa756ea26f936de88d4121319";
  }

  static const char* value(const vrep_common::simRosGetModelProperty&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosGetModelProperty> {
  static const char* value() 
  {
    return "vrep_common/simRosGetModelProperty";
  }

  static const char* value(const vrep_common::simRosGetModelProperty&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosGetModelPropertyRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "a54fb13fa756ea26f936de88d4121319";
  }

  static const char* value(const vrep_common::simRosGetModelPropertyRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosGetModelPropertyRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetModelProperty";
  }

  static const char* value(const vrep_common::simRosGetModelPropertyRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosGetModelPropertyResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "a54fb13fa756ea26f936de88d4121319";
  }

  static const char* value(const vrep_common::simRosGetModelPropertyResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosGetModelPropertyResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetModelProperty";
  }

  static const char* value(const vrep_common::simRosGetModelPropertyResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSGETMODELPROPERTY_H

