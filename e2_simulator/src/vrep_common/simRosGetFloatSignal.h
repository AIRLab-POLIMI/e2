/* Auto-generated by genmsg_cpp for file /home/jackal/ros_workspace/src/vrep/vrep_common/srv/simRosGetFloatSignal.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSGETFLOATSIGNAL_H
#define VREP_COMMON_SERVICE_SIMROSGETFLOATSIGNAL_H
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
struct simRosGetFloatSignalRequest_ {
  typedef simRosGetFloatSignalRequest_<ContainerAllocator> Type;

  simRosGetFloatSignalRequest_()
  : signalName()
  {
  }

  simRosGetFloatSignalRequest_(const ContainerAllocator& _alloc)
  : signalName(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _signalName_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  signalName;


  typedef boost::shared_ptr< ::vrep_common::simRosGetFloatSignalRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosGetFloatSignalRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosGetFloatSignalRequest
typedef  ::vrep_common::simRosGetFloatSignalRequest_<std::allocator<void> > simRosGetFloatSignalRequest;

typedef boost::shared_ptr< ::vrep_common::simRosGetFloatSignalRequest> simRosGetFloatSignalRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosGetFloatSignalRequest const> simRosGetFloatSignalRequestConstPtr;


template <class ContainerAllocator>
struct simRosGetFloatSignalResponse_ {
  typedef simRosGetFloatSignalResponse_<ContainerAllocator> Type;

  simRosGetFloatSignalResponse_()
  : result(0)
  , signalValue(0.0)
  {
  }

  simRosGetFloatSignalResponse_(const ContainerAllocator& _alloc)
  : result(0)
  , signalValue(0.0)
  {
  }

  typedef int32_t _result_type;
  int32_t result;

  typedef float _signalValue_type;
  float signalValue;


  typedef boost::shared_ptr< ::vrep_common::simRosGetFloatSignalResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosGetFloatSignalResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosGetFloatSignalResponse
typedef  ::vrep_common::simRosGetFloatSignalResponse_<std::allocator<void> > simRosGetFloatSignalResponse;

typedef boost::shared_ptr< ::vrep_common::simRosGetFloatSignalResponse> simRosGetFloatSignalResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosGetFloatSignalResponse const> simRosGetFloatSignalResponseConstPtr;

struct simRosGetFloatSignal
{

typedef simRosGetFloatSignalRequest Request;
typedef simRosGetFloatSignalResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosGetFloatSignal
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetFloatSignalRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetFloatSignalRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosGetFloatSignalRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "00c43203ad474c6ce746089e83166bba";
  }

  static const char* value(const  ::vrep_common::simRosGetFloatSignalRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x00c43203ad474c6cULL;
  static const uint64_t static_value2 = 0xe746089e83166bbaULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosGetFloatSignalRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetFloatSignalRequest";
  }

  static const char* value(const  ::vrep_common::simRosGetFloatSignalRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosGetFloatSignalRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
string signalName\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosGetFloatSignalRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetFloatSignalResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetFloatSignalResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosGetFloatSignalResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d8f2b92b89d5cf88cbffea18b9ddcc7d";
  }

  static const char* value(const  ::vrep_common::simRosGetFloatSignalResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd8f2b92b89d5cf88ULL;
  static const uint64_t static_value2 = 0xcbffea18b9ddcc7dULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosGetFloatSignalResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetFloatSignalResponse";
  }

  static const char* value(const  ::vrep_common::simRosGetFloatSignalResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosGetFloatSignalResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 result\n\
float32 signalValue\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosGetFloatSignalResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosGetFloatSignalResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosGetFloatSignalRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.signalName);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosGetFloatSignalRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosGetFloatSignalResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
    stream.next(m.signalValue);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosGetFloatSignalResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosGetFloatSignal> {
  static const char* value() 
  {
    return "52ba64366a10126c502d44acd3f25e3a";
  }

  static const char* value(const vrep_common::simRosGetFloatSignal&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosGetFloatSignal> {
  static const char* value() 
  {
    return "vrep_common/simRosGetFloatSignal";
  }

  static const char* value(const vrep_common::simRosGetFloatSignal&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosGetFloatSignalRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "52ba64366a10126c502d44acd3f25e3a";
  }

  static const char* value(const vrep_common::simRosGetFloatSignalRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosGetFloatSignalRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetFloatSignal";
  }

  static const char* value(const vrep_common::simRosGetFloatSignalRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosGetFloatSignalResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "52ba64366a10126c502d44acd3f25e3a";
  }

  static const char* value(const vrep_common::simRosGetFloatSignalResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosGetFloatSignalResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetFloatSignal";
  }

  static const char* value(const vrep_common::simRosGetFloatSignalResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSGETFLOATSIGNAL_H

