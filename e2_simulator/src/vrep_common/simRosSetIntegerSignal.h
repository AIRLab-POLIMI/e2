/* Auto-generated by genmsg_cpp for file /home/jackal/ros_workspace/src/vrep/vrep_common/srv/simRosSetIntegerSignal.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSSETINTEGERSIGNAL_H
#define VREP_COMMON_SERVICE_SIMROSSETINTEGERSIGNAL_H
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
struct simRosSetIntegerSignalRequest_ {
  typedef simRosSetIntegerSignalRequest_<ContainerAllocator> Type;

  simRosSetIntegerSignalRequest_()
  : signalName()
  , signalValue(0)
  {
  }

  simRosSetIntegerSignalRequest_(const ContainerAllocator& _alloc)
  : signalName(_alloc)
  , signalValue(0)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _signalName_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  signalName;

  typedef int32_t _signalValue_type;
  int32_t signalValue;


  typedef boost::shared_ptr< ::vrep_common::simRosSetIntegerSignalRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosSetIntegerSignalRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosSetIntegerSignalRequest
typedef  ::vrep_common::simRosSetIntegerSignalRequest_<std::allocator<void> > simRosSetIntegerSignalRequest;

typedef boost::shared_ptr< ::vrep_common::simRosSetIntegerSignalRequest> simRosSetIntegerSignalRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosSetIntegerSignalRequest const> simRosSetIntegerSignalRequestConstPtr;


template <class ContainerAllocator>
struct simRosSetIntegerSignalResponse_ {
  typedef simRosSetIntegerSignalResponse_<ContainerAllocator> Type;

  simRosSetIntegerSignalResponse_()
  : result(0)
  {
  }

  simRosSetIntegerSignalResponse_(const ContainerAllocator& _alloc)
  : result(0)
  {
  }

  typedef int32_t _result_type;
  int32_t result;


  typedef boost::shared_ptr< ::vrep_common::simRosSetIntegerSignalResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosSetIntegerSignalResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosSetIntegerSignalResponse
typedef  ::vrep_common::simRosSetIntegerSignalResponse_<std::allocator<void> > simRosSetIntegerSignalResponse;

typedef boost::shared_ptr< ::vrep_common::simRosSetIntegerSignalResponse> simRosSetIntegerSignalResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosSetIntegerSignalResponse const> simRosSetIntegerSignalResponseConstPtr;

struct simRosSetIntegerSignal
{

typedef simRosSetIntegerSignalRequest Request;
typedef simRosSetIntegerSignalResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosSetIntegerSignal
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetIntegerSignalRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetIntegerSignalRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosSetIntegerSignalRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "70abbfe43fc78e6558d58b762827146c";
  }

  static const char* value(const  ::vrep_common::simRosSetIntegerSignalRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x70abbfe43fc78e65ULL;
  static const uint64_t static_value2 = 0x58d58b762827146cULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosSetIntegerSignalRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetIntegerSignalRequest";
  }

  static const char* value(const  ::vrep_common::simRosSetIntegerSignalRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosSetIntegerSignalRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
string signalName\n\
int32 signalValue\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosSetIntegerSignalRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetIntegerSignalResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetIntegerSignalResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosSetIntegerSignalResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "034a8e20d6a306665e3a5b340fab3f09";
  }

  static const char* value(const  ::vrep_common::simRosSetIntegerSignalResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x034a8e20d6a30666ULL;
  static const uint64_t static_value2 = 0x5e3a5b340fab3f09ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosSetIntegerSignalResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetIntegerSignalResponse";
  }

  static const char* value(const  ::vrep_common::simRosSetIntegerSignalResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosSetIntegerSignalResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 result\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosSetIntegerSignalResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosSetIntegerSignalResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosSetIntegerSignalRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.signalName);
    stream.next(m.signalValue);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosSetIntegerSignalRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosSetIntegerSignalResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosSetIntegerSignalResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosSetIntegerSignal> {
  static const char* value() 
  {
    return "6348838e968005d75ae2126a83942b2a";
  }

  static const char* value(const vrep_common::simRosSetIntegerSignal&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosSetIntegerSignal> {
  static const char* value() 
  {
    return "vrep_common/simRosSetIntegerSignal";
  }

  static const char* value(const vrep_common::simRosSetIntegerSignal&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosSetIntegerSignalRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6348838e968005d75ae2126a83942b2a";
  }

  static const char* value(const vrep_common::simRosSetIntegerSignalRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosSetIntegerSignalRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetIntegerSignal";
  }

  static const char* value(const vrep_common::simRosSetIntegerSignalRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosSetIntegerSignalResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6348838e968005d75ae2126a83942b2a";
  }

  static const char* value(const vrep_common::simRosSetIntegerSignalResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosSetIntegerSignalResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetIntegerSignal";
  }

  static const char* value(const vrep_common::simRosSetIntegerSignalResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSSETINTEGERSIGNAL_H

