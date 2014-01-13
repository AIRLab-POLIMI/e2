/* Auto-generated by genmsg_cpp for file /home/jackal/ros_workspace/src/vrep/vrep_common/srv/simRosGetStringSignal.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSGETSTRINGSIGNAL_H
#define VREP_COMMON_SERVICE_SIMROSGETSTRINGSIGNAL_H
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
struct simRosGetStringSignalRequest_ {
  typedef simRosGetStringSignalRequest_<ContainerAllocator> Type;

  simRosGetStringSignalRequest_()
  : signalName()
  {
  }

  simRosGetStringSignalRequest_(const ContainerAllocator& _alloc)
  : signalName(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _signalName_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  signalName;


  typedef boost::shared_ptr< ::vrep_common::simRosGetStringSignalRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosGetStringSignalRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosGetStringSignalRequest
typedef  ::vrep_common::simRosGetStringSignalRequest_<std::allocator<void> > simRosGetStringSignalRequest;

typedef boost::shared_ptr< ::vrep_common::simRosGetStringSignalRequest> simRosGetStringSignalRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosGetStringSignalRequest const> simRosGetStringSignalRequestConstPtr;


template <class ContainerAllocator>
struct simRosGetStringSignalResponse_ {
  typedef simRosGetStringSignalResponse_<ContainerAllocator> Type;

  simRosGetStringSignalResponse_()
  : result(0)
  , signalValue()
  {
  }

  simRosGetStringSignalResponse_(const ContainerAllocator& _alloc)
  : result(0)
  , signalValue(_alloc)
  {
  }

  typedef int32_t _result_type;
  int32_t result;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _signalValue_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  signalValue;


  typedef boost::shared_ptr< ::vrep_common::simRosGetStringSignalResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosGetStringSignalResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosGetStringSignalResponse
typedef  ::vrep_common::simRosGetStringSignalResponse_<std::allocator<void> > simRosGetStringSignalResponse;

typedef boost::shared_ptr< ::vrep_common::simRosGetStringSignalResponse> simRosGetStringSignalResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosGetStringSignalResponse const> simRosGetStringSignalResponseConstPtr;

struct simRosGetStringSignal
{

typedef simRosGetStringSignalRequest Request;
typedef simRosGetStringSignalResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosGetStringSignal
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetStringSignalRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetStringSignalRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosGetStringSignalRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "00c43203ad474c6ce746089e83166bba";
  }

  static const char* value(const  ::vrep_common::simRosGetStringSignalRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x00c43203ad474c6cULL;
  static const uint64_t static_value2 = 0xe746089e83166bbaULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosGetStringSignalRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetStringSignalRequest";
  }

  static const char* value(const  ::vrep_common::simRosGetStringSignalRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosGetStringSignalRequest_<ContainerAllocator> > {
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

  static const char* value(const  ::vrep_common::simRosGetStringSignalRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetStringSignalResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetStringSignalResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosGetStringSignalResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "db26c269346c2452f7366e4e3eed9867";
  }

  static const char* value(const  ::vrep_common::simRosGetStringSignalResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xdb26c269346c2452ULL;
  static const uint64_t static_value2 = 0xf7366e4e3eed9867ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosGetStringSignalResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetStringSignalResponse";
  }

  static const char* value(const  ::vrep_common::simRosGetStringSignalResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosGetStringSignalResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 result\n\
string signalValue\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosGetStringSignalResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosGetStringSignalRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.signalName);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosGetStringSignalRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosGetStringSignalResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
    stream.next(m.signalValue);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosGetStringSignalResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosGetStringSignal> {
  static const char* value() 
  {
    return "2404c94dd63ca66111460f0d88cb7f64";
  }

  static const char* value(const vrep_common::simRosGetStringSignal&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosGetStringSignal> {
  static const char* value() 
  {
    return "vrep_common/simRosGetStringSignal";
  }

  static const char* value(const vrep_common::simRosGetStringSignal&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosGetStringSignalRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "2404c94dd63ca66111460f0d88cb7f64";
  }

  static const char* value(const vrep_common::simRosGetStringSignalRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosGetStringSignalRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetStringSignal";
  }

  static const char* value(const vrep_common::simRosGetStringSignalRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosGetStringSignalResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "2404c94dd63ca66111460f0d88cb7f64";
  }

  static const char* value(const vrep_common::simRosGetStringSignalResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosGetStringSignalResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetStringSignal";
  }

  static const char* value(const vrep_common::simRosGetStringSignalResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSGETSTRINGSIGNAL_H

