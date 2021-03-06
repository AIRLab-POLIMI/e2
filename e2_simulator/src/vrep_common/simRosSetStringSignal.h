/* Auto-generated by genmsg_cpp for file /home/jackal/ros_workspace/src/vrep/vrep_common/srv/simRosSetStringSignal.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSSETSTRINGSIGNAL_H
#define VREP_COMMON_SERVICE_SIMROSSETSTRINGSIGNAL_H
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
struct simRosSetStringSignalRequest_ {
  typedef simRosSetStringSignalRequest_<ContainerAllocator> Type;

  simRosSetStringSignalRequest_()
  : signalName()
  , signalValue()
  {
  }

  simRosSetStringSignalRequest_(const ContainerAllocator& _alloc)
  : signalName(_alloc)
  , signalValue(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _signalName_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  signalName;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _signalValue_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  signalValue;


  typedef boost::shared_ptr< ::vrep_common::simRosSetStringSignalRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosSetStringSignalRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosSetStringSignalRequest
typedef  ::vrep_common::simRosSetStringSignalRequest_<std::allocator<void> > simRosSetStringSignalRequest;

typedef boost::shared_ptr< ::vrep_common::simRosSetStringSignalRequest> simRosSetStringSignalRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosSetStringSignalRequest const> simRosSetStringSignalRequestConstPtr;


template <class ContainerAllocator>
struct simRosSetStringSignalResponse_ {
  typedef simRosSetStringSignalResponse_<ContainerAllocator> Type;

  simRosSetStringSignalResponse_()
  : result(0)
  {
  }

  simRosSetStringSignalResponse_(const ContainerAllocator& _alloc)
  : result(0)
  {
  }

  typedef int32_t _result_type;
  int32_t result;


  typedef boost::shared_ptr< ::vrep_common::simRosSetStringSignalResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosSetStringSignalResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosSetStringSignalResponse
typedef  ::vrep_common::simRosSetStringSignalResponse_<std::allocator<void> > simRosSetStringSignalResponse;

typedef boost::shared_ptr< ::vrep_common::simRosSetStringSignalResponse> simRosSetStringSignalResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosSetStringSignalResponse const> simRosSetStringSignalResponseConstPtr;

struct simRosSetStringSignal
{

typedef simRosSetStringSignalRequest Request;
typedef simRosSetStringSignalResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosSetStringSignal
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetStringSignalRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetStringSignalRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosSetStringSignalRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "a27f6ed9e44bd0fc3f310d02aeac102f";
  }

  static const char* value(const  ::vrep_common::simRosSetStringSignalRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xa27f6ed9e44bd0fcULL;
  static const uint64_t static_value2 = 0x3f310d02aeac102fULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosSetStringSignalRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetStringSignalRequest";
  }

  static const char* value(const  ::vrep_common::simRosSetStringSignalRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosSetStringSignalRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
string signalName\n\
string signalValue\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosSetStringSignalRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetStringSignalResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetStringSignalResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosSetStringSignalResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "034a8e20d6a306665e3a5b340fab3f09";
  }

  static const char* value(const  ::vrep_common::simRosSetStringSignalResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x034a8e20d6a30666ULL;
  static const uint64_t static_value2 = 0x5e3a5b340fab3f09ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosSetStringSignalResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetStringSignalResponse";
  }

  static const char* value(const  ::vrep_common::simRosSetStringSignalResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosSetStringSignalResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 result\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosSetStringSignalResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosSetStringSignalResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosSetStringSignalRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.signalName);
    stream.next(m.signalValue);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosSetStringSignalRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosSetStringSignalResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosSetStringSignalResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosSetStringSignal> {
  static const char* value() 
  {
    return "c49759e205be8f8195cc764a3a6b6c4d";
  }

  static const char* value(const vrep_common::simRosSetStringSignal&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosSetStringSignal> {
  static const char* value() 
  {
    return "vrep_common/simRosSetStringSignal";
  }

  static const char* value(const vrep_common::simRosSetStringSignal&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosSetStringSignalRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c49759e205be8f8195cc764a3a6b6c4d";
  }

  static const char* value(const vrep_common::simRosSetStringSignalRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosSetStringSignalRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetStringSignal";
  }

  static const char* value(const vrep_common::simRosSetStringSignalRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosSetStringSignalResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c49759e205be8f8195cc764a3a6b6c4d";
  }

  static const char* value(const vrep_common::simRosSetStringSignalResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosSetStringSignalResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetStringSignal";
  }

  static const char* value(const vrep_common::simRosSetStringSignalResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSSETSTRINGSIGNAL_H

