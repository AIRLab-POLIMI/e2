/* Auto-generated by genmsg_cpp for file /home/jackal/ros_workspace/src/vrep/vrep_common/srv/simRosEnablePublisher.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSENABLEPUBLISHER_H
#define VREP_COMMON_SERVICE_SIMROSENABLEPUBLISHER_H
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
struct simRosEnablePublisherRequest_ {
  typedef simRosEnablePublisherRequest_<ContainerAllocator> Type;

  simRosEnablePublisherRequest_()
  : topicName()
  , queueSize(0)
  , streamCmd(0)
  , auxInt1(0)
  , auxInt2(0)
  , auxString()
  {
  }

  simRosEnablePublisherRequest_(const ContainerAllocator& _alloc)
  : topicName(_alloc)
  , queueSize(0)
  , streamCmd(0)
  , auxInt1(0)
  , auxInt2(0)
  , auxString(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _topicName_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  topicName;

  typedef int32_t _queueSize_type;
  int32_t queueSize;

  typedef int32_t _streamCmd_type;
  int32_t streamCmd;

  typedef int32_t _auxInt1_type;
  int32_t auxInt1;

  typedef int32_t _auxInt2_type;
  int32_t auxInt2;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _auxString_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  auxString;


  typedef boost::shared_ptr< ::vrep_common::simRosEnablePublisherRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosEnablePublisherRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosEnablePublisherRequest
typedef  ::vrep_common::simRosEnablePublisherRequest_<std::allocator<void> > simRosEnablePublisherRequest;

typedef boost::shared_ptr< ::vrep_common::simRosEnablePublisherRequest> simRosEnablePublisherRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosEnablePublisherRequest const> simRosEnablePublisherRequestConstPtr;


template <class ContainerAllocator>
struct simRosEnablePublisherResponse_ {
  typedef simRosEnablePublisherResponse_<ContainerAllocator> Type;

  simRosEnablePublisherResponse_()
  : effectiveTopicName()
  {
  }

  simRosEnablePublisherResponse_(const ContainerAllocator& _alloc)
  : effectiveTopicName(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _effectiveTopicName_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  effectiveTopicName;


  typedef boost::shared_ptr< ::vrep_common::simRosEnablePublisherResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosEnablePublisherResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosEnablePublisherResponse
typedef  ::vrep_common::simRosEnablePublisherResponse_<std::allocator<void> > simRosEnablePublisherResponse;

typedef boost::shared_ptr< ::vrep_common::simRosEnablePublisherResponse> simRosEnablePublisherResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosEnablePublisherResponse const> simRosEnablePublisherResponseConstPtr;

struct simRosEnablePublisher
{

typedef simRosEnablePublisherRequest Request;
typedef simRosEnablePublisherResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosEnablePublisher
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosEnablePublisherRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosEnablePublisherRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosEnablePublisherRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e7df7c248dc5801f9f5b98f61f3741b8";
  }

  static const char* value(const  ::vrep_common::simRosEnablePublisherRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xe7df7c248dc5801fULL;
  static const uint64_t static_value2 = 0x9f5b98f61f3741b8ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosEnablePublisherRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosEnablePublisherRequest";
  }

  static const char* value(const  ::vrep_common::simRosEnablePublisherRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosEnablePublisherRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
string topicName\n\
int32 queueSize\n\
int32 streamCmd\n\
int32 auxInt1\n\
int32 auxInt2\n\
string auxString\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosEnablePublisherRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosEnablePublisherResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosEnablePublisherResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosEnablePublisherResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8280ce6c5ec203a92c1559486bea2e2e";
  }

  static const char* value(const  ::vrep_common::simRosEnablePublisherResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8280ce6c5ec203a9ULL;
  static const uint64_t static_value2 = 0x2c1559486bea2e2eULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosEnablePublisherResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosEnablePublisherResponse";
  }

  static const char* value(const  ::vrep_common::simRosEnablePublisherResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosEnablePublisherResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string effectiveTopicName\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosEnablePublisherResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosEnablePublisherRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.topicName);
    stream.next(m.queueSize);
    stream.next(m.streamCmd);
    stream.next(m.auxInt1);
    stream.next(m.auxInt2);
    stream.next(m.auxString);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosEnablePublisherRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosEnablePublisherResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.effectiveTopicName);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosEnablePublisherResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosEnablePublisher> {
  static const char* value() 
  {
    return "1656daea9e4b58b53b650fa39f39e74e";
  }

  static const char* value(const vrep_common::simRosEnablePublisher&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosEnablePublisher> {
  static const char* value() 
  {
    return "vrep_common/simRosEnablePublisher";
  }

  static const char* value(const vrep_common::simRosEnablePublisher&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosEnablePublisherRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1656daea9e4b58b53b650fa39f39e74e";
  }

  static const char* value(const vrep_common::simRosEnablePublisherRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosEnablePublisherRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosEnablePublisher";
  }

  static const char* value(const vrep_common::simRosEnablePublisherRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosEnablePublisherResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1656daea9e4b58b53b650fa39f39e74e";
  }

  static const char* value(const vrep_common::simRosEnablePublisherResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosEnablePublisherResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosEnablePublisher";
  }

  static const char* value(const vrep_common::simRosEnablePublisherResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSENABLEPUBLISHER_H

