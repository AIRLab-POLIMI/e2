/* Auto-generated by genmsg_cpp for file /home/jackal/ros_workspace/src/vrep/vrep_common/srv/simRosDisablePublisher.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSDISABLEPUBLISHER_H
#define VREP_COMMON_SERVICE_SIMROSDISABLEPUBLISHER_H
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
struct simRosDisablePublisherRequest_ {
  typedef simRosDisablePublisherRequest_<ContainerAllocator> Type;

  simRosDisablePublisherRequest_()
  : topicName()
  {
  }

  simRosDisablePublisherRequest_(const ContainerAllocator& _alloc)
  : topicName(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _topicName_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  topicName;


  typedef boost::shared_ptr< ::vrep_common::simRosDisablePublisherRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosDisablePublisherRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosDisablePublisherRequest
typedef  ::vrep_common::simRosDisablePublisherRequest_<std::allocator<void> > simRosDisablePublisherRequest;

typedef boost::shared_ptr< ::vrep_common::simRosDisablePublisherRequest> simRosDisablePublisherRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosDisablePublisherRequest const> simRosDisablePublisherRequestConstPtr;


template <class ContainerAllocator>
struct simRosDisablePublisherResponse_ {
  typedef simRosDisablePublisherResponse_<ContainerAllocator> Type;

  simRosDisablePublisherResponse_()
  : referenceCounter(0)
  {
  }

  simRosDisablePublisherResponse_(const ContainerAllocator& _alloc)
  : referenceCounter(0)
  {
  }

  typedef int32_t _referenceCounter_type;
  int32_t referenceCounter;


  typedef boost::shared_ptr< ::vrep_common::simRosDisablePublisherResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosDisablePublisherResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosDisablePublisherResponse
typedef  ::vrep_common::simRosDisablePublisherResponse_<std::allocator<void> > simRosDisablePublisherResponse;

typedef boost::shared_ptr< ::vrep_common::simRosDisablePublisherResponse> simRosDisablePublisherResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosDisablePublisherResponse const> simRosDisablePublisherResponseConstPtr;

struct simRosDisablePublisher
{

typedef simRosDisablePublisherRequest Request;
typedef simRosDisablePublisherResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosDisablePublisher
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosDisablePublisherRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosDisablePublisherRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosDisablePublisherRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "74b3306276d42621c8d9905fba018178";
  }

  static const char* value(const  ::vrep_common::simRosDisablePublisherRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x74b3306276d42621ULL;
  static const uint64_t static_value2 = 0xc8d9905fba018178ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosDisablePublisherRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosDisablePublisherRequest";
  }

  static const char* value(const  ::vrep_common::simRosDisablePublisherRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosDisablePublisherRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
string topicName\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosDisablePublisherRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosDisablePublisherResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosDisablePublisherResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosDisablePublisherResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b77f442068c4e56b29b4900433a0f3d6";
  }

  static const char* value(const  ::vrep_common::simRosDisablePublisherResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xb77f442068c4e56bULL;
  static const uint64_t static_value2 = 0x29b4900433a0f3d6ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosDisablePublisherResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosDisablePublisherResponse";
  }

  static const char* value(const  ::vrep_common::simRosDisablePublisherResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosDisablePublisherResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 referenceCounter\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosDisablePublisherResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosDisablePublisherResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosDisablePublisherRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.topicName);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosDisablePublisherRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosDisablePublisherResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.referenceCounter);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosDisablePublisherResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosDisablePublisher> {
  static const char* value() 
  {
    return "468250467bc8406ae24bdf79d4391996";
  }

  static const char* value(const vrep_common::simRosDisablePublisher&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosDisablePublisher> {
  static const char* value() 
  {
    return "vrep_common/simRosDisablePublisher";
  }

  static const char* value(const vrep_common::simRosDisablePublisher&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosDisablePublisherRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "468250467bc8406ae24bdf79d4391996";
  }

  static const char* value(const vrep_common::simRosDisablePublisherRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosDisablePublisherRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosDisablePublisher";
  }

  static const char* value(const vrep_common::simRosDisablePublisherRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosDisablePublisherResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "468250467bc8406ae24bdf79d4391996";
  }

  static const char* value(const vrep_common::simRosDisablePublisherResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosDisablePublisherResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosDisablePublisher";
  }

  static const char* value(const vrep_common::simRosDisablePublisherResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSDISABLEPUBLISHER_H

