/* Auto-generated by genmsg_cpp for file /home/jackal/ros_workspace/src/vrep/vrep_common/srv/simRosAuxiliaryConsolePrint.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSAUXILIARYCONSOLEPRINT_H
#define VREP_COMMON_SERVICE_SIMROSAUXILIARYCONSOLEPRINT_H
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
struct simRosAuxiliaryConsolePrintRequest_ {
  typedef simRosAuxiliaryConsolePrintRequest_<ContainerAllocator> Type;

  simRosAuxiliaryConsolePrintRequest_()
  : consoleHandle(0)
  , text()
  {
  }

  simRosAuxiliaryConsolePrintRequest_(const ContainerAllocator& _alloc)
  : consoleHandle(0)
  , text(_alloc)
  {
  }

  typedef int32_t _consoleHandle_type;
  int32_t consoleHandle;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _text_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  text;


  typedef boost::shared_ptr< ::vrep_common::simRosAuxiliaryConsolePrintRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosAuxiliaryConsolePrintRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosAuxiliaryConsolePrintRequest
typedef  ::vrep_common::simRosAuxiliaryConsolePrintRequest_<std::allocator<void> > simRosAuxiliaryConsolePrintRequest;

typedef boost::shared_ptr< ::vrep_common::simRosAuxiliaryConsolePrintRequest> simRosAuxiliaryConsolePrintRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosAuxiliaryConsolePrintRequest const> simRosAuxiliaryConsolePrintRequestConstPtr;


template <class ContainerAllocator>
struct simRosAuxiliaryConsolePrintResponse_ {
  typedef simRosAuxiliaryConsolePrintResponse_<ContainerAllocator> Type;

  simRosAuxiliaryConsolePrintResponse_()
  : result(0)
  {
  }

  simRosAuxiliaryConsolePrintResponse_(const ContainerAllocator& _alloc)
  : result(0)
  {
  }

  typedef int32_t _result_type;
  int32_t result;


  typedef boost::shared_ptr< ::vrep_common::simRosAuxiliaryConsolePrintResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosAuxiliaryConsolePrintResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosAuxiliaryConsolePrintResponse
typedef  ::vrep_common::simRosAuxiliaryConsolePrintResponse_<std::allocator<void> > simRosAuxiliaryConsolePrintResponse;

typedef boost::shared_ptr< ::vrep_common::simRosAuxiliaryConsolePrintResponse> simRosAuxiliaryConsolePrintResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosAuxiliaryConsolePrintResponse const> simRosAuxiliaryConsolePrintResponseConstPtr;

struct simRosAuxiliaryConsolePrint
{

typedef simRosAuxiliaryConsolePrintRequest Request;
typedef simRosAuxiliaryConsolePrintResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosAuxiliaryConsolePrint
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosAuxiliaryConsolePrintRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosAuxiliaryConsolePrintRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosAuxiliaryConsolePrintRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "01b8405a29eed17e1ac8fe2b1db7c0a0";
  }

  static const char* value(const  ::vrep_common::simRosAuxiliaryConsolePrintRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x01b8405a29eed17eULL;
  static const uint64_t static_value2 = 0x1ac8fe2b1db7c0a0ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosAuxiliaryConsolePrintRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosAuxiliaryConsolePrintRequest";
  }

  static const char* value(const  ::vrep_common::simRosAuxiliaryConsolePrintRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosAuxiliaryConsolePrintRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
int32 consoleHandle\n\
string text\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosAuxiliaryConsolePrintRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosAuxiliaryConsolePrintResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosAuxiliaryConsolePrintResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosAuxiliaryConsolePrintResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "034a8e20d6a306665e3a5b340fab3f09";
  }

  static const char* value(const  ::vrep_common::simRosAuxiliaryConsolePrintResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x034a8e20d6a30666ULL;
  static const uint64_t static_value2 = 0x5e3a5b340fab3f09ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosAuxiliaryConsolePrintResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosAuxiliaryConsolePrintResponse";
  }

  static const char* value(const  ::vrep_common::simRosAuxiliaryConsolePrintResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosAuxiliaryConsolePrintResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 result\n\
\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosAuxiliaryConsolePrintResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosAuxiliaryConsolePrintResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosAuxiliaryConsolePrintRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.consoleHandle);
    stream.next(m.text);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosAuxiliaryConsolePrintRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosAuxiliaryConsolePrintResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosAuxiliaryConsolePrintResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosAuxiliaryConsolePrint> {
  static const char* value() 
  {
    return "9d84d1075d5165c5c1bafd9cf0faa6fd";
  }

  static const char* value(const vrep_common::simRosAuxiliaryConsolePrint&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosAuxiliaryConsolePrint> {
  static const char* value() 
  {
    return "vrep_common/simRosAuxiliaryConsolePrint";
  }

  static const char* value(const vrep_common::simRosAuxiliaryConsolePrint&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosAuxiliaryConsolePrintRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "9d84d1075d5165c5c1bafd9cf0faa6fd";
  }

  static const char* value(const vrep_common::simRosAuxiliaryConsolePrintRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosAuxiliaryConsolePrintRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosAuxiliaryConsolePrint";
  }

  static const char* value(const vrep_common::simRosAuxiliaryConsolePrintRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosAuxiliaryConsolePrintResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "9d84d1075d5165c5c1bafd9cf0faa6fd";
  }

  static const char* value(const vrep_common::simRosAuxiliaryConsolePrintResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosAuxiliaryConsolePrintResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosAuxiliaryConsolePrint";
  }

  static const char* value(const vrep_common::simRosAuxiliaryConsolePrintResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSAUXILIARYCONSOLEPRINT_H

