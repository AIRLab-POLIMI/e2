/* Auto-generated by genmsg_cpp for file /home/jackal/ros_workspace/src/vrep/vrep_common/srv/simRosAuxiliaryConsoleOpen.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSAUXILIARYCONSOLEOPEN_H
#define VREP_COMMON_SERVICE_SIMROSAUXILIARYCONSOLEOPEN_H
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
struct simRosAuxiliaryConsoleOpenRequest_ {
  typedef simRosAuxiliaryConsoleOpenRequest_<ContainerAllocator> Type;

  simRosAuxiliaryConsoleOpenRequest_()
  : title()
  , maxLines(0)
  , mode(0)
  , position()
  , size()
  , textColor()
  , backgroundColor()
  {
  }

  simRosAuxiliaryConsoleOpenRequest_(const ContainerAllocator& _alloc)
  : title(_alloc)
  , maxLines(0)
  , mode(0)
  , position(_alloc)
  , size(_alloc)
  , textColor(_alloc)
  , backgroundColor(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _title_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  title;

  typedef int32_t _maxLines_type;
  int32_t maxLines;

  typedef int32_t _mode_type;
  int32_t mode;

  typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _position_type;
  std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  position;

  typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _size_type;
  std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  size;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _textColor_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  textColor;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _backgroundColor_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  backgroundColor;


  typedef boost::shared_ptr< ::vrep_common::simRosAuxiliaryConsoleOpenRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosAuxiliaryConsoleOpenRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosAuxiliaryConsoleOpenRequest
typedef  ::vrep_common::simRosAuxiliaryConsoleOpenRequest_<std::allocator<void> > simRosAuxiliaryConsoleOpenRequest;

typedef boost::shared_ptr< ::vrep_common::simRosAuxiliaryConsoleOpenRequest> simRosAuxiliaryConsoleOpenRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosAuxiliaryConsoleOpenRequest const> simRosAuxiliaryConsoleOpenRequestConstPtr;


template <class ContainerAllocator>
struct simRosAuxiliaryConsoleOpenResponse_ {
  typedef simRosAuxiliaryConsoleOpenResponse_<ContainerAllocator> Type;

  simRosAuxiliaryConsoleOpenResponse_()
  : consoleHandle(0)
  {
  }

  simRosAuxiliaryConsoleOpenResponse_(const ContainerAllocator& _alloc)
  : consoleHandle(0)
  {
  }

  typedef int32_t _consoleHandle_type;
  int32_t consoleHandle;


  typedef boost::shared_ptr< ::vrep_common::simRosAuxiliaryConsoleOpenResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosAuxiliaryConsoleOpenResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosAuxiliaryConsoleOpenResponse
typedef  ::vrep_common::simRosAuxiliaryConsoleOpenResponse_<std::allocator<void> > simRosAuxiliaryConsoleOpenResponse;

typedef boost::shared_ptr< ::vrep_common::simRosAuxiliaryConsoleOpenResponse> simRosAuxiliaryConsoleOpenResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosAuxiliaryConsoleOpenResponse const> simRosAuxiliaryConsoleOpenResponseConstPtr;

struct simRosAuxiliaryConsoleOpen
{

typedef simRosAuxiliaryConsoleOpenRequest Request;
typedef simRosAuxiliaryConsoleOpenResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosAuxiliaryConsoleOpen
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosAuxiliaryConsoleOpenRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosAuxiliaryConsoleOpenRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosAuxiliaryConsoleOpenRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b11b7ee0194549fd289229f6b0fe6c7a";
  }

  static const char* value(const  ::vrep_common::simRosAuxiliaryConsoleOpenRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xb11b7ee0194549fdULL;
  static const uint64_t static_value2 = 0x289229f6b0fe6c7aULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosAuxiliaryConsoleOpenRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosAuxiliaryConsoleOpenRequest";
  }

  static const char* value(const  ::vrep_common::simRosAuxiliaryConsoleOpenRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosAuxiliaryConsoleOpenRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
string title\n\
int32 maxLines\n\
int32 mode\n\
int32[] position\n\
int32[] size\n\
float32[] textColor\n\
float32[] backgroundColor\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosAuxiliaryConsoleOpenRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosAuxiliaryConsoleOpenResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosAuxiliaryConsoleOpenResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosAuxiliaryConsoleOpenResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0712f8f971970cd49793e7755140f018";
  }

  static const char* value(const  ::vrep_common::simRosAuxiliaryConsoleOpenResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x0712f8f971970cd4ULL;
  static const uint64_t static_value2 = 0x9793e7755140f018ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosAuxiliaryConsoleOpenResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosAuxiliaryConsoleOpenResponse";
  }

  static const char* value(const  ::vrep_common::simRosAuxiliaryConsoleOpenResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosAuxiliaryConsoleOpenResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 consoleHandle\n\
\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosAuxiliaryConsoleOpenResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosAuxiliaryConsoleOpenResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosAuxiliaryConsoleOpenRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.title);
    stream.next(m.maxLines);
    stream.next(m.mode);
    stream.next(m.position);
    stream.next(m.size);
    stream.next(m.textColor);
    stream.next(m.backgroundColor);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosAuxiliaryConsoleOpenRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosAuxiliaryConsoleOpenResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.consoleHandle);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosAuxiliaryConsoleOpenResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosAuxiliaryConsoleOpen> {
  static const char* value() 
  {
    return "f861a9b4fa1cfe42d343017f3c8914a5";
  }

  static const char* value(const vrep_common::simRosAuxiliaryConsoleOpen&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosAuxiliaryConsoleOpen> {
  static const char* value() 
  {
    return "vrep_common/simRosAuxiliaryConsoleOpen";
  }

  static const char* value(const vrep_common::simRosAuxiliaryConsoleOpen&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosAuxiliaryConsoleOpenRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f861a9b4fa1cfe42d343017f3c8914a5";
  }

  static const char* value(const vrep_common::simRosAuxiliaryConsoleOpenRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosAuxiliaryConsoleOpenRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosAuxiliaryConsoleOpen";
  }

  static const char* value(const vrep_common::simRosAuxiliaryConsoleOpenRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosAuxiliaryConsoleOpenResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f861a9b4fa1cfe42d343017f3c8914a5";
  }

  static const char* value(const vrep_common::simRosAuxiliaryConsoleOpenResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosAuxiliaryConsoleOpenResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosAuxiliaryConsoleOpen";
  }

  static const char* value(const vrep_common::simRosAuxiliaryConsoleOpenResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSAUXILIARYCONSOLEOPEN_H

