/* Auto-generated by genmsg_cpp for file /home/jackal/ros_workspace/src/vrep/vrep_common/srv/simRosGetUISlider.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSGETUISLIDER_H
#define VREP_COMMON_SERVICE_SIMROSGETUISLIDER_H
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
struct simRosGetUISliderRequest_ {
  typedef simRosGetUISliderRequest_<ContainerAllocator> Type;

  simRosGetUISliderRequest_()
  : uiHandle(0)
  , buttonID(0)
  {
  }

  simRosGetUISliderRequest_(const ContainerAllocator& _alloc)
  : uiHandle(0)
  , buttonID(0)
  {
  }

  typedef int32_t _uiHandle_type;
  int32_t uiHandle;

  typedef int32_t _buttonID_type;
  int32_t buttonID;


  typedef boost::shared_ptr< ::vrep_common::simRosGetUISliderRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosGetUISliderRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosGetUISliderRequest
typedef  ::vrep_common::simRosGetUISliderRequest_<std::allocator<void> > simRosGetUISliderRequest;

typedef boost::shared_ptr< ::vrep_common::simRosGetUISliderRequest> simRosGetUISliderRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosGetUISliderRequest const> simRosGetUISliderRequestConstPtr;


template <class ContainerAllocator>
struct simRosGetUISliderResponse_ {
  typedef simRosGetUISliderResponse_<ContainerAllocator> Type;

  simRosGetUISliderResponse_()
  : position(0)
  {
  }

  simRosGetUISliderResponse_(const ContainerAllocator& _alloc)
  : position(0)
  {
  }

  typedef int32_t _position_type;
  int32_t position;


  typedef boost::shared_ptr< ::vrep_common::simRosGetUISliderResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosGetUISliderResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosGetUISliderResponse
typedef  ::vrep_common::simRosGetUISliderResponse_<std::allocator<void> > simRosGetUISliderResponse;

typedef boost::shared_ptr< ::vrep_common::simRosGetUISliderResponse> simRosGetUISliderResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosGetUISliderResponse const> simRosGetUISliderResponseConstPtr;

struct simRosGetUISlider
{

typedef simRosGetUISliderRequest Request;
typedef simRosGetUISliderResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosGetUISlider
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetUISliderRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetUISliderRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosGetUISliderRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3237fb7c1b11a9bf71b5bb80da60a11a";
  }

  static const char* value(const  ::vrep_common::simRosGetUISliderRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x3237fb7c1b11a9bfULL;
  static const uint64_t static_value2 = 0x71b5bb80da60a11aULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosGetUISliderRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetUISliderRequest";
  }

  static const char* value(const  ::vrep_common::simRosGetUISliderRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosGetUISliderRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
int32 uiHandle\n\
int32 buttonID\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosGetUISliderRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosGetUISliderRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetUISliderResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetUISliderResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosGetUISliderResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ada70156e12e6e31948c64c60d8bb212";
  }

  static const char* value(const  ::vrep_common::simRosGetUISliderResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xada70156e12e6e31ULL;
  static const uint64_t static_value2 = 0x948c64c60d8bb212ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosGetUISliderResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetUISliderResponse";
  }

  static const char* value(const  ::vrep_common::simRosGetUISliderResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosGetUISliderResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 position\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosGetUISliderResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosGetUISliderResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosGetUISliderRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.uiHandle);
    stream.next(m.buttonID);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosGetUISliderRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosGetUISliderResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.position);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosGetUISliderResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosGetUISlider> {
  static const char* value() 
  {
    return "5406790310ec56ad5fb998c1037f3650";
  }

  static const char* value(const vrep_common::simRosGetUISlider&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosGetUISlider> {
  static const char* value() 
  {
    return "vrep_common/simRosGetUISlider";
  }

  static const char* value(const vrep_common::simRosGetUISlider&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosGetUISliderRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5406790310ec56ad5fb998c1037f3650";
  }

  static const char* value(const vrep_common::simRosGetUISliderRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosGetUISliderRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetUISlider";
  }

  static const char* value(const vrep_common::simRosGetUISliderRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosGetUISliderResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5406790310ec56ad5fb998c1037f3650";
  }

  static const char* value(const vrep_common::simRosGetUISliderResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosGetUISliderResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetUISlider";
  }

  static const char* value(const vrep_common::simRosGetUISliderResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSGETUISLIDER_H

