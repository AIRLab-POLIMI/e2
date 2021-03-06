/* Auto-generated by genmsg_cpp for file /home/jackal/ros_workspace/src/vrep/vrep_common/srv/simRosSetUIButtonLabel.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSSETUIBUTTONLABEL_H
#define VREP_COMMON_SERVICE_SIMROSSETUIBUTTONLABEL_H
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
struct simRosSetUIButtonLabelRequest_ {
  typedef simRosSetUIButtonLabelRequest_<ContainerAllocator> Type;

  simRosSetUIButtonLabelRequest_()
  : uiHandle(0)
  , buttonID(0)
  , upStateLabel()
  , downStateLabel()
  {
  }

  simRosSetUIButtonLabelRequest_(const ContainerAllocator& _alloc)
  : uiHandle(0)
  , buttonID(0)
  , upStateLabel(_alloc)
  , downStateLabel(_alloc)
  {
  }

  typedef int32_t _uiHandle_type;
  int32_t uiHandle;

  typedef int32_t _buttonID_type;
  int32_t buttonID;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _upStateLabel_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  upStateLabel;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _downStateLabel_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  downStateLabel;


  typedef boost::shared_ptr< ::vrep_common::simRosSetUIButtonLabelRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosSetUIButtonLabelRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosSetUIButtonLabelRequest
typedef  ::vrep_common::simRosSetUIButtonLabelRequest_<std::allocator<void> > simRosSetUIButtonLabelRequest;

typedef boost::shared_ptr< ::vrep_common::simRosSetUIButtonLabelRequest> simRosSetUIButtonLabelRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosSetUIButtonLabelRequest const> simRosSetUIButtonLabelRequestConstPtr;


template <class ContainerAllocator>
struct simRosSetUIButtonLabelResponse_ {
  typedef simRosSetUIButtonLabelResponse_<ContainerAllocator> Type;

  simRosSetUIButtonLabelResponse_()
  : result(0)
  {
  }

  simRosSetUIButtonLabelResponse_(const ContainerAllocator& _alloc)
  : result(0)
  {
  }

  typedef int32_t _result_type;
  int32_t result;


  typedef boost::shared_ptr< ::vrep_common::simRosSetUIButtonLabelResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosSetUIButtonLabelResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosSetUIButtonLabelResponse
typedef  ::vrep_common::simRosSetUIButtonLabelResponse_<std::allocator<void> > simRosSetUIButtonLabelResponse;

typedef boost::shared_ptr< ::vrep_common::simRosSetUIButtonLabelResponse> simRosSetUIButtonLabelResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosSetUIButtonLabelResponse const> simRosSetUIButtonLabelResponseConstPtr;

struct simRosSetUIButtonLabel
{

typedef simRosSetUIButtonLabelRequest Request;
typedef simRosSetUIButtonLabelResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosSetUIButtonLabel
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetUIButtonLabelRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetUIButtonLabelRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosSetUIButtonLabelRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3e1ad0cb5a7e9bfb5b83bdbdf550ecc4";
  }

  static const char* value(const  ::vrep_common::simRosSetUIButtonLabelRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x3e1ad0cb5a7e9bfbULL;
  static const uint64_t static_value2 = 0x5b83bdbdf550ecc4ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosSetUIButtonLabelRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetUIButtonLabelRequest";
  }

  static const char* value(const  ::vrep_common::simRosSetUIButtonLabelRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosSetUIButtonLabelRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
int32 uiHandle\n\
int32 buttonID\n\
string upStateLabel\n\
string downStateLabel\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosSetUIButtonLabelRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetUIButtonLabelResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetUIButtonLabelResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosSetUIButtonLabelResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "034a8e20d6a306665e3a5b340fab3f09";
  }

  static const char* value(const  ::vrep_common::simRosSetUIButtonLabelResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x034a8e20d6a30666ULL;
  static const uint64_t static_value2 = 0x5e3a5b340fab3f09ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosSetUIButtonLabelResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetUIButtonLabelResponse";
  }

  static const char* value(const  ::vrep_common::simRosSetUIButtonLabelResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosSetUIButtonLabelResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 result\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosSetUIButtonLabelResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosSetUIButtonLabelResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosSetUIButtonLabelRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.uiHandle);
    stream.next(m.buttonID);
    stream.next(m.upStateLabel);
    stream.next(m.downStateLabel);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosSetUIButtonLabelRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosSetUIButtonLabelResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosSetUIButtonLabelResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosSetUIButtonLabel> {
  static const char* value() 
  {
    return "43f15cd21d5f9887f83532420e4a3463";
  }

  static const char* value(const vrep_common::simRosSetUIButtonLabel&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosSetUIButtonLabel> {
  static const char* value() 
  {
    return "vrep_common/simRosSetUIButtonLabel";
  }

  static const char* value(const vrep_common::simRosSetUIButtonLabel&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosSetUIButtonLabelRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "43f15cd21d5f9887f83532420e4a3463";
  }

  static const char* value(const vrep_common::simRosSetUIButtonLabelRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosSetUIButtonLabelRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetUIButtonLabel";
  }

  static const char* value(const vrep_common::simRosSetUIButtonLabelRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosSetUIButtonLabelResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "43f15cd21d5f9887f83532420e4a3463";
  }

  static const char* value(const vrep_common::simRosSetUIButtonLabelResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosSetUIButtonLabelResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetUIButtonLabel";
  }

  static const char* value(const vrep_common::simRosSetUIButtonLabelResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSSETUIBUTTONLABEL_H

