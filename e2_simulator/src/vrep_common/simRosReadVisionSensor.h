/* Auto-generated by genmsg_cpp for file /home/jackal/ros_workspace/src/vrep/vrep_common/srv/simRosReadVisionSensor.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSREADVISIONSENSOR_H
#define VREP_COMMON_SERVICE_SIMROSREADVISIONSENSOR_H
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
struct simRosReadVisionSensorRequest_ {
  typedef simRosReadVisionSensorRequest_<ContainerAllocator> Type;

  simRosReadVisionSensorRequest_()
  : handle(0)
  {
  }

  simRosReadVisionSensorRequest_(const ContainerAllocator& _alloc)
  : handle(0)
  {
  }

  typedef int32_t _handle_type;
  int32_t handle;


  typedef boost::shared_ptr< ::vrep_common::simRosReadVisionSensorRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosReadVisionSensorRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosReadVisionSensorRequest
typedef  ::vrep_common::simRosReadVisionSensorRequest_<std::allocator<void> > simRosReadVisionSensorRequest;

typedef boost::shared_ptr< ::vrep_common::simRosReadVisionSensorRequest> simRosReadVisionSensorRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosReadVisionSensorRequest const> simRosReadVisionSensorRequestConstPtr;


template <class ContainerAllocator>
struct simRosReadVisionSensorResponse_ {
  typedef simRosReadVisionSensorResponse_<ContainerAllocator> Type;

  simRosReadVisionSensorResponse_()
  : result(0)
  , packetData()
  , packetSizes()
  {
  }

  simRosReadVisionSensorResponse_(const ContainerAllocator& _alloc)
  : result(0)
  , packetData(_alloc)
  , packetSizes(_alloc)
  {
  }

  typedef int32_t _result_type;
  int32_t result;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _packetData_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  packetData;

  typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _packetSizes_type;
  std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  packetSizes;


  typedef boost::shared_ptr< ::vrep_common::simRosReadVisionSensorResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosReadVisionSensorResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosReadVisionSensorResponse
typedef  ::vrep_common::simRosReadVisionSensorResponse_<std::allocator<void> > simRosReadVisionSensorResponse;

typedef boost::shared_ptr< ::vrep_common::simRosReadVisionSensorResponse> simRosReadVisionSensorResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosReadVisionSensorResponse const> simRosReadVisionSensorResponseConstPtr;

struct simRosReadVisionSensor
{

typedef simRosReadVisionSensorRequest Request;
typedef simRosReadVisionSensorResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosReadVisionSensor
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosReadVisionSensorRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosReadVisionSensorRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosReadVisionSensorRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "92535b678299d2bdda959704e78c275e";
  }

  static const char* value(const  ::vrep_common::simRosReadVisionSensorRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x92535b678299d2bdULL;
  static const uint64_t static_value2 = 0xda959704e78c275eULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosReadVisionSensorRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosReadVisionSensorRequest";
  }

  static const char* value(const  ::vrep_common::simRosReadVisionSensorRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosReadVisionSensorRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
int32 handle\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosReadVisionSensorRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosReadVisionSensorRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosReadVisionSensorResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosReadVisionSensorResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosReadVisionSensorResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "a46558cdf2ec9ef5fe9ba6ee4605c6bc";
  }

  static const char* value(const  ::vrep_common::simRosReadVisionSensorResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xa46558cdf2ec9ef5ULL;
  static const uint64_t static_value2 = 0xfe9ba6ee4605c6bcULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosReadVisionSensorResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosReadVisionSensorResponse";
  }

  static const char* value(const  ::vrep_common::simRosReadVisionSensorResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosReadVisionSensorResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 result\n\
float32[] packetData\n\
int32[] packetSizes\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosReadVisionSensorResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosReadVisionSensorRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.handle);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosReadVisionSensorRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosReadVisionSensorResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
    stream.next(m.packetData);
    stream.next(m.packetSizes);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosReadVisionSensorResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosReadVisionSensor> {
  static const char* value() 
  {
    return "06a229ce8d1f580db5bd558c65f74c4e";
  }

  static const char* value(const vrep_common::simRosReadVisionSensor&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosReadVisionSensor> {
  static const char* value() 
  {
    return "vrep_common/simRosReadVisionSensor";
  }

  static const char* value(const vrep_common::simRosReadVisionSensor&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosReadVisionSensorRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "06a229ce8d1f580db5bd558c65f74c4e";
  }

  static const char* value(const vrep_common::simRosReadVisionSensorRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosReadVisionSensorRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosReadVisionSensor";
  }

  static const char* value(const vrep_common::simRosReadVisionSensorRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosReadVisionSensorResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "06a229ce8d1f580db5bd558c65f74c4e";
  }

  static const char* value(const vrep_common::simRosReadVisionSensorResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosReadVisionSensorResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosReadVisionSensor";
  }

  static const char* value(const vrep_common::simRosReadVisionSensorResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSREADVISIONSENSOR_H

