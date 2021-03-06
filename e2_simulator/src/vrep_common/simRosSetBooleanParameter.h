/* Auto-generated by genmsg_cpp for file /home/jackal/ros_workspace/src/vrep/vrep_common/srv/simRosSetBooleanParameter.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSSETBOOLEANPARAMETER_H
#define VREP_COMMON_SERVICE_SIMROSSETBOOLEANPARAMETER_H
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
struct simRosSetBooleanParameterRequest_ {
  typedef simRosSetBooleanParameterRequest_<ContainerAllocator> Type;

  simRosSetBooleanParameterRequest_()
  : parameter(0)
  , parameterValue(0)
  {
  }

  simRosSetBooleanParameterRequest_(const ContainerAllocator& _alloc)
  : parameter(0)
  , parameterValue(0)
  {
  }

  typedef int32_t _parameter_type;
  int32_t parameter;

  typedef uint8_t _parameterValue_type;
  uint8_t parameterValue;


  typedef boost::shared_ptr< ::vrep_common::simRosSetBooleanParameterRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosSetBooleanParameterRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosSetBooleanParameterRequest
typedef  ::vrep_common::simRosSetBooleanParameterRequest_<std::allocator<void> > simRosSetBooleanParameterRequest;

typedef boost::shared_ptr< ::vrep_common::simRosSetBooleanParameterRequest> simRosSetBooleanParameterRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosSetBooleanParameterRequest const> simRosSetBooleanParameterRequestConstPtr;


template <class ContainerAllocator>
struct simRosSetBooleanParameterResponse_ {
  typedef simRosSetBooleanParameterResponse_<ContainerAllocator> Type;

  simRosSetBooleanParameterResponse_()
  : result(0)
  {
  }

  simRosSetBooleanParameterResponse_(const ContainerAllocator& _alloc)
  : result(0)
  {
  }

  typedef int32_t _result_type;
  int32_t result;


  typedef boost::shared_ptr< ::vrep_common::simRosSetBooleanParameterResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosSetBooleanParameterResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosSetBooleanParameterResponse
typedef  ::vrep_common::simRosSetBooleanParameterResponse_<std::allocator<void> > simRosSetBooleanParameterResponse;

typedef boost::shared_ptr< ::vrep_common::simRosSetBooleanParameterResponse> simRosSetBooleanParameterResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosSetBooleanParameterResponse const> simRosSetBooleanParameterResponseConstPtr;

struct simRosSetBooleanParameter
{

typedef simRosSetBooleanParameterRequest Request;
typedef simRosSetBooleanParameterResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosSetBooleanParameter
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetBooleanParameterRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetBooleanParameterRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosSetBooleanParameterRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "10fa8f42f19dd4cbb2be19bb12f6a724";
  }

  static const char* value(const  ::vrep_common::simRosSetBooleanParameterRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x10fa8f42f19dd4cbULL;
  static const uint64_t static_value2 = 0xb2be19bb12f6a724ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosSetBooleanParameterRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetBooleanParameterRequest";
  }

  static const char* value(const  ::vrep_common::simRosSetBooleanParameterRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosSetBooleanParameterRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
int32 parameter\n\
uint8 parameterValue\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosSetBooleanParameterRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosSetBooleanParameterRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetBooleanParameterResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetBooleanParameterResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosSetBooleanParameterResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "034a8e20d6a306665e3a5b340fab3f09";
  }

  static const char* value(const  ::vrep_common::simRosSetBooleanParameterResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x034a8e20d6a30666ULL;
  static const uint64_t static_value2 = 0x5e3a5b340fab3f09ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosSetBooleanParameterResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetBooleanParameterResponse";
  }

  static const char* value(const  ::vrep_common::simRosSetBooleanParameterResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosSetBooleanParameterResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 result\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosSetBooleanParameterResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosSetBooleanParameterResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosSetBooleanParameterRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.parameter);
    stream.next(m.parameterValue);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosSetBooleanParameterRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosSetBooleanParameterResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosSetBooleanParameterResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosSetBooleanParameter> {
  static const char* value() 
  {
    return "48226e7166296d8d45f6626e588efb5a";
  }

  static const char* value(const vrep_common::simRosSetBooleanParameter&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosSetBooleanParameter> {
  static const char* value() 
  {
    return "vrep_common/simRosSetBooleanParameter";
  }

  static const char* value(const vrep_common::simRosSetBooleanParameter&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosSetBooleanParameterRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "48226e7166296d8d45f6626e588efb5a";
  }

  static const char* value(const vrep_common::simRosSetBooleanParameterRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosSetBooleanParameterRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetBooleanParameter";
  }

  static const char* value(const vrep_common::simRosSetBooleanParameterRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosSetBooleanParameterResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "48226e7166296d8d45f6626e588efb5a";
  }

  static const char* value(const vrep_common::simRosSetBooleanParameterResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosSetBooleanParameterResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetBooleanParameter";
  }

  static const char* value(const vrep_common::simRosSetBooleanParameterResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSSETBOOLEANPARAMETER_H

