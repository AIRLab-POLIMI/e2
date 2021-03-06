/* Auto-generated by genmsg_cpp for file /home/jackal/ros_workspace/src/vrep/vrep_common/srv/simRosGetIntegerParameter.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSGETINTEGERPARAMETER_H
#define VREP_COMMON_SERVICE_SIMROSGETINTEGERPARAMETER_H
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
struct simRosGetIntegerParameterRequest_ {
  typedef simRosGetIntegerParameterRequest_<ContainerAllocator> Type;

  simRosGetIntegerParameterRequest_()
  : parameter(0)
  {
  }

  simRosGetIntegerParameterRequest_(const ContainerAllocator& _alloc)
  : parameter(0)
  {
  }

  typedef int32_t _parameter_type;
  int32_t parameter;


  typedef boost::shared_ptr< ::vrep_common::simRosGetIntegerParameterRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosGetIntegerParameterRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosGetIntegerParameterRequest
typedef  ::vrep_common::simRosGetIntegerParameterRequest_<std::allocator<void> > simRosGetIntegerParameterRequest;

typedef boost::shared_ptr< ::vrep_common::simRosGetIntegerParameterRequest> simRosGetIntegerParameterRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosGetIntegerParameterRequest const> simRosGetIntegerParameterRequestConstPtr;


template <class ContainerAllocator>
struct simRosGetIntegerParameterResponse_ {
  typedef simRosGetIntegerParameterResponse_<ContainerAllocator> Type;

  simRosGetIntegerParameterResponse_()
  : result(0)
  , parameterValue(0)
  {
  }

  simRosGetIntegerParameterResponse_(const ContainerAllocator& _alloc)
  : result(0)
  , parameterValue(0)
  {
  }

  typedef int32_t _result_type;
  int32_t result;

  typedef int32_t _parameterValue_type;
  int32_t parameterValue;


  typedef boost::shared_ptr< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosGetIntegerParameterResponse
typedef  ::vrep_common::simRosGetIntegerParameterResponse_<std::allocator<void> > simRosGetIntegerParameterResponse;

typedef boost::shared_ptr< ::vrep_common::simRosGetIntegerParameterResponse> simRosGetIntegerParameterResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosGetIntegerParameterResponse const> simRosGetIntegerParameterResponseConstPtr;

struct simRosGetIntegerParameter
{

typedef simRosGetIntegerParameterRequest Request;
typedef simRosGetIntegerParameterResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosGetIntegerParameter
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetIntegerParameterRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetIntegerParameterRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosGetIntegerParameterRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3b5e34835331aac7a9065c5abd204e3b";
  }

  static const char* value(const  ::vrep_common::simRosGetIntegerParameterRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x3b5e34835331aac7ULL;
  static const uint64_t static_value2 = 0xa9065c5abd204e3bULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosGetIntegerParameterRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetIntegerParameterRequest";
  }

  static const char* value(const  ::vrep_common::simRosGetIntegerParameterRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosGetIntegerParameterRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
int32 parameter\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosGetIntegerParameterRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosGetIntegerParameterRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6c2f4c807e1ab6d671a7c18b9d47ce4a";
  }

  static const char* value(const  ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x6c2f4c807e1ab6d6ULL;
  static const uint64_t static_value2 = 0x71a7c18b9d47ce4aULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetIntegerParameterResponse";
  }

  static const char* value(const  ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 result\n\
int32 parameterValue\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosGetIntegerParameterRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.parameter);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosGetIntegerParameterRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
    stream.next(m.parameterValue);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosGetIntegerParameterResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosGetIntegerParameter> {
  static const char* value() 
  {
    return "60ddbc43aea06d61ad0817005dac2c22";
  }

  static const char* value(const vrep_common::simRosGetIntegerParameter&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosGetIntegerParameter> {
  static const char* value() 
  {
    return "vrep_common/simRosGetIntegerParameter";
  }

  static const char* value(const vrep_common::simRosGetIntegerParameter&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosGetIntegerParameterRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "60ddbc43aea06d61ad0817005dac2c22";
  }

  static const char* value(const vrep_common::simRosGetIntegerParameterRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosGetIntegerParameterRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetIntegerParameter";
  }

  static const char* value(const vrep_common::simRosGetIntegerParameterRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "60ddbc43aea06d61ad0817005dac2c22";
  }

  static const char* value(const vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetIntegerParameter";
  }

  static const char* value(const vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSGETINTEGERPARAMETER_H

