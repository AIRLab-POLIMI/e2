/* Auto-generated by genmsg_cpp for file /home/jackal/ros_workspace/src/vrep/vrep_common/srv/simRosRMLPosition.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSRMLPOSITION_H
#define VREP_COMMON_SERVICE_SIMROSRMLPOSITION_H
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
struct simRosRMLPositionRequest_ {
  typedef simRosRMLPositionRequest_<ContainerAllocator> Type;

  simRosRMLPositionRequest_()
  : dofs(0)
  , timeStep(0.0)
  , flags(0)
  , currentPosVelAccel()
  , maxVelAccelJerk()
  , selection()
  , targetPosVel()
  {
  }

  simRosRMLPositionRequest_(const ContainerAllocator& _alloc)
  : dofs(0)
  , timeStep(0.0)
  , flags(0)
  , currentPosVelAccel(_alloc)
  , maxVelAccelJerk(_alloc)
  , selection(_alloc)
  , targetPosVel(_alloc)
  {
  }

  typedef int32_t _dofs_type;
  int32_t dofs;

  typedef double _timeStep_type;
  double timeStep;

  typedef int32_t _flags_type;
  int32_t flags;

  typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _currentPosVelAccel_type;
  std::vector<double, typename ContainerAllocator::template rebind<double>::other >  currentPosVelAccel;

  typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _maxVelAccelJerk_type;
  std::vector<double, typename ContainerAllocator::template rebind<double>::other >  maxVelAccelJerk;

  typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _selection_type;
  std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  selection;

  typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _targetPosVel_type;
  std::vector<double, typename ContainerAllocator::template rebind<double>::other >  targetPosVel;


  typedef boost::shared_ptr< ::vrep_common::simRosRMLPositionRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosRMLPositionRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosRMLPositionRequest
typedef  ::vrep_common::simRosRMLPositionRequest_<std::allocator<void> > simRosRMLPositionRequest;

typedef boost::shared_ptr< ::vrep_common::simRosRMLPositionRequest> simRosRMLPositionRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosRMLPositionRequest const> simRosRMLPositionRequestConstPtr;


template <class ContainerAllocator>
struct simRosRMLPositionResponse_ {
  typedef simRosRMLPositionResponse_<ContainerAllocator> Type;

  simRosRMLPositionResponse_()
  : result(0)
  , newPosVelAccel()
  {
  }

  simRosRMLPositionResponse_(const ContainerAllocator& _alloc)
  : result(0)
  , newPosVelAccel(_alloc)
  {
  }

  typedef int32_t _result_type;
  int32_t result;

  typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _newPosVelAccel_type;
  std::vector<double, typename ContainerAllocator::template rebind<double>::other >  newPosVelAccel;


  typedef boost::shared_ptr< ::vrep_common::simRosRMLPositionResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosRMLPositionResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosRMLPositionResponse
typedef  ::vrep_common::simRosRMLPositionResponse_<std::allocator<void> > simRosRMLPositionResponse;

typedef boost::shared_ptr< ::vrep_common::simRosRMLPositionResponse> simRosRMLPositionResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosRMLPositionResponse const> simRosRMLPositionResponseConstPtr;

struct simRosRMLPosition
{

typedef simRosRMLPositionRequest Request;
typedef simRosRMLPositionResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosRMLPosition
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosRMLPositionRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosRMLPositionRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosRMLPositionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "51b047c1bcec03c8732e598e5af81fe8";
  }

  static const char* value(const  ::vrep_common::simRosRMLPositionRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x51b047c1bcec03c8ULL;
  static const uint64_t static_value2 = 0x732e598e5af81fe8ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosRMLPositionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosRMLPositionRequest";
  }

  static const char* value(const  ::vrep_common::simRosRMLPositionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosRMLPositionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
int32 dofs\n\
float64 timeStep\n\
int32 flags\n\
float64[] currentPosVelAccel\n\
float64[] maxVelAccelJerk\n\
uint8[] selection\n\
float64[] targetPosVel\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosRMLPositionRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosRMLPositionResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosRMLPositionResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosRMLPositionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c60aeff05e58a47d8d5ec6ccf1dfd343";
  }

  static const char* value(const  ::vrep_common::simRosRMLPositionResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xc60aeff05e58a47dULL;
  static const uint64_t static_value2 = 0x8d5ec6ccf1dfd343ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosRMLPositionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosRMLPositionResponse";
  }

  static const char* value(const  ::vrep_common::simRosRMLPositionResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosRMLPositionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 result\n\
float64[] newPosVelAccel\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosRMLPositionResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosRMLPositionRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.dofs);
    stream.next(m.timeStep);
    stream.next(m.flags);
    stream.next(m.currentPosVelAccel);
    stream.next(m.maxVelAccelJerk);
    stream.next(m.selection);
    stream.next(m.targetPosVel);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosRMLPositionRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosRMLPositionResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
    stream.next(m.newPosVelAccel);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosRMLPositionResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosRMLPosition> {
  static const char* value() 
  {
    return "856f1cf620db1f8c10961c091b09d93b";
  }

  static const char* value(const vrep_common::simRosRMLPosition&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosRMLPosition> {
  static const char* value() 
  {
    return "vrep_common/simRosRMLPosition";
  }

  static const char* value(const vrep_common::simRosRMLPosition&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosRMLPositionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "856f1cf620db1f8c10961c091b09d93b";
  }

  static const char* value(const vrep_common::simRosRMLPositionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosRMLPositionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosRMLPosition";
  }

  static const char* value(const vrep_common::simRosRMLPositionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosRMLPositionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "856f1cf620db1f8c10961c091b09d93b";
  }

  static const char* value(const vrep_common::simRosRMLPositionResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosRMLPositionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosRMLPosition";
  }

  static const char* value(const vrep_common::simRosRMLPositionResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSRMLPOSITION_H

