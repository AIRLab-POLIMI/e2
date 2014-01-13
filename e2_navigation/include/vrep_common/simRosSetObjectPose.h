/* Auto-generated by genmsg_cpp for file /home/jackal/ros_workspace/src/vrep/vrep_common/srv/simRosSetObjectPose.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSSETOBJECTPOSE_H
#define VREP_COMMON_SERVICE_SIMROSSETOBJECTPOSE_H
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

#include "geometry_msgs/Pose.h"



namespace vrep_common
{
template <class ContainerAllocator>
struct simRosSetObjectPoseRequest_ {
  typedef simRosSetObjectPoseRequest_<ContainerAllocator> Type;

  simRosSetObjectPoseRequest_()
  : handle(0)
  , relativeToObjectHandle(0)
  , pose()
  {
  }

  simRosSetObjectPoseRequest_(const ContainerAllocator& _alloc)
  : handle(0)
  , relativeToObjectHandle(0)
  , pose(_alloc)
  {
  }

  typedef int32_t _handle_type;
  int32_t handle;

  typedef int32_t _relativeToObjectHandle_type;
  int32_t relativeToObjectHandle;

  typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
   ::geometry_msgs::Pose_<ContainerAllocator>  pose;


  typedef boost::shared_ptr< ::vrep_common::simRosSetObjectPoseRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosSetObjectPoseRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosSetObjectPoseRequest
typedef  ::vrep_common::simRosSetObjectPoseRequest_<std::allocator<void> > simRosSetObjectPoseRequest;

typedef boost::shared_ptr< ::vrep_common::simRosSetObjectPoseRequest> simRosSetObjectPoseRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosSetObjectPoseRequest const> simRosSetObjectPoseRequestConstPtr;


template <class ContainerAllocator>
struct simRosSetObjectPoseResponse_ {
  typedef simRosSetObjectPoseResponse_<ContainerAllocator> Type;

  simRosSetObjectPoseResponse_()
  : result(0)
  {
  }

  simRosSetObjectPoseResponse_(const ContainerAllocator& _alloc)
  : result(0)
  {
  }

  typedef int32_t _result_type;
  int32_t result;


  typedef boost::shared_ptr< ::vrep_common::simRosSetObjectPoseResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosSetObjectPoseResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosSetObjectPoseResponse
typedef  ::vrep_common::simRosSetObjectPoseResponse_<std::allocator<void> > simRosSetObjectPoseResponse;

typedef boost::shared_ptr< ::vrep_common::simRosSetObjectPoseResponse> simRosSetObjectPoseResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosSetObjectPoseResponse const> simRosSetObjectPoseResponseConstPtr;

struct simRosSetObjectPose
{

typedef simRosSetObjectPoseRequest Request;
typedef simRosSetObjectPoseResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosSetObjectPose
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetObjectPoseRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetObjectPoseRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosSetObjectPoseRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e96867bf56162355e110db9de4e03f85";
  }

  static const char* value(const  ::vrep_common::simRosSetObjectPoseRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xe96867bf56162355ULL;
  static const uint64_t static_value2 = 0xe110db9de4e03f85ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosSetObjectPoseRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetObjectPoseRequest";
  }

  static const char* value(const  ::vrep_common::simRosSetObjectPoseRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosSetObjectPoseRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
int32 handle\n\
int32 relativeToObjectHandle\n\
geometry_msgs/Pose pose\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosSetObjectPoseRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosSetObjectPoseRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetObjectPoseResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetObjectPoseResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosSetObjectPoseResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "034a8e20d6a306665e3a5b340fab3f09";
  }

  static const char* value(const  ::vrep_common::simRosSetObjectPoseResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x034a8e20d6a30666ULL;
  static const uint64_t static_value2 = 0x5e3a5b340fab3f09ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosSetObjectPoseResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetObjectPoseResponse";
  }

  static const char* value(const  ::vrep_common::simRosSetObjectPoseResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosSetObjectPoseResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 result\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosSetObjectPoseResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosSetObjectPoseResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosSetObjectPoseRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.handle);
    stream.next(m.relativeToObjectHandle);
    stream.next(m.pose);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosSetObjectPoseRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosSetObjectPoseResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosSetObjectPoseResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosSetObjectPose> {
  static const char* value() 
  {
    return "61b308bf14be660ce4de1c3374dc2f73";
  }

  static const char* value(const vrep_common::simRosSetObjectPose&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosSetObjectPose> {
  static const char* value() 
  {
    return "vrep_common/simRosSetObjectPose";
  }

  static const char* value(const vrep_common::simRosSetObjectPose&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosSetObjectPoseRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "61b308bf14be660ce4de1c3374dc2f73";
  }

  static const char* value(const vrep_common::simRosSetObjectPoseRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosSetObjectPoseRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetObjectPose";
  }

  static const char* value(const vrep_common::simRosSetObjectPoseRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosSetObjectPoseResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "61b308bf14be660ce4de1c3374dc2f73";
  }

  static const char* value(const vrep_common::simRosSetObjectPoseResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosSetObjectPoseResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetObjectPose";
  }

  static const char* value(const vrep_common::simRosSetObjectPoseResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSSETOBJECTPOSE_H

