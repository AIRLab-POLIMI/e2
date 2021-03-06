/* Auto-generated by genmsg_cpp for file /home/jackal/ros_workspace/src/vrep/vrep_common/srv/simRosGetVisionSensorImage.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSGETVISIONSENSORIMAGE_H
#define VREP_COMMON_SERVICE_SIMROSGETVISIONSENSORIMAGE_H
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



#include "sensor_msgs/Image.h"

namespace vrep_common
{
template <class ContainerAllocator>
struct simRosGetVisionSensorImageRequest_ {
  typedef simRosGetVisionSensorImageRequest_<ContainerAllocator> Type;

  simRosGetVisionSensorImageRequest_()
  : handle(0)
  , options(0)
  {
  }

  simRosGetVisionSensorImageRequest_(const ContainerAllocator& _alloc)
  : handle(0)
  , options(0)
  {
  }

  typedef int32_t _handle_type;
  int32_t handle;

  typedef uint8_t _options_type;
  uint8_t options;


  typedef boost::shared_ptr< ::vrep_common::simRosGetVisionSensorImageRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosGetVisionSensorImageRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosGetVisionSensorImageRequest
typedef  ::vrep_common::simRosGetVisionSensorImageRequest_<std::allocator<void> > simRosGetVisionSensorImageRequest;

typedef boost::shared_ptr< ::vrep_common::simRosGetVisionSensorImageRequest> simRosGetVisionSensorImageRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosGetVisionSensorImageRequest const> simRosGetVisionSensorImageRequestConstPtr;


template <class ContainerAllocator>
struct simRosGetVisionSensorImageResponse_ {
  typedef simRosGetVisionSensorImageResponse_<ContainerAllocator> Type;

  simRosGetVisionSensorImageResponse_()
  : result(0)
  , image()
  {
  }

  simRosGetVisionSensorImageResponse_(const ContainerAllocator& _alloc)
  : result(0)
  , image(_alloc)
  {
  }

  typedef int32_t _result_type;
  int32_t result;

  typedef  ::sensor_msgs::Image_<ContainerAllocator>  _image_type;
   ::sensor_msgs::Image_<ContainerAllocator>  image;


  typedef boost::shared_ptr< ::vrep_common::simRosGetVisionSensorImageResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosGetVisionSensorImageResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosGetVisionSensorImageResponse
typedef  ::vrep_common::simRosGetVisionSensorImageResponse_<std::allocator<void> > simRosGetVisionSensorImageResponse;

typedef boost::shared_ptr< ::vrep_common::simRosGetVisionSensorImageResponse> simRosGetVisionSensorImageResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosGetVisionSensorImageResponse const> simRosGetVisionSensorImageResponseConstPtr;

struct simRosGetVisionSensorImage
{

typedef simRosGetVisionSensorImageRequest Request;
typedef simRosGetVisionSensorImageResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosGetVisionSensorImage
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetVisionSensorImageRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetVisionSensorImageRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosGetVisionSensorImageRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "849631fc9158c7822872c77f87a72668";
  }

  static const char* value(const  ::vrep_common::simRosGetVisionSensorImageRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x849631fc9158c782ULL;
  static const uint64_t static_value2 = 0x2872c77f87a72668ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosGetVisionSensorImageRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetVisionSensorImageRequest";
  }

  static const char* value(const  ::vrep_common::simRosGetVisionSensorImageRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosGetVisionSensorImageRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
int32 handle\n\
uint8 options\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosGetVisionSensorImageRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosGetVisionSensorImageRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetVisionSensorImageResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetVisionSensorImageResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosGetVisionSensorImageResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "52755fc56cbfa4eb9fe755efd8eb5ca6";
  }

  static const char* value(const  ::vrep_common::simRosGetVisionSensorImageResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x52755fc56cbfa4ebULL;
  static const uint64_t static_value2 = 0x9fe755efd8eb5ca6ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosGetVisionSensorImageResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetVisionSensorImageResponse";
  }

  static const char* value(const  ::vrep_common::simRosGetVisionSensorImageResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosGetVisionSensorImageResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 result\n\
sensor_msgs/Image image\n\
\n\
\n\
================================================================================\n\
MSG: sensor_msgs/Image\n\
# This message contains an uncompressed image\n\
# (0, 0) is at top-left corner of image\n\
#\n\
\n\
Header header        # Header timestamp should be acquisition time of image\n\
                     # Header frame_id should be optical frame of camera\n\
                     # origin of frame should be optical center of cameara\n\
                     # +x should point to the right in the image\n\
                     # +y should point down in the image\n\
                     # +z should point into to plane of the image\n\
                     # If the frame_id here and the frame_id of the CameraInfo\n\
                     # message associated with the image conflict\n\
                     # the behavior is undefined\n\
\n\
uint32 height         # image height, that is, number of rows\n\
uint32 width          # image width, that is, number of columns\n\
\n\
# The legal values for encoding are in file src/image_encodings.cpp\n\
# If you want to standardize a new string format, join\n\
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n\
\n\
string encoding       # Encoding of pixels -- channel meaning, ordering, size\n\
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h\n\
\n\
uint8 is_bigendian    # is this data bigendian?\n\
uint32 step           # Full row length in bytes\n\
uint8[] data          # actual matrix data, size is (step * rows)\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosGetVisionSensorImageResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosGetVisionSensorImageRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.handle);
    stream.next(m.options);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosGetVisionSensorImageRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosGetVisionSensorImageResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
    stream.next(m.image);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosGetVisionSensorImageResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosGetVisionSensorImage> {
  static const char* value() 
  {
    return "c9fa464de5ffa4b5a019f79bc572d29f";
  }

  static const char* value(const vrep_common::simRosGetVisionSensorImage&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosGetVisionSensorImage> {
  static const char* value() 
  {
    return "vrep_common/simRosGetVisionSensorImage";
  }

  static const char* value(const vrep_common::simRosGetVisionSensorImage&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosGetVisionSensorImageRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c9fa464de5ffa4b5a019f79bc572d29f";
  }

  static const char* value(const vrep_common::simRosGetVisionSensorImageRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosGetVisionSensorImageRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetVisionSensorImage";
  }

  static const char* value(const vrep_common::simRosGetVisionSensorImageRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosGetVisionSensorImageResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c9fa464de5ffa4b5a019f79bc572d29f";
  }

  static const char* value(const vrep_common::simRosGetVisionSensorImageResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosGetVisionSensorImageResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetVisionSensorImage";
  }

  static const char* value(const vrep_common::simRosGetVisionSensorImageResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSGETVISIONSENSORIMAGE_H

