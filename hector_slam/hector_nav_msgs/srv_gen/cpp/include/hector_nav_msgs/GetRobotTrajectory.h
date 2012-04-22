/* Auto-generated by genmsg_cpp for file /home/tallevy/ros-workspace/hector_slam/hector_nav_msgs/srv/GetRobotTrajectory.srv */
#ifndef HECTOR_NAV_MSGS_SERVICE_GETROBOTTRAJECTORY_H
#define HECTOR_NAV_MSGS_SERVICE_GETROBOTTRAJECTORY_H
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



#include "nav_msgs/Path.h"

namespace hector_nav_msgs
{
template <class ContainerAllocator>
struct GetRobotTrajectoryRequest_ {
  typedef GetRobotTrajectoryRequest_<ContainerAllocator> Type;

  GetRobotTrajectoryRequest_()
  {
  }

  GetRobotTrajectoryRequest_(const ContainerAllocator& _alloc)
  {
  }


private:
  static const char* __s_getDataType_() { return "hector_nav_msgs/GetRobotTrajectoryRequest"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "d41d8cd98f00b204e9800998ecf8427e"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "c7bd40129c5786fc26351edbd33b8d33"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "\n\
\n\
\n\
\n\
\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    return size;
  }

  typedef boost::shared_ptr< ::hector_nav_msgs::GetRobotTrajectoryRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hector_nav_msgs::GetRobotTrajectoryRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct GetRobotTrajectoryRequest
typedef  ::hector_nav_msgs::GetRobotTrajectoryRequest_<std::allocator<void> > GetRobotTrajectoryRequest;

typedef boost::shared_ptr< ::hector_nav_msgs::GetRobotTrajectoryRequest> GetRobotTrajectoryRequestPtr;
typedef boost::shared_ptr< ::hector_nav_msgs::GetRobotTrajectoryRequest const> GetRobotTrajectoryRequestConstPtr;


template <class ContainerAllocator>
struct GetRobotTrajectoryResponse_ {
  typedef GetRobotTrajectoryResponse_<ContainerAllocator> Type;

  GetRobotTrajectoryResponse_()
  : trajectory()
  {
  }

  GetRobotTrajectoryResponse_(const ContainerAllocator& _alloc)
  : trajectory(_alloc)
  {
  }

  typedef  ::nav_msgs::Path_<ContainerAllocator>  _trajectory_type;
   ::nav_msgs::Path_<ContainerAllocator>  trajectory;


private:
  static const char* __s_getDataType_() { return "hector_nav_msgs/GetRobotTrajectoryResponse"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "c7bd40129c5786fc26351edbd33b8d33"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "c7bd40129c5786fc26351edbd33b8d33"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "nav_msgs/Path trajectory\n\
\n\
\n\
\n\
================================================================================\n\
MSG: nav_msgs/Path\n\
#An array of poses that represents a Path for a robot to follow\n\
Header header\n\
geometry_msgs/PoseStamped[] poses\n\
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
================================================================================\n\
MSG: geometry_msgs/PoseStamped\n\
# A Pose with reference coordinate frame and timestamp\n\
Header header\n\
Pose pose\n\
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
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, trajectory);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, trajectory);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(trajectory);
    return size;
  }

  typedef boost::shared_ptr< ::hector_nav_msgs::GetRobotTrajectoryResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hector_nav_msgs::GetRobotTrajectoryResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct GetRobotTrajectoryResponse
typedef  ::hector_nav_msgs::GetRobotTrajectoryResponse_<std::allocator<void> > GetRobotTrajectoryResponse;

typedef boost::shared_ptr< ::hector_nav_msgs::GetRobotTrajectoryResponse> GetRobotTrajectoryResponsePtr;
typedef boost::shared_ptr< ::hector_nav_msgs::GetRobotTrajectoryResponse const> GetRobotTrajectoryResponseConstPtr;

struct GetRobotTrajectory
{

typedef GetRobotTrajectoryRequest Request;
typedef GetRobotTrajectoryResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct GetRobotTrajectory
} // namespace hector_nav_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::hector_nav_msgs::GetRobotTrajectoryRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::hector_nav_msgs::GetRobotTrajectoryRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::hector_nav_msgs::GetRobotTrajectoryRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::hector_nav_msgs::GetRobotTrajectoryRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::hector_nav_msgs::GetRobotTrajectoryRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hector_nav_msgs/GetRobotTrajectoryRequest";
  }

  static const char* value(const  ::hector_nav_msgs::GetRobotTrajectoryRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::hector_nav_msgs::GetRobotTrajectoryRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
\n\
\n\
";
  }

  static const char* value(const  ::hector_nav_msgs::GetRobotTrajectoryRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::hector_nav_msgs::GetRobotTrajectoryRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::hector_nav_msgs::GetRobotTrajectoryResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::hector_nav_msgs::GetRobotTrajectoryResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::hector_nav_msgs::GetRobotTrajectoryResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c7bd40129c5786fc26351edbd33b8d33";
  }

  static const char* value(const  ::hector_nav_msgs::GetRobotTrajectoryResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xc7bd40129c5786fcULL;
  static const uint64_t static_value2 = 0x26351edbd33b8d33ULL;
};

template<class ContainerAllocator>
struct DataType< ::hector_nav_msgs::GetRobotTrajectoryResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hector_nav_msgs/GetRobotTrajectoryResponse";
  }

  static const char* value(const  ::hector_nav_msgs::GetRobotTrajectoryResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::hector_nav_msgs::GetRobotTrajectoryResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nav_msgs/Path trajectory\n\
\n\
\n\
\n\
================================================================================\n\
MSG: nav_msgs/Path\n\
#An array of poses that represents a Path for a robot to follow\n\
Header header\n\
geometry_msgs/PoseStamped[] poses\n\
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
================================================================================\n\
MSG: geometry_msgs/PoseStamped\n\
# A Pose with reference coordinate frame and timestamp\n\
Header header\n\
Pose pose\n\
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

  static const char* value(const  ::hector_nav_msgs::GetRobotTrajectoryResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::hector_nav_msgs::GetRobotTrajectoryRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct GetRobotTrajectoryRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::hector_nav_msgs::GetRobotTrajectoryResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.trajectory);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct GetRobotTrajectoryResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<hector_nav_msgs::GetRobotTrajectory> {
  static const char* value() 
  {
    return "c7bd40129c5786fc26351edbd33b8d33";
  }

  static const char* value(const hector_nav_msgs::GetRobotTrajectory&) { return value(); } 
};

template<>
struct DataType<hector_nav_msgs::GetRobotTrajectory> {
  static const char* value() 
  {
    return "hector_nav_msgs/GetRobotTrajectory";
  }

  static const char* value(const hector_nav_msgs::GetRobotTrajectory&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<hector_nav_msgs::GetRobotTrajectoryRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c7bd40129c5786fc26351edbd33b8d33";
  }

  static const char* value(const hector_nav_msgs::GetRobotTrajectoryRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<hector_nav_msgs::GetRobotTrajectoryRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hector_nav_msgs/GetRobotTrajectory";
  }

  static const char* value(const hector_nav_msgs::GetRobotTrajectoryRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<hector_nav_msgs::GetRobotTrajectoryResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c7bd40129c5786fc26351edbd33b8d33";
  }

  static const char* value(const hector_nav_msgs::GetRobotTrajectoryResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<hector_nav_msgs::GetRobotTrajectoryResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hector_nav_msgs/GetRobotTrajectory";
  }

  static const char* value(const hector_nav_msgs::GetRobotTrajectoryResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // HECTOR_NAV_MSGS_SERVICE_GETROBOTTRAJECTORY_H

