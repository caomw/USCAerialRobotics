/* Auto-generated by genmsg_cpp for file /home/russell/ros/USCAerialRobotics/art_lrf/msg/Lines.msg */
#ifndef ART_LRF_MESSAGE_LINES_H
#define ART_LRF_MESSAGE_LINES_H
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

#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Polygon.h"

namespace art_lrf
{
template <class ContainerAllocator>
struct Lines_ {
  typedef Lines_<ContainerAllocator> Type;

  Lines_()
  : theta_index()
  , est_rho()
  , est_theta()
  , delta_rho()
  , endpoints()
  , lengths()
  , endpoint_ranges()
  , theta()
  {
  }

  Lines_(const ContainerAllocator& _alloc)
  : theta_index(_alloc)
  , est_rho(_alloc)
  , est_theta(_alloc)
  , delta_rho(_alloc)
  , endpoints(_alloc)
  , lengths(_alloc)
  , endpoint_ranges(_alloc)
  , theta(_alloc)
  {
  }

  typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _theta_index_type;
  std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  theta_index;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _est_rho_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  est_rho;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _est_theta_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  est_theta;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _delta_rho_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  delta_rho;

  typedef std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other >  _endpoints_type;
  std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other >  endpoints;

  typedef std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other >  _lengths_type;
  std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other >  lengths;

  typedef std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other >  _endpoint_ranges_type;
  std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other >  endpoint_ranges;

  typedef std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other >  _theta_type;
  std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other >  theta;


  typedef boost::shared_ptr< ::art_lrf::Lines_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::art_lrf::Lines_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Lines
typedef  ::art_lrf::Lines_<std::allocator<void> > Lines;

typedef boost::shared_ptr< ::art_lrf::Lines> LinesPtr;
typedef boost::shared_ptr< ::art_lrf::Lines const> LinesConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::art_lrf::Lines_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::art_lrf::Lines_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace art_lrf

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::art_lrf::Lines_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::art_lrf::Lines_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::art_lrf::Lines_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5e886f1edefc993f520f05d33f4e8b92";
  }

  static const char* value(const  ::art_lrf::Lines_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5e886f1edefc993fULL;
  static const uint64_t static_value2 = 0x520f05d33f4e8b92ULL;
};

template<class ContainerAllocator>
struct DataType< ::art_lrf::Lines_<ContainerAllocator> > {
  static const char* value() 
  {
    return "art_lrf/Lines";
  }

  static const char* value(const  ::art_lrf::Lines_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::art_lrf::Lines_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32[] theta_index\n\
float32[] est_rho\n\
float32[] est_theta\n\
float32[] delta_rho\n\
geometry_msgs/Polygon[] endpoints\n\
geometry_msgs/Polygon[] lengths\n\
geometry_msgs/Polygon[] endpoint_ranges\n\
geometry_msgs/Polygon[] theta\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Polygon\n\
#A specification of a polygon where the first and last points are assumed to be connected\n\
Point32[] points\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point32\n\
# This contains the position of a point in free space(with 32 bits of precision).\n\
# It is recommeded to use Point wherever possible instead of Point32.  \n\
# \n\
# This recommendation is to promote interoperability.  \n\
#\n\
# This message is designed to take up less space when sending\n\
# lots of points at once, as in the case of a PointCloud.  \n\
\n\
float32 x\n\
float32 y\n\
float32 z\n\
";
  }

  static const char* value(const  ::art_lrf::Lines_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::art_lrf::Lines_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.theta_index);
    stream.next(m.est_rho);
    stream.next(m.est_theta);
    stream.next(m.delta_rho);
    stream.next(m.endpoints);
    stream.next(m.lengths);
    stream.next(m.endpoint_ranges);
    stream.next(m.theta);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Lines_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::art_lrf::Lines_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::art_lrf::Lines_<ContainerAllocator> & v) 
  {
    s << indent << "theta_index[]" << std::endl;
    for (size_t i = 0; i < v.theta_index.size(); ++i)
    {
      s << indent << "  theta_index[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.theta_index[i]);
    }
    s << indent << "est_rho[]" << std::endl;
    for (size_t i = 0; i < v.est_rho.size(); ++i)
    {
      s << indent << "  est_rho[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.est_rho[i]);
    }
    s << indent << "est_theta[]" << std::endl;
    for (size_t i = 0; i < v.est_theta.size(); ++i)
    {
      s << indent << "  est_theta[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.est_theta[i]);
    }
    s << indent << "delta_rho[]" << std::endl;
    for (size_t i = 0; i < v.delta_rho.size(); ++i)
    {
      s << indent << "  delta_rho[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.delta_rho[i]);
    }
    s << indent << "endpoints[]" << std::endl;
    for (size_t i = 0; i < v.endpoints.size(); ++i)
    {
      s << indent << "  endpoints[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Polygon_<ContainerAllocator> >::stream(s, indent + "    ", v.endpoints[i]);
    }
    s << indent << "lengths[]" << std::endl;
    for (size_t i = 0; i < v.lengths.size(); ++i)
    {
      s << indent << "  lengths[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Polygon_<ContainerAllocator> >::stream(s, indent + "    ", v.lengths[i]);
    }
    s << indent << "endpoint_ranges[]" << std::endl;
    for (size_t i = 0; i < v.endpoint_ranges.size(); ++i)
    {
      s << indent << "  endpoint_ranges[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Polygon_<ContainerAllocator> >::stream(s, indent + "    ", v.endpoint_ranges[i]);
    }
    s << indent << "theta[]" << std::endl;
    for (size_t i = 0; i < v.theta.size(); ++i)
    {
      s << indent << "  theta[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Polygon_<ContainerAllocator> >::stream(s, indent + "    ", v.theta[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // ART_LRF_MESSAGE_LINES_H

