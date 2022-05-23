// Generated by gencpp from file kortex_driver/Pose.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_POSE_H
#define KORTEX_DRIVER_MESSAGE_POSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace kortex_driver
{
template <class ContainerAllocator>
struct Pose_
{
  typedef Pose_<ContainerAllocator> Type;

  Pose_()
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , theta_x(0.0)
    , theta_y(0.0)
    , theta_z(0.0)  {
    }
  Pose_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , theta_x(0.0)
    , theta_y(0.0)
    , theta_z(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;

   typedef float _theta_x_type;
  _theta_x_type theta_x;

   typedef float _theta_y_type;
  _theta_y_type theta_y;

   typedef float _theta_z_type;
  _theta_z_type theta_z;





  typedef boost::shared_ptr< ::kortex_driver::Pose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::Pose_<ContainerAllocator> const> ConstPtr;

}; // struct Pose_

typedef ::kortex_driver::Pose_<std::allocator<void> > Pose;

typedef boost::shared_ptr< ::kortex_driver::Pose > PosePtr;
typedef boost::shared_ptr< ::kortex_driver::Pose const> PoseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::Pose_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::Pose_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::Pose_<ContainerAllocator1> & lhs, const ::kortex_driver::Pose_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z &&
    lhs.theta_x == rhs.theta_x &&
    lhs.theta_y == rhs.theta_y &&
    lhs.theta_z == rhs.theta_z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::Pose_<ContainerAllocator1> & lhs, const ::kortex_driver::Pose_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::Pose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::Pose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::Pose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::Pose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::Pose_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::Pose_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::Pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b01742cd81703ae7d171d91df9eaf69e";
  }

  static const char* value(const ::kortex_driver::Pose_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb01742cd81703ae7ULL;
  static const uint64_t static_value2 = 0xd171d91df9eaf69eULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::Pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/Pose";
  }

  static const char* value(const ::kortex_driver::Pose_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::Pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
"float32 theta_x\n"
"float32 theta_y\n"
"float32 theta_z\n"
;
  }

  static const char* value(const ::kortex_driver::Pose_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::Pose_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.theta_x);
      stream.next(m.theta_y);
      stream.next(m.theta_z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Pose_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::Pose_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::Pose_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
    s << indent << "theta_x: ";
    Printer<float>::stream(s, indent + "  ", v.theta_x);
    s << indent << "theta_y: ";
    Printer<float>::stream(s, indent + "  ", v.theta_y);
    s << indent << "theta_z: ";
    Printer<float>::stream(s, indent + "  ", v.theta_z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_POSE_H
