// Generated by gencpp from file kortex_driver/TwistAngularSoftLimit.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_TWISTANGULARSOFTLIMIT_H
#define KORTEX_DRIVER_MESSAGE_TWISTANGULARSOFTLIMIT_H


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
struct TwistAngularSoftLimit_
{
  typedef TwistAngularSoftLimit_<ContainerAllocator> Type;

  TwistAngularSoftLimit_()
    : control_mode(0)
    , twist_angular_soft_limit(0.0)  {
    }
  TwistAngularSoftLimit_(const ContainerAllocator& _alloc)
    : control_mode(0)
    , twist_angular_soft_limit(0.0)  {
  (void)_alloc;
    }



   typedef uint32_t _control_mode_type;
  _control_mode_type control_mode;

   typedef float _twist_angular_soft_limit_type;
  _twist_angular_soft_limit_type twist_angular_soft_limit;





  typedef boost::shared_ptr< ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator> const> ConstPtr;

}; // struct TwistAngularSoftLimit_

typedef ::kortex_driver::TwistAngularSoftLimit_<std::allocator<void> > TwistAngularSoftLimit;

typedef boost::shared_ptr< ::kortex_driver::TwistAngularSoftLimit > TwistAngularSoftLimitPtr;
typedef boost::shared_ptr< ::kortex_driver::TwistAngularSoftLimit const> TwistAngularSoftLimitConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator1> & lhs, const ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator2> & rhs)
{
  return lhs.control_mode == rhs.control_mode &&
    lhs.twist_angular_soft_limit == rhs.twist_angular_soft_limit;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator1> & lhs, const ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ec20d4ed4983e99d6172917731a2eee5";
  }

  static const char* value(const ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xec20d4ed4983e99dULL;
  static const uint64_t static_value2 = 0x6172917731a2eee5ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/TwistAngularSoftLimit";
  }

  static const char* value(const ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 control_mode\n"
"float32 twist_angular_soft_limit\n"
;
  }

  static const char* value(const ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.control_mode);
      stream.next(m.twist_angular_soft_limit);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TwistAngularSoftLimit_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::TwistAngularSoftLimit_<ContainerAllocator>& v)
  {
    s << indent << "control_mode: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.control_mode);
    s << indent << "twist_angular_soft_limit: ";
    Printer<float>::stream(s, indent + "  ", v.twist_angular_soft_limit);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_TWISTANGULARSOFTLIMIT_H
