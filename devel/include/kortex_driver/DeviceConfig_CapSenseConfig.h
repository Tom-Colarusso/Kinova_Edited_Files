// Generated by gencpp from file kortex_driver/DeviceConfig_CapSenseConfig.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_DEVICECONFIG_CAPSENSECONFIG_H
#define KORTEX_DRIVER_MESSAGE_DEVICECONFIG_CAPSENSECONFIG_H


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
struct DeviceConfig_CapSenseConfig_
{
  typedef DeviceConfig_CapSenseConfig_<ContainerAllocator> Type;

  DeviceConfig_CapSenseConfig_()
    : mode(0)
    , threshold_a(0.0)
    , threshold_b(0.0)  {
    }
  DeviceConfig_CapSenseConfig_(const ContainerAllocator& _alloc)
    : mode(0)
    , threshold_a(0.0)
    , threshold_b(0.0)  {
  (void)_alloc;
    }



   typedef uint32_t _mode_type;
  _mode_type mode;

   typedef float _threshold_a_type;
  _threshold_a_type threshold_a;

   typedef float _threshold_b_type;
  _threshold_b_type threshold_b;





  typedef boost::shared_ptr< ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator> const> ConstPtr;

}; // struct DeviceConfig_CapSenseConfig_

typedef ::kortex_driver::DeviceConfig_CapSenseConfig_<std::allocator<void> > DeviceConfig_CapSenseConfig;

typedef boost::shared_ptr< ::kortex_driver::DeviceConfig_CapSenseConfig > DeviceConfig_CapSenseConfigPtr;
typedef boost::shared_ptr< ::kortex_driver::DeviceConfig_CapSenseConfig const> DeviceConfig_CapSenseConfigConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator1> & lhs, const ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator2> & rhs)
{
  return lhs.mode == rhs.mode &&
    lhs.threshold_a == rhs.threshold_a &&
    lhs.threshold_b == rhs.threshold_b;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator1> & lhs, const ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bd34e36d2242f94fe2b0925f57d7bdbd";
  }

  static const char* value(const ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbd34e36d2242f94fULL;
  static const uint64_t static_value2 = 0xe2b0925f57d7bdbdULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/DeviceConfig_CapSenseConfig";
  }

  static const char* value(const ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 mode\n"
"float32 threshold_a\n"
"float32 threshold_b\n"
;
  }

  static const char* value(const ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mode);
      stream.next(m.threshold_a);
      stream.next(m.threshold_b);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DeviceConfig_CapSenseConfig_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::DeviceConfig_CapSenseConfig_<ContainerAllocator>& v)
  {
    s << indent << "mode: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.mode);
    s << indent << "threshold_a: ";
    Printer<float>::stream(s, indent + "  ", v.threshold_a);
    s << indent << "threshold_b: ";
    Printer<float>::stream(s, indent + "  ", v.threshold_b);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_DEVICECONFIG_CAPSENSECONFIG_H