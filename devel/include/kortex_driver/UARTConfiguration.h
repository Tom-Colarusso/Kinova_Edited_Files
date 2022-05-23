// Generated by gencpp from file kortex_driver/UARTConfiguration.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_UARTCONFIGURATION_H
#define KORTEX_DRIVER_MESSAGE_UARTCONFIGURATION_H


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
struct UARTConfiguration_
{
  typedef UARTConfiguration_<ContainerAllocator> Type;

  UARTConfiguration_()
    : port_id(0)
    , enabled(false)
    , speed(0)
    , word_length(0)
    , stop_bits(0)
    , parity(0)  {
    }
  UARTConfiguration_(const ContainerAllocator& _alloc)
    : port_id(0)
    , enabled(false)
    , speed(0)
    , word_length(0)
    , stop_bits(0)
    , parity(0)  {
  (void)_alloc;
    }



   typedef uint32_t _port_id_type;
  _port_id_type port_id;

   typedef uint8_t _enabled_type;
  _enabled_type enabled;

   typedef uint32_t _speed_type;
  _speed_type speed;

   typedef uint32_t _word_length_type;
  _word_length_type word_length;

   typedef uint32_t _stop_bits_type;
  _stop_bits_type stop_bits;

   typedef uint32_t _parity_type;
  _parity_type parity;





  typedef boost::shared_ptr< ::kortex_driver::UARTConfiguration_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::UARTConfiguration_<ContainerAllocator> const> ConstPtr;

}; // struct UARTConfiguration_

typedef ::kortex_driver::UARTConfiguration_<std::allocator<void> > UARTConfiguration;

typedef boost::shared_ptr< ::kortex_driver::UARTConfiguration > UARTConfigurationPtr;
typedef boost::shared_ptr< ::kortex_driver::UARTConfiguration const> UARTConfigurationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::UARTConfiguration_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::UARTConfiguration_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::UARTConfiguration_<ContainerAllocator1> & lhs, const ::kortex_driver::UARTConfiguration_<ContainerAllocator2> & rhs)
{
  return lhs.port_id == rhs.port_id &&
    lhs.enabled == rhs.enabled &&
    lhs.speed == rhs.speed &&
    lhs.word_length == rhs.word_length &&
    lhs.stop_bits == rhs.stop_bits &&
    lhs.parity == rhs.parity;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::UARTConfiguration_<ContainerAllocator1> & lhs, const ::kortex_driver::UARTConfiguration_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::UARTConfiguration_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::UARTConfiguration_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::UARTConfiguration_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::UARTConfiguration_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::UARTConfiguration_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::UARTConfiguration_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::UARTConfiguration_<ContainerAllocator> >
{
  static const char* value()
  {
    return "da8e919001edf0734e1795af994fc4e9";
  }

  static const char* value(const ::kortex_driver::UARTConfiguration_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xda8e919001edf073ULL;
  static const uint64_t static_value2 = 0x4e1795af994fc4e9ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::UARTConfiguration_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/UARTConfiguration";
  }

  static const char* value(const ::kortex_driver::UARTConfiguration_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::UARTConfiguration_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 port_id\n"
"bool enabled\n"
"uint32 speed\n"
"uint32 word_length\n"
"uint32 stop_bits\n"
"uint32 parity\n"
;
  }

  static const char* value(const ::kortex_driver::UARTConfiguration_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::UARTConfiguration_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.port_id);
      stream.next(m.enabled);
      stream.next(m.speed);
      stream.next(m.word_length);
      stream.next(m.stop_bits);
      stream.next(m.parity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct UARTConfiguration_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::UARTConfiguration_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::UARTConfiguration_<ContainerAllocator>& v)
  {
    s << indent << "port_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.port_id);
    s << indent << "enabled: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.enabled);
    s << indent << "speed: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.speed);
    s << indent << "word_length: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.word_length);
    s << indent << "stop_bits: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.stop_bits);
    s << indent << "parity: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.parity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_UARTCONFIGURATION_H
