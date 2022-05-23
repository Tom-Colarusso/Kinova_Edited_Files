// Generated by gencpp from file kortex_driver/WifiInformation.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_WIFIINFORMATION_H
#define KORTEX_DRIVER_MESSAGE_WIFIINFORMATION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/Ssid.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct WifiInformation_
{
  typedef WifiInformation_<ContainerAllocator> Type;

  WifiInformation_()
    : ssid()
    , security_type(0)
    , encryption_type(0)
    , signal_quality(0)
    , signal_strength(0)
    , frequency(0)
    , channel(0)  {
    }
  WifiInformation_(const ContainerAllocator& _alloc)
    : ssid(_alloc)
    , security_type(0)
    , encryption_type(0)
    , signal_quality(0)
    , signal_strength(0)
    , frequency(0)
    , channel(0)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::Ssid_<ContainerAllocator>  _ssid_type;
  _ssid_type ssid;

   typedef uint32_t _security_type_type;
  _security_type_type security_type;

   typedef uint32_t _encryption_type_type;
  _encryption_type_type encryption_type;

   typedef uint32_t _signal_quality_type;
  _signal_quality_type signal_quality;

   typedef int32_t _signal_strength_type;
  _signal_strength_type signal_strength;

   typedef uint32_t _frequency_type;
  _frequency_type frequency;

   typedef uint32_t _channel_type;
  _channel_type channel;





  typedef boost::shared_ptr< ::kortex_driver::WifiInformation_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::WifiInformation_<ContainerAllocator> const> ConstPtr;

}; // struct WifiInformation_

typedef ::kortex_driver::WifiInformation_<std::allocator<void> > WifiInformation;

typedef boost::shared_ptr< ::kortex_driver::WifiInformation > WifiInformationPtr;
typedef boost::shared_ptr< ::kortex_driver::WifiInformation const> WifiInformationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::WifiInformation_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::WifiInformation_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::WifiInformation_<ContainerAllocator1> & lhs, const ::kortex_driver::WifiInformation_<ContainerAllocator2> & rhs)
{
  return lhs.ssid == rhs.ssid &&
    lhs.security_type == rhs.security_type &&
    lhs.encryption_type == rhs.encryption_type &&
    lhs.signal_quality == rhs.signal_quality &&
    lhs.signal_strength == rhs.signal_strength &&
    lhs.frequency == rhs.frequency &&
    lhs.channel == rhs.channel;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::WifiInformation_<ContainerAllocator1> & lhs, const ::kortex_driver::WifiInformation_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::WifiInformation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::WifiInformation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::WifiInformation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::WifiInformation_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::WifiInformation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::WifiInformation_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::WifiInformation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6cd167c73dec6e8468316540111c37c6";
  }

  static const char* value(const ::kortex_driver::WifiInformation_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6cd167c73dec6e84ULL;
  static const uint64_t static_value2 = 0x68316540111c37c6ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::WifiInformation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/WifiInformation";
  }

  static const char* value(const ::kortex_driver::WifiInformation_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::WifiInformation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"Ssid ssid\n"
"uint32 security_type\n"
"uint32 encryption_type\n"
"uint32 signal_quality\n"
"int32 signal_strength\n"
"uint32 frequency\n"
"uint32 channel\n"
"================================================================================\n"
"MSG: kortex_driver/Ssid\n"
"\n"
"string identifier\n"
;
  }

  static const char* value(const ::kortex_driver::WifiInformation_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::WifiInformation_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ssid);
      stream.next(m.security_type);
      stream.next(m.encryption_type);
      stream.next(m.signal_quality);
      stream.next(m.signal_strength);
      stream.next(m.frequency);
      stream.next(m.channel);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct WifiInformation_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::WifiInformation_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::WifiInformation_<ContainerAllocator>& v)
  {
    s << indent << "ssid: ";
    s << std::endl;
    Printer< ::kortex_driver::Ssid_<ContainerAllocator> >::stream(s, indent + "  ", v.ssid);
    s << indent << "security_type: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.security_type);
    s << indent << "encryption_type: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.encryption_type);
    s << indent << "signal_quality: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.signal_quality);
    s << indent << "signal_strength: ";
    Printer<int32_t>::stream(s, indent + "  ", v.signal_strength);
    s << indent << "frequency: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.frequency);
    s << indent << "channel: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.channel);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_WIFIINFORMATION_H
