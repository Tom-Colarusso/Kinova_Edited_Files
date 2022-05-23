// Generated by gencpp from file kortex_driver/BridgePortConfig.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_BRIDGEPORTCONFIG_H
#define KORTEX_DRIVER_MESSAGE_BRIDGEPORTCONFIG_H


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
struct BridgePortConfig_
{
  typedef BridgePortConfig_<ContainerAllocator> Type;

  BridgePortConfig_()
    : target_port(0)
    , out_port(0)  {
    }
  BridgePortConfig_(const ContainerAllocator& _alloc)
    : target_port(0)
    , out_port(0)  {
  (void)_alloc;
    }



   typedef uint32_t _target_port_type;
  _target_port_type target_port;

   typedef uint32_t _out_port_type;
  _out_port_type out_port;





  typedef boost::shared_ptr< ::kortex_driver::BridgePortConfig_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::BridgePortConfig_<ContainerAllocator> const> ConstPtr;

}; // struct BridgePortConfig_

typedef ::kortex_driver::BridgePortConfig_<std::allocator<void> > BridgePortConfig;

typedef boost::shared_ptr< ::kortex_driver::BridgePortConfig > BridgePortConfigPtr;
typedef boost::shared_ptr< ::kortex_driver::BridgePortConfig const> BridgePortConfigConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::BridgePortConfig_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::BridgePortConfig_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::BridgePortConfig_<ContainerAllocator1> & lhs, const ::kortex_driver::BridgePortConfig_<ContainerAllocator2> & rhs)
{
  return lhs.target_port == rhs.target_port &&
    lhs.out_port == rhs.out_port;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::BridgePortConfig_<ContainerAllocator1> & lhs, const ::kortex_driver::BridgePortConfig_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::BridgePortConfig_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::BridgePortConfig_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::BridgePortConfig_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::BridgePortConfig_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::BridgePortConfig_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::BridgePortConfig_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::BridgePortConfig_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c2591c5a923ec65c5c9c9b1eb43de273";
  }

  static const char* value(const ::kortex_driver::BridgePortConfig_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc2591c5a923ec65cULL;
  static const uint64_t static_value2 = 0x5c9c9b1eb43de273ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::BridgePortConfig_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/BridgePortConfig";
  }

  static const char* value(const ::kortex_driver::BridgePortConfig_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::BridgePortConfig_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 target_port\n"
"uint32 out_port\n"
;
  }

  static const char* value(const ::kortex_driver::BridgePortConfig_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::BridgePortConfig_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.target_port);
      stream.next(m.out_port);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BridgePortConfig_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::BridgePortConfig_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::BridgePortConfig_<ContainerAllocator>& v)
  {
    s << indent << "target_port: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.target_port);
    s << indent << "out_port: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.out_port);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_BRIDGEPORTCONFIG_H
