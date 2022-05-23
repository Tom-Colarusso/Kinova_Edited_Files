// Generated by gencpp from file kortex_driver/BridgeList.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_BRIDGELIST_H
#define KORTEX_DRIVER_MESSAGE_BRIDGELIST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/BridgeConfig.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct BridgeList_
{
  typedef BridgeList_<ContainerAllocator> Type;

  BridgeList_()
    : bridgeConfig()  {
    }
  BridgeList_(const ContainerAllocator& _alloc)
    : bridgeConfig(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::kortex_driver::BridgeConfig_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::kortex_driver::BridgeConfig_<ContainerAllocator> >::other >  _bridgeConfig_type;
  _bridgeConfig_type bridgeConfig;





  typedef boost::shared_ptr< ::kortex_driver::BridgeList_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::BridgeList_<ContainerAllocator> const> ConstPtr;

}; // struct BridgeList_

typedef ::kortex_driver::BridgeList_<std::allocator<void> > BridgeList;

typedef boost::shared_ptr< ::kortex_driver::BridgeList > BridgeListPtr;
typedef boost::shared_ptr< ::kortex_driver::BridgeList const> BridgeListConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::BridgeList_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::BridgeList_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::BridgeList_<ContainerAllocator1> & lhs, const ::kortex_driver::BridgeList_<ContainerAllocator2> & rhs)
{
  return lhs.bridgeConfig == rhs.bridgeConfig;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::BridgeList_<ContainerAllocator1> & lhs, const ::kortex_driver::BridgeList_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::BridgeList_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::BridgeList_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::BridgeList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::BridgeList_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::BridgeList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::BridgeList_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::BridgeList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "67edf56b8cf474f747335e24413448f0";
  }

  static const char* value(const ::kortex_driver::BridgeList_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x67edf56b8cf474f7ULL;
  static const uint64_t static_value2 = 0x47335e24413448f0ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::BridgeList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/BridgeList";
  }

  static const char* value(const ::kortex_driver::BridgeList_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::BridgeList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"BridgeConfig[] bridgeConfig\n"
"================================================================================\n"
"MSG: kortex_driver/BridgeConfig\n"
"\n"
"uint32 device_identifier\n"
"uint32 bridgetype\n"
"BridgePortConfig port_config\n"
"BridgeIdentifier bridge_id\n"
"================================================================================\n"
"MSG: kortex_driver/BridgePortConfig\n"
"\n"
"uint32 target_port\n"
"uint32 out_port\n"
"================================================================================\n"
"MSG: kortex_driver/BridgeIdentifier\n"
"\n"
"uint32 bridge_id\n"
;
  }

  static const char* value(const ::kortex_driver::BridgeList_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::BridgeList_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.bridgeConfig);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BridgeList_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::BridgeList_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::BridgeList_<ContainerAllocator>& v)
  {
    s << indent << "bridgeConfig[]" << std::endl;
    for (size_t i = 0; i < v.bridgeConfig.size(); ++i)
    {
      s << indent << "  bridgeConfig[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::kortex_driver::BridgeConfig_<ContainerAllocator> >::stream(s, indent + "    ", v.bridgeConfig[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_BRIDGELIST_H
