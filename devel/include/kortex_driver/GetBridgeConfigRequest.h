// Generated by gencpp from file kortex_driver/GetBridgeConfigRequest.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETBRIDGECONFIGREQUEST_H
#define KORTEX_DRIVER_MESSAGE_GETBRIDGECONFIGREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/BridgeIdentifier.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct GetBridgeConfigRequest_
{
  typedef GetBridgeConfigRequest_<ContainerAllocator> Type;

  GetBridgeConfigRequest_()
    : input()  {
    }
  GetBridgeConfigRequest_(const ContainerAllocator& _alloc)
    : input(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::BridgeIdentifier_<ContainerAllocator>  _input_type;
  _input_type input;





  typedef boost::shared_ptr< ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetBridgeConfigRequest_

typedef ::kortex_driver::GetBridgeConfigRequest_<std::allocator<void> > GetBridgeConfigRequest;

typedef boost::shared_ptr< ::kortex_driver::GetBridgeConfigRequest > GetBridgeConfigRequestPtr;
typedef boost::shared_ptr< ::kortex_driver::GetBridgeConfigRequest const> GetBridgeConfigRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator2> & rhs)
{
  return lhs.input == rhs.input;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e7fff85c040dee9f9ad9fd35ed6100dc";
  }

  static const char* value(const ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe7fff85c040dee9fULL;
  static const uint64_t static_value2 = 0x9ad9fd35ed6100dcULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/GetBridgeConfigRequest";
  }

  static const char* value(const ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "BridgeIdentifier input\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/BridgeIdentifier\n"
"\n"
"uint32 bridge_id\n"
;
  }

  static const char* value(const ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.input);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetBridgeConfigRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::GetBridgeConfigRequest_<ContainerAllocator>& v)
  {
    s << indent << "input: ";
    s << std::endl;
    Printer< ::kortex_driver::BridgeIdentifier_<ContainerAllocator> >::stream(s, indent + "  ", v.input);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETBRIDGECONFIGREQUEST_H
