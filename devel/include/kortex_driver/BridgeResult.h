// Generated by gencpp from file kortex_driver/BridgeResult.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_BRIDGERESULT_H
#define KORTEX_DRIVER_MESSAGE_BRIDGERESULT_H


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
struct BridgeResult_
{
  typedef BridgeResult_<ContainerAllocator> Type;

  BridgeResult_()
    : bridge_id()
    , status(0)  {
    }
  BridgeResult_(const ContainerAllocator& _alloc)
    : bridge_id(_alloc)
    , status(0)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::BridgeIdentifier_<ContainerAllocator>  _bridge_id_type;
  _bridge_id_type bridge_id;

   typedef uint32_t _status_type;
  _status_type status;





  typedef boost::shared_ptr< ::kortex_driver::BridgeResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::BridgeResult_<ContainerAllocator> const> ConstPtr;

}; // struct BridgeResult_

typedef ::kortex_driver::BridgeResult_<std::allocator<void> > BridgeResult;

typedef boost::shared_ptr< ::kortex_driver::BridgeResult > BridgeResultPtr;
typedef boost::shared_ptr< ::kortex_driver::BridgeResult const> BridgeResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::BridgeResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::BridgeResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::BridgeResult_<ContainerAllocator1> & lhs, const ::kortex_driver::BridgeResult_<ContainerAllocator2> & rhs)
{
  return lhs.bridge_id == rhs.bridge_id &&
    lhs.status == rhs.status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::BridgeResult_<ContainerAllocator1> & lhs, const ::kortex_driver::BridgeResult_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::BridgeResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::BridgeResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::BridgeResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::BridgeResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::BridgeResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::BridgeResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::BridgeResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c86e99efcef2f596ae80430e65714cbc";
  }

  static const char* value(const ::kortex_driver::BridgeResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc86e99efcef2f596ULL;
  static const uint64_t static_value2 = 0xae80430e65714cbcULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::BridgeResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/BridgeResult";
  }

  static const char* value(const ::kortex_driver::BridgeResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::BridgeResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"BridgeIdentifier bridge_id\n"
"uint32 status\n"
"================================================================================\n"
"MSG: kortex_driver/BridgeIdentifier\n"
"\n"
"uint32 bridge_id\n"
;
  }

  static const char* value(const ::kortex_driver::BridgeResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::BridgeResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.bridge_id);
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BridgeResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::BridgeResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::BridgeResult_<ContainerAllocator>& v)
  {
    s << indent << "bridge_id: ";
    s << std::endl;
    Printer< ::kortex_driver::BridgeIdentifier_<ContainerAllocator> >::stream(s, indent + "  ", v.bridge_id);
    s << indent << "status: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_BRIDGERESULT_H
