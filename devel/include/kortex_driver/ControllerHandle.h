// Generated by gencpp from file kortex_driver/ControllerHandle.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_CONTROLLERHANDLE_H
#define KORTEX_DRIVER_MESSAGE_CONTROLLERHANDLE_H


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
struct ControllerHandle_
{
  typedef ControllerHandle_<ContainerAllocator> Type;

  ControllerHandle_()
    : type(0)
    , controller_identifier(0)  {
    }
  ControllerHandle_(const ContainerAllocator& _alloc)
    : type(0)
    , controller_identifier(0)  {
  (void)_alloc;
    }



   typedef uint32_t _type_type;
  _type_type type;

   typedef uint32_t _controller_identifier_type;
  _controller_identifier_type controller_identifier;





  typedef boost::shared_ptr< ::kortex_driver::ControllerHandle_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::ControllerHandle_<ContainerAllocator> const> ConstPtr;

}; // struct ControllerHandle_

typedef ::kortex_driver::ControllerHandle_<std::allocator<void> > ControllerHandle;

typedef boost::shared_ptr< ::kortex_driver::ControllerHandle > ControllerHandlePtr;
typedef boost::shared_ptr< ::kortex_driver::ControllerHandle const> ControllerHandleConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::ControllerHandle_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::ControllerHandle_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::ControllerHandle_<ContainerAllocator1> & lhs, const ::kortex_driver::ControllerHandle_<ContainerAllocator2> & rhs)
{
  return lhs.type == rhs.type &&
    lhs.controller_identifier == rhs.controller_identifier;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::ControllerHandle_<ContainerAllocator1> & lhs, const ::kortex_driver::ControllerHandle_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ControllerHandle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ControllerHandle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ControllerHandle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ControllerHandle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ControllerHandle_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ControllerHandle_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::ControllerHandle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e80ce64bff64a2583fb879d270960edd";
  }

  static const char* value(const ::kortex_driver::ControllerHandle_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe80ce64bff64a258ULL;
  static const uint64_t static_value2 = 0x3fb879d270960eddULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::ControllerHandle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/ControllerHandle";
  }

  static const char* value(const ::kortex_driver::ControllerHandle_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::ControllerHandle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 type\n"
"uint32 controller_identifier\n"
;
  }

  static const char* value(const ::kortex_driver::ControllerHandle_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::ControllerHandle_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.type);
      stream.next(m.controller_identifier);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ControllerHandle_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::ControllerHandle_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::ControllerHandle_<ContainerAllocator>& v)
  {
    s << indent << "type: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.type);
    s << indent << "controller_identifier: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.controller_identifier);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_CONTROLLERHANDLE_H