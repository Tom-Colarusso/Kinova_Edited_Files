// Generated by gencpp from file kortex_driver/Servoing.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_SERVOING_H
#define KORTEX_DRIVER_MESSAGE_SERVOING_H


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
struct Servoing_
{
  typedef Servoing_<ContainerAllocator> Type;

  Servoing_()
    : enabled(false)  {
    }
  Servoing_(const ContainerAllocator& _alloc)
    : enabled(false)  {
  (void)_alloc;
    }



   typedef uint8_t _enabled_type;
  _enabled_type enabled;





  typedef boost::shared_ptr< ::kortex_driver::Servoing_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::Servoing_<ContainerAllocator> const> ConstPtr;

}; // struct Servoing_

typedef ::kortex_driver::Servoing_<std::allocator<void> > Servoing;

typedef boost::shared_ptr< ::kortex_driver::Servoing > ServoingPtr;
typedef boost::shared_ptr< ::kortex_driver::Servoing const> ServoingConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::Servoing_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::Servoing_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::Servoing_<ContainerAllocator1> & lhs, const ::kortex_driver::Servoing_<ContainerAllocator2> & rhs)
{
  return lhs.enabled == rhs.enabled;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::Servoing_<ContainerAllocator1> & lhs, const ::kortex_driver::Servoing_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::Servoing_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::Servoing_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::Servoing_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::Servoing_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::Servoing_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::Servoing_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::Servoing_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2815464f55ab63684cc1bc38072d0b9b";
  }

  static const char* value(const ::kortex_driver::Servoing_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2815464f55ab6368ULL;
  static const uint64_t static_value2 = 0x4cc1bc38072d0b9bULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::Servoing_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/Servoing";
  }

  static const char* value(const ::kortex_driver::Servoing_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::Servoing_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"bool enabled\n"
;
  }

  static const char* value(const ::kortex_driver::Servoing_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::Servoing_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.enabled);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Servoing_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::Servoing_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::Servoing_<ContainerAllocator>& v)
  {
    s << indent << "enabled: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.enabled);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_SERVOING_H