// Generated by gencpp from file kortex_driver/WrenchMode.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_WRENCHMODE_H
#define KORTEX_DRIVER_MESSAGE_WRENCHMODE_H


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
struct WrenchMode_
{
  typedef WrenchMode_<ContainerAllocator> Type;

  WrenchMode_()
    {
    }
  WrenchMode_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }





// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(UNSPECIFIED_WRENCH_MODE)
  #undef UNSPECIFIED_WRENCH_MODE
#endif
#if defined(_WIN32) && defined(WRENCH_RESTRICTED)
  #undef WRENCH_RESTRICTED
#endif
#if defined(_WIN32) && defined(WRENCH_NORMAL)
  #undef WRENCH_NORMAL
#endif

  enum {
    UNSPECIFIED_WRENCH_MODE = 0u,
    WRENCH_RESTRICTED = 1u,
    WRENCH_NORMAL = 2u,
  };


  typedef boost::shared_ptr< ::kortex_driver::WrenchMode_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::WrenchMode_<ContainerAllocator> const> ConstPtr;

}; // struct WrenchMode_

typedef ::kortex_driver::WrenchMode_<std::allocator<void> > WrenchMode;

typedef boost::shared_ptr< ::kortex_driver::WrenchMode > WrenchModePtr;
typedef boost::shared_ptr< ::kortex_driver::WrenchMode const> WrenchModeConstPtr;

// constants requiring out of line definition

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::WrenchMode_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::WrenchMode_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::WrenchMode_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::WrenchMode_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::WrenchMode_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::WrenchMode_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::WrenchMode_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::WrenchMode_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::WrenchMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "21f4098a76f681f49eb3f39d12684c72";
  }

  static const char* value(const ::kortex_driver::WrenchMode_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x21f4098a76f681f4ULL;
  static const uint64_t static_value2 = 0x9eb3f39d12684c72ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::WrenchMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/WrenchMode";
  }

  static const char* value(const ::kortex_driver::WrenchMode_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::WrenchMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 UNSPECIFIED_WRENCH_MODE = 0\n"
"\n"
"uint32 WRENCH_RESTRICTED = 1\n"
"\n"
"uint32 WRENCH_NORMAL = 2\n"
;
  }

  static const char* value(const ::kortex_driver::WrenchMode_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::WrenchMode_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct WrenchMode_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::WrenchMode_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::kortex_driver::WrenchMode_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_WRENCHMODE_H
