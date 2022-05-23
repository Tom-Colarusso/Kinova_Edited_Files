// Generated by gencpp from file kortex_driver/GPIOMode.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GPIOMODE_H
#define KORTEX_DRIVER_MESSAGE_GPIOMODE_H


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
struct GPIOMode_
{
  typedef GPIOMode_<ContainerAllocator> Type;

  GPIOMode_()
    {
    }
  GPIOMode_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }





// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(GPIO_MODE_UNSPECIFIED)
  #undef GPIO_MODE_UNSPECIFIED
#endif
#if defined(_WIN32) && defined(GPIO_MODE_INPUT_FLOATING)
  #undef GPIO_MODE_INPUT_FLOATING
#endif
#if defined(_WIN32) && defined(GPIO_MODE_OUTPUT_PUSH_PULL)
  #undef GPIO_MODE_OUTPUT_PUSH_PULL
#endif
#if defined(_WIN32) && defined(GPIO_MODE_OUTPUT_OPEN_DRAIN)
  #undef GPIO_MODE_OUTPUT_OPEN_DRAIN
#endif

  enum {
    GPIO_MODE_UNSPECIFIED = 0u,
    GPIO_MODE_INPUT_FLOATING = 1u,
    GPIO_MODE_OUTPUT_PUSH_PULL = 2u,
    GPIO_MODE_OUTPUT_OPEN_DRAIN = 3u,
  };


  typedef boost::shared_ptr< ::kortex_driver::GPIOMode_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::GPIOMode_<ContainerAllocator> const> ConstPtr;

}; // struct GPIOMode_

typedef ::kortex_driver::GPIOMode_<std::allocator<void> > GPIOMode;

typedef boost::shared_ptr< ::kortex_driver::GPIOMode > GPIOModePtr;
typedef boost::shared_ptr< ::kortex_driver::GPIOMode const> GPIOModeConstPtr;

// constants requiring out of line definition

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::GPIOMode_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::GPIOMode_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GPIOMode_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GPIOMode_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GPIOMode_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GPIOMode_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GPIOMode_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GPIOMode_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::GPIOMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "67dcb56ddb7c7d3c3c961e401c8325a4";
  }

  static const char* value(const ::kortex_driver::GPIOMode_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x67dcb56ddb7c7d3cULL;
  static const uint64_t static_value2 = 0x3c961e401c8325a4ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::GPIOMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/GPIOMode";
  }

  static const char* value(const ::kortex_driver::GPIOMode_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::GPIOMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 GPIO_MODE_UNSPECIFIED = 0\n"
"\n"
"uint32 GPIO_MODE_INPUT_FLOATING = 1\n"
"\n"
"uint32 GPIO_MODE_OUTPUT_PUSH_PULL = 2\n"
"\n"
"uint32 GPIO_MODE_OUTPUT_OPEN_DRAIN = 3\n"
;
  }

  static const char* value(const ::kortex_driver::GPIOMode_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::GPIOMode_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GPIOMode_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::GPIOMode_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::kortex_driver::GPIOMode_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GPIOMODE_H
