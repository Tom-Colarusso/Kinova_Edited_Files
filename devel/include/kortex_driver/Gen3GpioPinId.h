// Generated by gencpp from file kortex_driver/Gen3GpioPinId.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GEN3GPIOPINID_H
#define KORTEX_DRIVER_MESSAGE_GEN3GPIOPINID_H


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
struct Gen3GpioPinId_
{
  typedef Gen3GpioPinId_<ContainerAllocator> Type;

  Gen3GpioPinId_()
    {
    }
  Gen3GpioPinId_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }





// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(UNSPECIFIED_PIN)
  #undef UNSPECIFIED_PIN
#endif
#if defined(_WIN32) && defined(GPIO_PIN_B)
  #undef GPIO_PIN_B
#endif
#if defined(_WIN32) && defined(GPIO_PIN_C)
  #undef GPIO_PIN_C
#endif
#if defined(_WIN32) && defined(GPIO_PIN_D)
  #undef GPIO_PIN_D
#endif
#if defined(_WIN32) && defined(GPIO_PIN_E)
  #undef GPIO_PIN_E
#endif
#if defined(_WIN32) && defined(GPIO_PIN_G)
  #undef GPIO_PIN_G
#endif
#if defined(_WIN32) && defined(GPIO_PIN_H)
  #undef GPIO_PIN_H
#endif
#if defined(_WIN32) && defined(GPIO_PIN_I)
  #undef GPIO_PIN_I
#endif
#if defined(_WIN32) && defined(GPIO_PIN_K)
  #undef GPIO_PIN_K
#endif
#if defined(_WIN32) && defined(GPIO_PIN_N)
  #undef GPIO_PIN_N
#endif
#if defined(_WIN32) && defined(GPIO_PIN_O)
  #undef GPIO_PIN_O
#endif
#if defined(_WIN32) && defined(GPIO_PIN_S)
  #undef GPIO_PIN_S
#endif
#if defined(_WIN32) && defined(GPIO_PIN_T)
  #undef GPIO_PIN_T
#endif

  enum {
    UNSPECIFIED_PIN = 0u,
    GPIO_PIN_B = 1u,
    GPIO_PIN_C = 2u,
    GPIO_PIN_D = 3u,
    GPIO_PIN_E = 4u,
    GPIO_PIN_G = 5u,
    GPIO_PIN_H = 6u,
    GPIO_PIN_I = 7u,
    GPIO_PIN_K = 8u,
    GPIO_PIN_N = 9u,
    GPIO_PIN_O = 10u,
    GPIO_PIN_S = 11u,
    GPIO_PIN_T = 12u,
  };


  typedef boost::shared_ptr< ::kortex_driver::Gen3GpioPinId_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::Gen3GpioPinId_<ContainerAllocator> const> ConstPtr;

}; // struct Gen3GpioPinId_

typedef ::kortex_driver::Gen3GpioPinId_<std::allocator<void> > Gen3GpioPinId;

typedef boost::shared_ptr< ::kortex_driver::Gen3GpioPinId > Gen3GpioPinIdPtr;
typedef boost::shared_ptr< ::kortex_driver::Gen3GpioPinId const> Gen3GpioPinIdConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::Gen3GpioPinId_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::Gen3GpioPinId_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::Gen3GpioPinId_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::Gen3GpioPinId_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::Gen3GpioPinId_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::Gen3GpioPinId_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::Gen3GpioPinId_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::Gen3GpioPinId_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::Gen3GpioPinId_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e4129f3451b6d28fcb9b382446ece1a9";
  }

  static const char* value(const ::kortex_driver::Gen3GpioPinId_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe4129f3451b6d28fULL;
  static const uint64_t static_value2 = 0xcb9b382446ece1a9ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::Gen3GpioPinId_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/Gen3GpioPinId";
  }

  static const char* value(const ::kortex_driver::Gen3GpioPinId_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::Gen3GpioPinId_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 UNSPECIFIED_PIN = 0\n"
"\n"
"uint32 GPIO_PIN_B = 1\n"
"\n"
"uint32 GPIO_PIN_C = 2\n"
"\n"
"uint32 GPIO_PIN_D = 3\n"
"\n"
"uint32 GPIO_PIN_E = 4\n"
"\n"
"uint32 GPIO_PIN_G = 5\n"
"\n"
"uint32 GPIO_PIN_H = 6\n"
"\n"
"uint32 GPIO_PIN_I = 7\n"
"\n"
"uint32 GPIO_PIN_K = 8\n"
"\n"
"uint32 GPIO_PIN_N = 9\n"
"\n"
"uint32 GPIO_PIN_O = 10\n"
"\n"
"uint32 GPIO_PIN_S = 11\n"
"\n"
"uint32 GPIO_PIN_T = 12\n"
;
  }

  static const char* value(const ::kortex_driver::Gen3GpioPinId_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::Gen3GpioPinId_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Gen3GpioPinId_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::Gen3GpioPinId_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::kortex_driver::Gen3GpioPinId_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GEN3GPIOPINID_H
