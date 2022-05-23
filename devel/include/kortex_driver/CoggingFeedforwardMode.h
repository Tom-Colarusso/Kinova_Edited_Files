// Generated by gencpp from file kortex_driver/CoggingFeedforwardMode.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_COGGINGFEEDFORWARDMODE_H
#define KORTEX_DRIVER_MESSAGE_COGGINGFEEDFORWARDMODE_H


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
struct CoggingFeedforwardMode_
{
  typedef CoggingFeedforwardMode_<ContainerAllocator> Type;

  CoggingFeedforwardMode_()
    {
    }
  CoggingFeedforwardMode_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }





// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(FEEDFORWARD_OFF)
  #undef FEEDFORWARD_OFF
#endif
#if defined(_WIN32) && defined(FEEDFORWARD_ADAPTIVE)
  #undef FEEDFORWARD_ADAPTIVE
#endif
#if defined(_WIN32) && defined(FEEDFORWARD_CALIBRATED)
  #undef FEEDFORWARD_CALIBRATED
#endif

  enum {
    FEEDFORWARD_OFF = 0u,
    FEEDFORWARD_ADAPTIVE = 1u,
    FEEDFORWARD_CALIBRATED = 2u,
  };


  typedef boost::shared_ptr< ::kortex_driver::CoggingFeedforwardMode_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::CoggingFeedforwardMode_<ContainerAllocator> const> ConstPtr;

}; // struct CoggingFeedforwardMode_

typedef ::kortex_driver::CoggingFeedforwardMode_<std::allocator<void> > CoggingFeedforwardMode;

typedef boost::shared_ptr< ::kortex_driver::CoggingFeedforwardMode > CoggingFeedforwardModePtr;
typedef boost::shared_ptr< ::kortex_driver::CoggingFeedforwardMode const> CoggingFeedforwardModeConstPtr;

// constants requiring out of line definition

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::CoggingFeedforwardMode_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::CoggingFeedforwardMode_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::CoggingFeedforwardMode_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::CoggingFeedforwardMode_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::CoggingFeedforwardMode_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::CoggingFeedforwardMode_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::CoggingFeedforwardMode_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::CoggingFeedforwardMode_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::CoggingFeedforwardMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "040e60ae33515a7c8082e377632750e6";
  }

  static const char* value(const ::kortex_driver::CoggingFeedforwardMode_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x040e60ae33515a7cULL;
  static const uint64_t static_value2 = 0x8082e377632750e6ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::CoggingFeedforwardMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/CoggingFeedforwardMode";
  }

  static const char* value(const ::kortex_driver::CoggingFeedforwardMode_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::CoggingFeedforwardMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 FEEDFORWARD_OFF = 0\n"
"\n"
"uint32 FEEDFORWARD_ADAPTIVE = 1\n"
"\n"
"uint32 FEEDFORWARD_CALIBRATED = 2\n"
;
  }

  static const char* value(const ::kortex_driver::CoggingFeedforwardMode_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::CoggingFeedforwardMode_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CoggingFeedforwardMode_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::CoggingFeedforwardMode_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::kortex_driver::CoggingFeedforwardMode_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_COGGINGFEEDFORWARDMODE_H