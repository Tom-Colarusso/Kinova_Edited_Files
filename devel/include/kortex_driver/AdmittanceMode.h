// Generated by gencpp from file kortex_driver/AdmittanceMode.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_ADMITTANCEMODE_H
#define KORTEX_DRIVER_MESSAGE_ADMITTANCEMODE_H


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
struct AdmittanceMode_
{
  typedef AdmittanceMode_<ContainerAllocator> Type;

  AdmittanceMode_()
    {
    }
  AdmittanceMode_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }





// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(UNSPECIFIED_ADMITTANCE_MODE)
  #undef UNSPECIFIED_ADMITTANCE_MODE
#endif
#if defined(_WIN32) && defined(CARTESIAN)
  #undef CARTESIAN
#endif
#if defined(_WIN32) && defined(JOINT)
  #undef JOINT
#endif
#if defined(_WIN32) && defined(NULL_SPACE)
  #undef NULL_SPACE
#endif
#if defined(_WIN32) && defined(DISABLED)
  #undef DISABLED
#endif

  enum {
    UNSPECIFIED_ADMITTANCE_MODE = 0u,
    CARTESIAN = 1u,
    JOINT = 2u,
    NULL_SPACE = 3u,
    DISABLED = 4u,
  };


  typedef boost::shared_ptr< ::kortex_driver::AdmittanceMode_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::AdmittanceMode_<ContainerAllocator> const> ConstPtr;

}; // struct AdmittanceMode_

typedef ::kortex_driver::AdmittanceMode_<std::allocator<void> > AdmittanceMode;

typedef boost::shared_ptr< ::kortex_driver::AdmittanceMode > AdmittanceModePtr;
typedef boost::shared_ptr< ::kortex_driver::AdmittanceMode const> AdmittanceModeConstPtr;

// constants requiring out of line definition

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::AdmittanceMode_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::AdmittanceMode_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::AdmittanceMode_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::AdmittanceMode_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::AdmittanceMode_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::AdmittanceMode_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::AdmittanceMode_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::AdmittanceMode_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::AdmittanceMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1a52f2d7eb92e509117483a6b9c79ce1";
  }

  static const char* value(const ::kortex_driver::AdmittanceMode_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1a52f2d7eb92e509ULL;
  static const uint64_t static_value2 = 0x117483a6b9c79ce1ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::AdmittanceMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/AdmittanceMode";
  }

  static const char* value(const ::kortex_driver::AdmittanceMode_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::AdmittanceMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 UNSPECIFIED_ADMITTANCE_MODE = 0\n"
"\n"
"uint32 CARTESIAN = 1\n"
"\n"
"uint32 JOINT = 2\n"
"\n"
"uint32 NULL_SPACE = 3\n"
"\n"
"uint32 DISABLED = 4\n"
;
  }

  static const char* value(const ::kortex_driver::AdmittanceMode_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::AdmittanceMode_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AdmittanceMode_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::AdmittanceMode_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::kortex_driver::AdmittanceMode_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_ADMITTANCEMODE_H
