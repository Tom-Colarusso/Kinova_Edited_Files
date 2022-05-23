// Generated by gencpp from file kortex_driver/ActuatorConfig_SafetyLimitType.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_ACTUATORCONFIG_SAFETYLIMITTYPE_H
#define KORTEX_DRIVER_MESSAGE_ACTUATORCONFIG_SAFETYLIMITTYPE_H


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
struct ActuatorConfig_SafetyLimitType_
{
  typedef ActuatorConfig_SafetyLimitType_<ContainerAllocator> Type;

  ActuatorConfig_SafetyLimitType_()
    {
    }
  ActuatorConfig_SafetyLimitType_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }





// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(MAXIMAL_LIMIT)
  #undef MAXIMAL_LIMIT
#endif
#if defined(_WIN32) && defined(MINIMAL_LIMIT)
  #undef MINIMAL_LIMIT
#endif

  enum {
    MAXIMAL_LIMIT = 0u,
    MINIMAL_LIMIT = 1u,
  };


  typedef boost::shared_ptr< ::kortex_driver::ActuatorConfig_SafetyLimitType_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::ActuatorConfig_SafetyLimitType_<ContainerAllocator> const> ConstPtr;

}; // struct ActuatorConfig_SafetyLimitType_

typedef ::kortex_driver::ActuatorConfig_SafetyLimitType_<std::allocator<void> > ActuatorConfig_SafetyLimitType;

typedef boost::shared_ptr< ::kortex_driver::ActuatorConfig_SafetyLimitType > ActuatorConfig_SafetyLimitTypePtr;
typedef boost::shared_ptr< ::kortex_driver::ActuatorConfig_SafetyLimitType const> ActuatorConfig_SafetyLimitTypeConstPtr;

// constants requiring out of line definition

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::ActuatorConfig_SafetyLimitType_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::ActuatorConfig_SafetyLimitType_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ActuatorConfig_SafetyLimitType_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ActuatorConfig_SafetyLimitType_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ActuatorConfig_SafetyLimitType_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ActuatorConfig_SafetyLimitType_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ActuatorConfig_SafetyLimitType_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ActuatorConfig_SafetyLimitType_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::ActuatorConfig_SafetyLimitType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2886a0a967849f2573bf0fd798659a83";
  }

  static const char* value(const ::kortex_driver::ActuatorConfig_SafetyLimitType_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2886a0a967849f25ULL;
  static const uint64_t static_value2 = 0x73bf0fd798659a83ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::ActuatorConfig_SafetyLimitType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/ActuatorConfig_SafetyLimitType";
  }

  static const char* value(const ::kortex_driver::ActuatorConfig_SafetyLimitType_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::ActuatorConfig_SafetyLimitType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 MAXIMAL_LIMIT = 0\n"
"\n"
"uint32 MINIMAL_LIMIT = 1\n"
;
  }

  static const char* value(const ::kortex_driver::ActuatorConfig_SafetyLimitType_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::ActuatorConfig_SafetyLimitType_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ActuatorConfig_SafetyLimitType_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::ActuatorConfig_SafetyLimitType_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::kortex_driver::ActuatorConfig_SafetyLimitType_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_ACTUATORCONFIG_SAFETYLIMITTYPE_H
