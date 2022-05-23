// Generated by gencpp from file kortex_driver/ResetTwistAngularSoftLimitRequest.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_RESETTWISTANGULARSOFTLIMITREQUEST_H
#define KORTEX_DRIVER_MESSAGE_RESETTWISTANGULARSOFTLIMITREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/ControlConfig_ControlModeInformation.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct ResetTwistAngularSoftLimitRequest_
{
  typedef ResetTwistAngularSoftLimitRequest_<ContainerAllocator> Type;

  ResetTwistAngularSoftLimitRequest_()
    : input()  {
    }
  ResetTwistAngularSoftLimitRequest_(const ContainerAllocator& _alloc)
    : input(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::ControlConfig_ControlModeInformation_<ContainerAllocator>  _input_type;
  _input_type input;





  typedef boost::shared_ptr< ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator> const> ConstPtr;

}; // struct ResetTwistAngularSoftLimitRequest_

typedef ::kortex_driver::ResetTwistAngularSoftLimitRequest_<std::allocator<void> > ResetTwistAngularSoftLimitRequest;

typedef boost::shared_ptr< ::kortex_driver::ResetTwistAngularSoftLimitRequest > ResetTwistAngularSoftLimitRequestPtr;
typedef boost::shared_ptr< ::kortex_driver::ResetTwistAngularSoftLimitRequest const> ResetTwistAngularSoftLimitRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator2> & rhs)
{
  return lhs.input == rhs.input;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ca19a04230874fdc811929f351eb6d06";
  }

  static const char* value(const ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xca19a04230874fdcULL;
  static const uint64_t static_value2 = 0x811929f351eb6d06ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/ResetTwistAngularSoftLimitRequest";
  }

  static const char* value(const ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ControlConfig_ControlModeInformation input\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/ControlConfig_ControlModeInformation\n"
"\n"
"uint32 control_mode\n"
;
  }

  static const char* value(const ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.input);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ResetTwistAngularSoftLimitRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::ResetTwistAngularSoftLimitRequest_<ContainerAllocator>& v)
  {
    s << indent << "input: ";
    s << std::endl;
    Printer< ::kortex_driver::ControlConfig_ControlModeInformation_<ContainerAllocator> >::stream(s, indent + "  ", v.input);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_RESETTWISTANGULARSOFTLIMITREQUEST_H
