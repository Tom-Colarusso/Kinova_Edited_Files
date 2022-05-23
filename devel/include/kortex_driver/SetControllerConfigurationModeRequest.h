// Generated by gencpp from file kortex_driver/SetControllerConfigurationModeRequest.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_SETCONTROLLERCONFIGURATIONMODEREQUEST_H
#define KORTEX_DRIVER_MESSAGE_SETCONTROLLERCONFIGURATIONMODEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/ControllerConfigurationMode.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct SetControllerConfigurationModeRequest_
{
  typedef SetControllerConfigurationModeRequest_<ContainerAllocator> Type;

  SetControllerConfigurationModeRequest_()
    : input()  {
    }
  SetControllerConfigurationModeRequest_(const ContainerAllocator& _alloc)
    : input(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::ControllerConfigurationMode_<ContainerAllocator>  _input_type;
  _input_type input;





  typedef boost::shared_ptr< ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetControllerConfigurationModeRequest_

typedef ::kortex_driver::SetControllerConfigurationModeRequest_<std::allocator<void> > SetControllerConfigurationModeRequest;

typedef boost::shared_ptr< ::kortex_driver::SetControllerConfigurationModeRequest > SetControllerConfigurationModeRequestPtr;
typedef boost::shared_ptr< ::kortex_driver::SetControllerConfigurationModeRequest const> SetControllerConfigurationModeRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator2> & rhs)
{
  return lhs.input == rhs.input;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8a1806dd909a81cda18bb43787efba54";
  }

  static const char* value(const ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8a1806dd909a81cdULL;
  static const uint64_t static_value2 = 0xa18bb43787efba54ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/SetControllerConfigurationModeRequest";
  }

  static const char* value(const ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ControllerConfigurationMode input\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/ControllerConfigurationMode\n"
"\n"
"bool enable\n"
;
  }

  static const char* value(const ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.input);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetControllerConfigurationModeRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::SetControllerConfigurationModeRequest_<ContainerAllocator>& v)
  {
    s << indent << "input: ";
    s << std::endl;
    Printer< ::kortex_driver::ControllerConfigurationMode_<ContainerAllocator> >::stream(s, indent + "  ", v.input);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_SETCONTROLLERCONFIGURATIONMODEREQUEST_H
