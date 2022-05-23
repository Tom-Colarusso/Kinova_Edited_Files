// Generated by gencpp from file kortex_driver/SensorFocusAction.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_SENSORFOCUSACTION_H
#define KORTEX_DRIVER_MESSAGE_SENSORFOCUSACTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/SensorFocusAction_action_parameters.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct SensorFocusAction_
{
  typedef SensorFocusAction_<ContainerAllocator> Type;

  SensorFocusAction_()
    : sensor(0)
    , focus_action(0)
    , oneof_action_parameters()  {
    }
  SensorFocusAction_(const ContainerAllocator& _alloc)
    : sensor(0)
    , focus_action(0)
    , oneof_action_parameters(_alloc)  {
  (void)_alloc;
    }



   typedef uint32_t _sensor_type;
  _sensor_type sensor;

   typedef uint32_t _focus_action_type;
  _focus_action_type focus_action;

   typedef  ::kortex_driver::SensorFocusAction_action_parameters_<ContainerAllocator>  _oneof_action_parameters_type;
  _oneof_action_parameters_type oneof_action_parameters;





  typedef boost::shared_ptr< ::kortex_driver::SensorFocusAction_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::SensorFocusAction_<ContainerAllocator> const> ConstPtr;

}; // struct SensorFocusAction_

typedef ::kortex_driver::SensorFocusAction_<std::allocator<void> > SensorFocusAction;

typedef boost::shared_ptr< ::kortex_driver::SensorFocusAction > SensorFocusActionPtr;
typedef boost::shared_ptr< ::kortex_driver::SensorFocusAction const> SensorFocusActionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::SensorFocusAction_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::SensorFocusAction_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::SensorFocusAction_<ContainerAllocator1> & lhs, const ::kortex_driver::SensorFocusAction_<ContainerAllocator2> & rhs)
{
  return lhs.sensor == rhs.sensor &&
    lhs.focus_action == rhs.focus_action &&
    lhs.oneof_action_parameters == rhs.oneof_action_parameters;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::SensorFocusAction_<ContainerAllocator1> & lhs, const ::kortex_driver::SensorFocusAction_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::SensorFocusAction_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::SensorFocusAction_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::SensorFocusAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::SensorFocusAction_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::SensorFocusAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::SensorFocusAction_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::SensorFocusAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cae3081e3d5f90fbdd9351620859003c";
  }

  static const char* value(const ::kortex_driver::SensorFocusAction_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcae3081e3d5f90fbULL;
  static const uint64_t static_value2 = 0xdd9351620859003cULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::SensorFocusAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/SensorFocusAction";
  }

  static const char* value(const ::kortex_driver::SensorFocusAction_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::SensorFocusAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 sensor\n"
"uint32 focus_action\n"
"SensorFocusAction_action_parameters oneof_action_parameters\n"
"================================================================================\n"
"MSG: kortex_driver/SensorFocusAction_action_parameters\n"
"\n"
"FocusPoint[] focus_point\n"
"ManualFocus[] manual_focus\n"
"================================================================================\n"
"MSG: kortex_driver/FocusPoint\n"
"\n"
"uint32 x\n"
"uint32 y\n"
"================================================================================\n"
"MSG: kortex_driver/ManualFocus\n"
"\n"
"uint32 value\n"
;
  }

  static const char* value(const ::kortex_driver::SensorFocusAction_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::SensorFocusAction_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.sensor);
      stream.next(m.focus_action);
      stream.next(m.oneof_action_parameters);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SensorFocusAction_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::SensorFocusAction_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::SensorFocusAction_<ContainerAllocator>& v)
  {
    s << indent << "sensor: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.sensor);
    s << indent << "focus_action: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.focus_action);
    s << indent << "oneof_action_parameters: ";
    s << std::endl;
    Printer< ::kortex_driver::SensorFocusAction_action_parameters_<ContainerAllocator> >::stream(s, indent + "  ", v.oneof_action_parameters);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_SENSORFOCUSACTION_H
