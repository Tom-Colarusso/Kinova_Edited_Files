// Generated by gencpp from file kortex_driver/ControllerNotification_state.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_CONTROLLERNOTIFICATION_STATE_H
#define KORTEX_DRIVER_MESSAGE_CONTROLLERNOTIFICATION_STATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/ControllerState.h>
#include <kortex_driver/ControllerElementState.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct ControllerNotification_state_
{
  typedef ControllerNotification_state_<ContainerAllocator> Type;

  ControllerNotification_state_()
    : controller_state()
    , controller_element()  {
    }
  ControllerNotification_state_(const ContainerAllocator& _alloc)
    : controller_state(_alloc)
    , controller_element(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::kortex_driver::ControllerState_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::kortex_driver::ControllerState_<ContainerAllocator> >::other >  _controller_state_type;
  _controller_state_type controller_state;

   typedef std::vector< ::kortex_driver::ControllerElementState_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::kortex_driver::ControllerElementState_<ContainerAllocator> >::other >  _controller_element_type;
  _controller_element_type controller_element;





  typedef boost::shared_ptr< ::kortex_driver::ControllerNotification_state_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::ControllerNotification_state_<ContainerAllocator> const> ConstPtr;

}; // struct ControllerNotification_state_

typedef ::kortex_driver::ControllerNotification_state_<std::allocator<void> > ControllerNotification_state;

typedef boost::shared_ptr< ::kortex_driver::ControllerNotification_state > ControllerNotification_statePtr;
typedef boost::shared_ptr< ::kortex_driver::ControllerNotification_state const> ControllerNotification_stateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::ControllerNotification_state_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::ControllerNotification_state_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::ControllerNotification_state_<ContainerAllocator1> & lhs, const ::kortex_driver::ControllerNotification_state_<ContainerAllocator2> & rhs)
{
  return lhs.controller_state == rhs.controller_state &&
    lhs.controller_element == rhs.controller_element;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::ControllerNotification_state_<ContainerAllocator1> & lhs, const ::kortex_driver::ControllerNotification_state_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ControllerNotification_state_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ControllerNotification_state_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ControllerNotification_state_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ControllerNotification_state_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ControllerNotification_state_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ControllerNotification_state_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::ControllerNotification_state_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f624e32135515547aa37c46203c9be9e";
  }

  static const char* value(const ::kortex_driver::ControllerNotification_state_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf624e32135515547ULL;
  static const uint64_t static_value2 = 0xaa37c46203c9be9eULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::ControllerNotification_state_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/ControllerNotification_state";
  }

  static const char* value(const ::kortex_driver::ControllerNotification_state_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::ControllerNotification_state_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"ControllerState[] controller_state\n"
"ControllerElementState[] controller_element\n"
"================================================================================\n"
"MSG: kortex_driver/ControllerState\n"
"\n"
"ControllerHandle handle\n"
"uint32 event_type\n"
"================================================================================\n"
"MSG: kortex_driver/ControllerHandle\n"
"\n"
"uint32 type\n"
"uint32 controller_identifier\n"
"================================================================================\n"
"MSG: kortex_driver/ControllerElementState\n"
"\n"
"ControllerElementHandle handle\n"
"uint32 event_type\n"
"float32 axis_value\n"
"================================================================================\n"
"MSG: kortex_driver/ControllerElementHandle\n"
"\n"
"ControllerHandle controller_handle\n"
"ControllerElementHandle_identifier oneof_identifier\n"
"================================================================================\n"
"MSG: kortex_driver/ControllerElementHandle_identifier\n"
"\n"
"uint32[] button\n"
"uint32[] axis\n"
;
  }

  static const char* value(const ::kortex_driver::ControllerNotification_state_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::ControllerNotification_state_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.controller_state);
      stream.next(m.controller_element);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ControllerNotification_state_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::ControllerNotification_state_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::ControllerNotification_state_<ContainerAllocator>& v)
  {
    s << indent << "controller_state[]" << std::endl;
    for (size_t i = 0; i < v.controller_state.size(); ++i)
    {
      s << indent << "  controller_state[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::kortex_driver::ControllerState_<ContainerAllocator> >::stream(s, indent + "    ", v.controller_state[i]);
    }
    s << indent << "controller_element[]" << std::endl;
    for (size_t i = 0; i < v.controller_element.size(); ++i)
    {
      s << indent << "  controller_element[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::kortex_driver::ControllerElementState_<ContainerAllocator> >::stream(s, indent + "    ", v.controller_element[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_CONTROLLERNOTIFICATION_STATE_H
