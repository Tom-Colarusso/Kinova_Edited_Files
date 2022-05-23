// Generated by gencpp from file kortex_driver/InterconnectCyclic_Command_tool_command.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_INTERCONNECTCYCLIC_COMMAND_TOOL_COMMAND_H
#define KORTEX_DRIVER_MESSAGE_INTERCONNECTCYCLIC_COMMAND_TOOL_COMMAND_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/GripperCyclic_Command.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct InterconnectCyclic_Command_tool_command_
{
  typedef InterconnectCyclic_Command_tool_command_<ContainerAllocator> Type;

  InterconnectCyclic_Command_tool_command_()
    : gripper_command()  {
    }
  InterconnectCyclic_Command_tool_command_(const ContainerAllocator& _alloc)
    : gripper_command(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::kortex_driver::GripperCyclic_Command_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::kortex_driver::GripperCyclic_Command_<ContainerAllocator> >::other >  _gripper_command_type;
  _gripper_command_type gripper_command;





  typedef boost::shared_ptr< ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator> const> ConstPtr;

}; // struct InterconnectCyclic_Command_tool_command_

typedef ::kortex_driver::InterconnectCyclic_Command_tool_command_<std::allocator<void> > InterconnectCyclic_Command_tool_command;

typedef boost::shared_ptr< ::kortex_driver::InterconnectCyclic_Command_tool_command > InterconnectCyclic_Command_tool_commandPtr;
typedef boost::shared_ptr< ::kortex_driver::InterconnectCyclic_Command_tool_command const> InterconnectCyclic_Command_tool_commandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator1> & lhs, const ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator2> & rhs)
{
  return lhs.gripper_command == rhs.gripper_command;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator1> & lhs, const ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cb4b9ede5a008a07d439d113030c34d0";
  }

  static const char* value(const ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcb4b9ede5a008a07ULL;
  static const uint64_t static_value2 = 0xd439d113030c34d0ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/InterconnectCyclic_Command_tool_command";
  }

  static const char* value(const ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"GripperCyclic_Command[] gripper_command\n"
"================================================================================\n"
"MSG: kortex_driver/GripperCyclic_Command\n"
"\n"
"GripperCyclic_MessageId command_id\n"
"uint32 flags\n"
"MotorCommand[] motor_cmd\n"
"================================================================================\n"
"MSG: kortex_driver/GripperCyclic_MessageId\n"
"\n"
"uint32 identifier\n"
"================================================================================\n"
"MSG: kortex_driver/MotorCommand\n"
"\n"
"uint32 motor_id\n"
"float32 position\n"
"float32 velocity\n"
"float32 force\n"
;
  }

  static const char* value(const ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.gripper_command);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct InterconnectCyclic_Command_tool_command_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::InterconnectCyclic_Command_tool_command_<ContainerAllocator>& v)
  {
    s << indent << "gripper_command[]" << std::endl;
    for (size_t i = 0; i < v.gripper_command.size(); ++i)
    {
      s << indent << "  gripper_command[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::kortex_driver::GripperCyclic_Command_<ContainerAllocator> >::stream(s, indent + "    ", v.gripper_command[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_INTERCONNECTCYCLIC_COMMAND_TOOL_COMMAND_H