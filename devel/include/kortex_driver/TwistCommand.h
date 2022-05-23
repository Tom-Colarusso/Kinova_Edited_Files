// Generated by gencpp from file kortex_driver/TwistCommand.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_TWISTCOMMAND_H
#define KORTEX_DRIVER_MESSAGE_TWISTCOMMAND_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/Twist.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct TwistCommand_
{
  typedef TwistCommand_<ContainerAllocator> Type;

  TwistCommand_()
    : reference_frame(0)
    , twist()
    , duration(0)  {
    }
  TwistCommand_(const ContainerAllocator& _alloc)
    : reference_frame(0)
    , twist(_alloc)
    , duration(0)  {
  (void)_alloc;
    }



   typedef uint32_t _reference_frame_type;
  _reference_frame_type reference_frame;

   typedef  ::kortex_driver::Twist_<ContainerAllocator>  _twist_type;
  _twist_type twist;

   typedef uint32_t _duration_type;
  _duration_type duration;





  typedef boost::shared_ptr< ::kortex_driver::TwistCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::TwistCommand_<ContainerAllocator> const> ConstPtr;

}; // struct TwistCommand_

typedef ::kortex_driver::TwistCommand_<std::allocator<void> > TwistCommand;

typedef boost::shared_ptr< ::kortex_driver::TwistCommand > TwistCommandPtr;
typedef boost::shared_ptr< ::kortex_driver::TwistCommand const> TwistCommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::TwistCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::TwistCommand_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::TwistCommand_<ContainerAllocator1> & lhs, const ::kortex_driver::TwistCommand_<ContainerAllocator2> & rhs)
{
  return lhs.reference_frame == rhs.reference_frame &&
    lhs.twist == rhs.twist &&
    lhs.duration == rhs.duration;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::TwistCommand_<ContainerAllocator1> & lhs, const ::kortex_driver::TwistCommand_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::TwistCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::TwistCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::TwistCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::TwistCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::TwistCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::TwistCommand_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::TwistCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "16dcfd20a022a10eea1f05e5a9cbb18a";
  }

  static const char* value(const ::kortex_driver::TwistCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x16dcfd20a022a10eULL;
  static const uint64_t static_value2 = 0xea1f05e5a9cbb18aULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::TwistCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/TwistCommand";
  }

  static const char* value(const ::kortex_driver::TwistCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::TwistCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 reference_frame\n"
"Twist twist\n"
"uint32 duration\n"
"================================================================================\n"
"MSG: kortex_driver/Twist\n"
"\n"
"float32 linear_x\n"
"float32 linear_y\n"
"float32 linear_z\n"
"float32 angular_x\n"
"float32 angular_y\n"
"float32 angular_z\n"
;
  }

  static const char* value(const ::kortex_driver::TwistCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::TwistCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.reference_frame);
      stream.next(m.twist);
      stream.next(m.duration);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TwistCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::TwistCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::TwistCommand_<ContainerAllocator>& v)
  {
    s << indent << "reference_frame: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.reference_frame);
    s << indent << "twist: ";
    s << std::endl;
    Printer< ::kortex_driver::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.twist);
    s << indent << "duration: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.duration);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_TWISTCOMMAND_H
