// Generated by gencpp from file kortex_driver/RunMode.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_RUNMODE_H
#define KORTEX_DRIVER_MESSAGE_RUNMODE_H


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
struct RunMode_
{
  typedef RunMode_<ContainerAllocator> Type;

  RunMode_()
    : run_mode(0)  {
    }
  RunMode_(const ContainerAllocator& _alloc)
    : run_mode(0)  {
  (void)_alloc;
    }



   typedef uint32_t _run_mode_type;
  _run_mode_type run_mode;





  typedef boost::shared_ptr< ::kortex_driver::RunMode_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::RunMode_<ContainerAllocator> const> ConstPtr;

}; // struct RunMode_

typedef ::kortex_driver::RunMode_<std::allocator<void> > RunMode;

typedef boost::shared_ptr< ::kortex_driver::RunMode > RunModePtr;
typedef boost::shared_ptr< ::kortex_driver::RunMode const> RunModeConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::RunMode_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::RunMode_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::RunMode_<ContainerAllocator1> & lhs, const ::kortex_driver::RunMode_<ContainerAllocator2> & rhs)
{
  return lhs.run_mode == rhs.run_mode;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::RunMode_<ContainerAllocator1> & lhs, const ::kortex_driver::RunMode_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::RunMode_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::RunMode_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::RunMode_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::RunMode_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::RunMode_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::RunMode_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::RunMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "abe2eb774be1b28bf7c30a06ea5e3691";
  }

  static const char* value(const ::kortex_driver::RunMode_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xabe2eb774be1b28bULL;
  static const uint64_t static_value2 = 0xf7c30a06ea5e3691ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::RunMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/RunMode";
  }

  static const char* value(const ::kortex_driver::RunMode_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::RunMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 run_mode\n"
;
  }

  static const char* value(const ::kortex_driver::RunMode_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::RunMode_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.run_mode);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RunMode_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::RunMode_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::RunMode_<ContainerAllocator>& v)
  {
    s << indent << "run_mode: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.run_mode);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_RUNMODE_H
