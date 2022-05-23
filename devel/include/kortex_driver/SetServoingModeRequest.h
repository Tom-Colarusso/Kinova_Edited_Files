// Generated by gencpp from file kortex_driver/SetServoingModeRequest.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_SETSERVOINGMODEREQUEST_H
#define KORTEX_DRIVER_MESSAGE_SETSERVOINGMODEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/ServoingModeInformation.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct SetServoingModeRequest_
{
  typedef SetServoingModeRequest_<ContainerAllocator> Type;

  SetServoingModeRequest_()
    : input()  {
    }
  SetServoingModeRequest_(const ContainerAllocator& _alloc)
    : input(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::ServoingModeInformation_<ContainerAllocator>  _input_type;
  _input_type input;





  typedef boost::shared_ptr< ::kortex_driver::SetServoingModeRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::SetServoingModeRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetServoingModeRequest_

typedef ::kortex_driver::SetServoingModeRequest_<std::allocator<void> > SetServoingModeRequest;

typedef boost::shared_ptr< ::kortex_driver::SetServoingModeRequest > SetServoingModeRequestPtr;
typedef boost::shared_ptr< ::kortex_driver::SetServoingModeRequest const> SetServoingModeRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::SetServoingModeRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::SetServoingModeRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::SetServoingModeRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::SetServoingModeRequest_<ContainerAllocator2> & rhs)
{
  return lhs.input == rhs.input;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::SetServoingModeRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::SetServoingModeRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::SetServoingModeRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::SetServoingModeRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::SetServoingModeRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::SetServoingModeRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::SetServoingModeRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::SetServoingModeRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::SetServoingModeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0c796408a75c2d65cd627a13a0143c2b";
  }

  static const char* value(const ::kortex_driver::SetServoingModeRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0c796408a75c2d65ULL;
  static const uint64_t static_value2 = 0xcd627a13a0143c2bULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::SetServoingModeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/SetServoingModeRequest";
  }

  static const char* value(const ::kortex_driver::SetServoingModeRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::SetServoingModeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ServoingModeInformation input\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/ServoingModeInformation\n"
"\n"
"uint32 servoing_mode\n"
;
  }

  static const char* value(const ::kortex_driver::SetServoingModeRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::SetServoingModeRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.input);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetServoingModeRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::SetServoingModeRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::SetServoingModeRequest_<ContainerAllocator>& v)
  {
    s << indent << "input: ";
    s << std::endl;
    Printer< ::kortex_driver::ServoingModeInformation_<ContainerAllocator> >::stream(s, indent + "  ", v.input);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_SETSERVOINGMODEREQUEST_H
