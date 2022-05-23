// Generated by gencpp from file kortex_driver/ReadSequenceTaskRequest.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_READSEQUENCETASKREQUEST_H
#define KORTEX_DRIVER_MESSAGE_READSEQUENCETASKREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/SequenceTaskHandle.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct ReadSequenceTaskRequest_
{
  typedef ReadSequenceTaskRequest_<ContainerAllocator> Type;

  ReadSequenceTaskRequest_()
    : input()  {
    }
  ReadSequenceTaskRequest_(const ContainerAllocator& _alloc)
    : input(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::SequenceTaskHandle_<ContainerAllocator>  _input_type;
  _input_type input;





  typedef boost::shared_ptr< ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator> const> ConstPtr;

}; // struct ReadSequenceTaskRequest_

typedef ::kortex_driver::ReadSequenceTaskRequest_<std::allocator<void> > ReadSequenceTaskRequest;

typedef boost::shared_ptr< ::kortex_driver::ReadSequenceTaskRequest > ReadSequenceTaskRequestPtr;
typedef boost::shared_ptr< ::kortex_driver::ReadSequenceTaskRequest const> ReadSequenceTaskRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator2> & rhs)
{
  return lhs.input == rhs.input;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8543eff83ef732ac4834b015839aa279";
  }

  static const char* value(const ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8543eff83ef732acULL;
  static const uint64_t static_value2 = 0x4834b015839aa279ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/ReadSequenceTaskRequest";
  }

  static const char* value(const ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "SequenceTaskHandle input\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/SequenceTaskHandle\n"
"\n"
"SequenceHandle sequence_handle\n"
"uint32 task_index\n"
"================================================================================\n"
"MSG: kortex_driver/SequenceHandle\n"
"\n"
"uint32 identifier\n"
"uint32 permission\n"
;
  }

  static const char* value(const ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.input);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ReadSequenceTaskRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::ReadSequenceTaskRequest_<ContainerAllocator>& v)
  {
    s << indent << "input: ";
    s << std::endl;
    Printer< ::kortex_driver::SequenceTaskHandle_<ContainerAllocator> >::stream(s, indent + "  ", v.input);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_READSEQUENCETASKREQUEST_H
