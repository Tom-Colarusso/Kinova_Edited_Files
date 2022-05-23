// Generated by gencpp from file kortex_driver/GetFirmwareBundleVersionsRequest.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETFIRMWAREBUNDLEVERSIONSREQUEST_H
#define KORTEX_DRIVER_MESSAGE_GETFIRMWAREBUNDLEVERSIONSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/Empty.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct GetFirmwareBundleVersionsRequest_
{
  typedef GetFirmwareBundleVersionsRequest_<ContainerAllocator> Type;

  GetFirmwareBundleVersionsRequest_()
    : input()  {
    }
  GetFirmwareBundleVersionsRequest_(const ContainerAllocator& _alloc)
    : input(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::Empty_<ContainerAllocator>  _input_type;
  _input_type input;





  typedef boost::shared_ptr< ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetFirmwareBundleVersionsRequest_

typedef ::kortex_driver::GetFirmwareBundleVersionsRequest_<std::allocator<void> > GetFirmwareBundleVersionsRequest;

typedef boost::shared_ptr< ::kortex_driver::GetFirmwareBundleVersionsRequest > GetFirmwareBundleVersionsRequestPtr;
typedef boost::shared_ptr< ::kortex_driver::GetFirmwareBundleVersionsRequest const> GetFirmwareBundleVersionsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator2> & rhs)
{
  return lhs.input == rhs.input;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fa3403cd5897c9698bc0fdcb2a453fbc";
  }

  static const char* value(const ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfa3403cd5897c969ULL;
  static const uint64_t static_value2 = 0x8bc0fdcb2a453fbcULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/GetFirmwareBundleVersionsRequest";
  }

  static const char* value(const ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Empty input\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/Empty\n"
;
  }

  static const char* value(const ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.input);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetFirmwareBundleVersionsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::GetFirmwareBundleVersionsRequest_<ContainerAllocator>& v)
  {
    s << indent << "input: ";
    s << std::endl;
    Printer< ::kortex_driver::Empty_<ContainerAllocator> >::stream(s, indent + "  ", v.input);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETFIRMWAREBUNDLEVERSIONSREQUEST_H
