// Generated by gencpp from file kortex_driver/VectorDriveParameters.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_VECTORDRIVEPARAMETERS_H
#define KORTEX_DRIVER_MESSAGE_VECTORDRIVEPARAMETERS_H


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
struct VectorDriveParameters_
{
  typedef VectorDriveParameters_<ContainerAllocator> Type;

  VectorDriveParameters_()
    : kpq(0.0)
    , kiq(0.0)
    , kpd(0.0)
    , kid(0.0)  {
    }
  VectorDriveParameters_(const ContainerAllocator& _alloc)
    : kpq(0.0)
    , kiq(0.0)
    , kpd(0.0)
    , kid(0.0)  {
  (void)_alloc;
    }



   typedef float _kpq_type;
  _kpq_type kpq;

   typedef float _kiq_type;
  _kiq_type kiq;

   typedef float _kpd_type;
  _kpd_type kpd;

   typedef float _kid_type;
  _kid_type kid;





  typedef boost::shared_ptr< ::kortex_driver::VectorDriveParameters_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::VectorDriveParameters_<ContainerAllocator> const> ConstPtr;

}; // struct VectorDriveParameters_

typedef ::kortex_driver::VectorDriveParameters_<std::allocator<void> > VectorDriveParameters;

typedef boost::shared_ptr< ::kortex_driver::VectorDriveParameters > VectorDriveParametersPtr;
typedef boost::shared_ptr< ::kortex_driver::VectorDriveParameters const> VectorDriveParametersConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::VectorDriveParameters_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::VectorDriveParameters_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::VectorDriveParameters_<ContainerAllocator1> & lhs, const ::kortex_driver::VectorDriveParameters_<ContainerAllocator2> & rhs)
{
  return lhs.kpq == rhs.kpq &&
    lhs.kiq == rhs.kiq &&
    lhs.kpd == rhs.kpd &&
    lhs.kid == rhs.kid;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::VectorDriveParameters_<ContainerAllocator1> & lhs, const ::kortex_driver::VectorDriveParameters_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::VectorDriveParameters_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::VectorDriveParameters_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::VectorDriveParameters_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::VectorDriveParameters_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::VectorDriveParameters_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::VectorDriveParameters_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::VectorDriveParameters_<ContainerAllocator> >
{
  static const char* value()
  {
    return "30e295016ac63da10cbe59e969bb6401";
  }

  static const char* value(const ::kortex_driver::VectorDriveParameters_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x30e295016ac63da1ULL;
  static const uint64_t static_value2 = 0x0cbe59e969bb6401ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::VectorDriveParameters_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/VectorDriveParameters";
  }

  static const char* value(const ::kortex_driver::VectorDriveParameters_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::VectorDriveParameters_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"float32 kpq\n"
"float32 kiq\n"
"float32 kpd\n"
"float32 kid\n"
;
  }

  static const char* value(const ::kortex_driver::VectorDriveParameters_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::VectorDriveParameters_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.kpq);
      stream.next(m.kiq);
      stream.next(m.kpd);
      stream.next(m.kid);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct VectorDriveParameters_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::VectorDriveParameters_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::VectorDriveParameters_<ContainerAllocator>& v)
  {
    s << indent << "kpq: ";
    Printer<float>::stream(s, indent + "  ", v.kpq);
    s << indent << "kiq: ";
    Printer<float>::stream(s, indent + "  ", v.kiq);
    s << indent << "kpd: ";
    Printer<float>::stream(s, indent + "  ", v.kpd);
    s << indent << "kid: ";
    Printer<float>::stream(s, indent + "  ", v.kid);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_VECTORDRIVEPARAMETERS_H
