// Generated by gencpp from file kortex_driver/GetAllJointsTorqueSoftLimitationResponse.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETALLJOINTSTORQUESOFTLIMITATIONRESPONSE_H
#define KORTEX_DRIVER_MESSAGE_GETALLJOINTSTORQUESOFTLIMITATIONRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/JointsLimitationsList.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct GetAllJointsTorqueSoftLimitationResponse_
{
  typedef GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator> Type;

  GetAllJointsTorqueSoftLimitationResponse_()
    : output()  {
    }
  GetAllJointsTorqueSoftLimitationResponse_(const ContainerAllocator& _alloc)
    : output(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::JointsLimitationsList_<ContainerAllocator>  _output_type;
  _output_type output;





  typedef boost::shared_ptr< ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetAllJointsTorqueSoftLimitationResponse_

typedef ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<std::allocator<void> > GetAllJointsTorqueSoftLimitationResponse;

typedef boost::shared_ptr< ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse > GetAllJointsTorqueSoftLimitationResponsePtr;
typedef boost::shared_ptr< ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse const> GetAllJointsTorqueSoftLimitationResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator2> & rhs)
{
  return lhs.output == rhs.output;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "51e6c4b6163d0d3c0b0ad680758e82f2";
  }

  static const char* value(const ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x51e6c4b6163d0d3cULL;
  static const uint64_t static_value2 = 0x0b0ad680758e82f2ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/GetAllJointsTorqueSoftLimitationResponse";
  }

  static const char* value(const ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "JointsLimitationsList output\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/JointsLimitationsList\n"
"\n"
"JointLimitation[] joints_limitations\n"
"================================================================================\n"
"MSG: kortex_driver/JointLimitation\n"
"\n"
"uint32 joint_identifier\n"
"uint32 type\n"
"float32 value\n"
;
  }

  static const char* value(const ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.output);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetAllJointsTorqueSoftLimitationResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::GetAllJointsTorqueSoftLimitationResponse_<ContainerAllocator>& v)
  {
    s << indent << "output: ";
    s << std::endl;
    Printer< ::kortex_driver::JointsLimitationsList_<ContainerAllocator> >::stream(s, indent + "  ", v.output);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETALLJOINTSTORQUESOFTLIMITATIONRESPONSE_H
