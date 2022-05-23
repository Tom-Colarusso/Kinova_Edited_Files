// Generated by gencpp from file kortex_driver/PlaySelectedJointTrajectoryRequest.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_PLAYSELECTEDJOINTTRAJECTORYREQUEST_H
#define KORTEX_DRIVER_MESSAGE_PLAYSELECTEDJOINTTRAJECTORYREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/ConstrainedJointAngle.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct PlaySelectedJointTrajectoryRequest_
{
  typedef PlaySelectedJointTrajectoryRequest_<ContainerAllocator> Type;

  PlaySelectedJointTrajectoryRequest_()
    : input()  {
    }
  PlaySelectedJointTrajectoryRequest_(const ContainerAllocator& _alloc)
    : input(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator>  _input_type;
  _input_type input;





  typedef boost::shared_ptr< ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator> const> ConstPtr;

}; // struct PlaySelectedJointTrajectoryRequest_

typedef ::kortex_driver::PlaySelectedJointTrajectoryRequest_<std::allocator<void> > PlaySelectedJointTrajectoryRequest;

typedef boost::shared_ptr< ::kortex_driver::PlaySelectedJointTrajectoryRequest > PlaySelectedJointTrajectoryRequestPtr;
typedef boost::shared_ptr< ::kortex_driver::PlaySelectedJointTrajectoryRequest const> PlaySelectedJointTrajectoryRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator2> & rhs)
{
  return lhs.input == rhs.input;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4b5498a3a68bc52a621a3c2619c54c24";
  }

  static const char* value(const ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4b5498a3a68bc52aULL;
  static const uint64_t static_value2 = 0x621a3c2619c54c24ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/PlaySelectedJointTrajectoryRequest";
  }

  static const char* value(const ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ConstrainedJointAngle input\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/ConstrainedJointAngle\n"
"\n"
"uint32 joint_identifier\n"
"float32 value\n"
"JointTrajectoryConstraint constraint\n"
"================================================================================\n"
"MSG: kortex_driver/JointTrajectoryConstraint\n"
"\n"
"uint32 type\n"
"float32 value\n"
;
  }

  static const char* value(const ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.input);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PlaySelectedJointTrajectoryRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::PlaySelectedJointTrajectoryRequest_<ContainerAllocator>& v)
  {
    s << indent << "input: ";
    s << std::endl;
    Printer< ::kortex_driver::ConstrainedJointAngle_<ContainerAllocator> >::stream(s, indent + "  ", v.input);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_PLAYSELECTEDJOINTTRAJECTORYREQUEST_H
