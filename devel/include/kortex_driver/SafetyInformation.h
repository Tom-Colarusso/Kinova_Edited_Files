// Generated by gencpp from file kortex_driver/SafetyInformation.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_SAFETYINFORMATION_H
#define KORTEX_DRIVER_MESSAGE_SAFETYINFORMATION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/SafetyHandle.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct SafetyInformation_
{
  typedef SafetyInformation_<ContainerAllocator> Type;

  SafetyInformation_()
    : handle()
    , can_change_safety_state(false)
    , has_warning_threshold(false)
    , has_error_threshold(false)
    , limit_type(0)
    , default_warning_threshold(0.0)
    , default_error_threshold(0.0)
    , upper_hard_limit(0.0)
    , lower_hard_limit(0.0)
    , status(0)
    , unit(0)  {
    }
  SafetyInformation_(const ContainerAllocator& _alloc)
    : handle(_alloc)
    , can_change_safety_state(false)
    , has_warning_threshold(false)
    , has_error_threshold(false)
    , limit_type(0)
    , default_warning_threshold(0.0)
    , default_error_threshold(0.0)
    , upper_hard_limit(0.0)
    , lower_hard_limit(0.0)
    , status(0)
    , unit(0)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::SafetyHandle_<ContainerAllocator>  _handle_type;
  _handle_type handle;

   typedef uint8_t _can_change_safety_state_type;
  _can_change_safety_state_type can_change_safety_state;

   typedef uint8_t _has_warning_threshold_type;
  _has_warning_threshold_type has_warning_threshold;

   typedef uint8_t _has_error_threshold_type;
  _has_error_threshold_type has_error_threshold;

   typedef uint32_t _limit_type_type;
  _limit_type_type limit_type;

   typedef float _default_warning_threshold_type;
  _default_warning_threshold_type default_warning_threshold;

   typedef float _default_error_threshold_type;
  _default_error_threshold_type default_error_threshold;

   typedef float _upper_hard_limit_type;
  _upper_hard_limit_type upper_hard_limit;

   typedef float _lower_hard_limit_type;
  _lower_hard_limit_type lower_hard_limit;

   typedef uint32_t _status_type;
  _status_type status;

   typedef uint32_t _unit_type;
  _unit_type unit;





  typedef boost::shared_ptr< ::kortex_driver::SafetyInformation_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::SafetyInformation_<ContainerAllocator> const> ConstPtr;

}; // struct SafetyInformation_

typedef ::kortex_driver::SafetyInformation_<std::allocator<void> > SafetyInformation;

typedef boost::shared_ptr< ::kortex_driver::SafetyInformation > SafetyInformationPtr;
typedef boost::shared_ptr< ::kortex_driver::SafetyInformation const> SafetyInformationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::SafetyInformation_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::SafetyInformation_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::SafetyInformation_<ContainerAllocator1> & lhs, const ::kortex_driver::SafetyInformation_<ContainerAllocator2> & rhs)
{
  return lhs.handle == rhs.handle &&
    lhs.can_change_safety_state == rhs.can_change_safety_state &&
    lhs.has_warning_threshold == rhs.has_warning_threshold &&
    lhs.has_error_threshold == rhs.has_error_threshold &&
    lhs.limit_type == rhs.limit_type &&
    lhs.default_warning_threshold == rhs.default_warning_threshold &&
    lhs.default_error_threshold == rhs.default_error_threshold &&
    lhs.upper_hard_limit == rhs.upper_hard_limit &&
    lhs.lower_hard_limit == rhs.lower_hard_limit &&
    lhs.status == rhs.status &&
    lhs.unit == rhs.unit;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::SafetyInformation_<ContainerAllocator1> & lhs, const ::kortex_driver::SafetyInformation_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::SafetyInformation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::SafetyInformation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::SafetyInformation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::SafetyInformation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::SafetyInformation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::SafetyInformation_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::SafetyInformation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e8597ef9acfa23c653020b88d86d8b2f";
  }

  static const char* value(const ::kortex_driver::SafetyInformation_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe8597ef9acfa23c6ULL;
  static const uint64_t static_value2 = 0x53020b88d86d8b2fULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::SafetyInformation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/SafetyInformation";
  }

  static const char* value(const ::kortex_driver::SafetyInformation_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::SafetyInformation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"SafetyHandle handle\n"
"bool can_change_safety_state\n"
"bool has_warning_threshold\n"
"bool has_error_threshold\n"
"uint32 limit_type\n"
"float32 default_warning_threshold\n"
"float32 default_error_threshold\n"
"float32 upper_hard_limit\n"
"float32 lower_hard_limit\n"
"uint32 status\n"
"uint32 unit\n"
"================================================================================\n"
"MSG: kortex_driver/SafetyHandle\n"
"\n"
"uint32 identifier\n"
;
  }

  static const char* value(const ::kortex_driver::SafetyInformation_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::SafetyInformation_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.handle);
      stream.next(m.can_change_safety_state);
      stream.next(m.has_warning_threshold);
      stream.next(m.has_error_threshold);
      stream.next(m.limit_type);
      stream.next(m.default_warning_threshold);
      stream.next(m.default_error_threshold);
      stream.next(m.upper_hard_limit);
      stream.next(m.lower_hard_limit);
      stream.next(m.status);
      stream.next(m.unit);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SafetyInformation_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::SafetyInformation_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::SafetyInformation_<ContainerAllocator>& v)
  {
    s << indent << "handle: ";
    s << std::endl;
    Printer< ::kortex_driver::SafetyHandle_<ContainerAllocator> >::stream(s, indent + "  ", v.handle);
    s << indent << "can_change_safety_state: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.can_change_safety_state);
    s << indent << "has_warning_threshold: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.has_warning_threshold);
    s << indent << "has_error_threshold: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.has_error_threshold);
    s << indent << "limit_type: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.limit_type);
    s << indent << "default_warning_threshold: ";
    Printer<float>::stream(s, indent + "  ", v.default_warning_threshold);
    s << indent << "default_error_threshold: ";
    Printer<float>::stream(s, indent + "  ", v.default_error_threshold);
    s << indent << "upper_hard_limit: ";
    Printer<float>::stream(s, indent + "  ", v.upper_hard_limit);
    s << indent << "lower_hard_limit: ";
    Printer<float>::stream(s, indent + "  ", v.lower_hard_limit);
    s << indent << "status: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.status);
    s << indent << "unit: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.unit);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_SAFETYINFORMATION_H
