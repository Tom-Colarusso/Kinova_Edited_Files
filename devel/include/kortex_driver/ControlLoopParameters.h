// Generated by gencpp from file kortex_driver/ControlLoopParameters.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_CONTROLLOOPPARAMETERS_H
#define KORTEX_DRIVER_MESSAGE_CONTROLLOOPPARAMETERS_H


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
struct ControlLoopParameters_
{
  typedef ControlLoopParameters_<ContainerAllocator> Type;

  ControlLoopParameters_()
    : loop_selection(0)
    , error_saturation(0.0)
    , output_saturation(0.0)
    , kAz()
    , kBz()
    , error_dead_band(0.0)  {
    }
  ControlLoopParameters_(const ContainerAllocator& _alloc)
    : loop_selection(0)
    , error_saturation(0.0)
    , output_saturation(0.0)
    , kAz(_alloc)
    , kBz(_alloc)
    , error_dead_band(0.0)  {
  (void)_alloc;
    }



   typedef uint32_t _loop_selection_type;
  _loop_selection_type loop_selection;

   typedef float _error_saturation_type;
  _error_saturation_type error_saturation;

   typedef float _output_saturation_type;
  _output_saturation_type output_saturation;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _kAz_type;
  _kAz_type kAz;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _kBz_type;
  _kBz_type kBz;

   typedef float _error_dead_band_type;
  _error_dead_band_type error_dead_band;





  typedef boost::shared_ptr< ::kortex_driver::ControlLoopParameters_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::ControlLoopParameters_<ContainerAllocator> const> ConstPtr;

}; // struct ControlLoopParameters_

typedef ::kortex_driver::ControlLoopParameters_<std::allocator<void> > ControlLoopParameters;

typedef boost::shared_ptr< ::kortex_driver::ControlLoopParameters > ControlLoopParametersPtr;
typedef boost::shared_ptr< ::kortex_driver::ControlLoopParameters const> ControlLoopParametersConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::ControlLoopParameters_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::ControlLoopParameters_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::ControlLoopParameters_<ContainerAllocator1> & lhs, const ::kortex_driver::ControlLoopParameters_<ContainerAllocator2> & rhs)
{
  return lhs.loop_selection == rhs.loop_selection &&
    lhs.error_saturation == rhs.error_saturation &&
    lhs.output_saturation == rhs.output_saturation &&
    lhs.kAz == rhs.kAz &&
    lhs.kBz == rhs.kBz &&
    lhs.error_dead_band == rhs.error_dead_band;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::ControlLoopParameters_<ContainerAllocator1> & lhs, const ::kortex_driver::ControlLoopParameters_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ControlLoopParameters_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ControlLoopParameters_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ControlLoopParameters_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ControlLoopParameters_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ControlLoopParameters_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ControlLoopParameters_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::ControlLoopParameters_<ContainerAllocator> >
{
  static const char* value()
  {
    return "309e14eef078d5bea86d94317d3d0e04";
  }

  static const char* value(const ::kortex_driver::ControlLoopParameters_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x309e14eef078d5beULL;
  static const uint64_t static_value2 = 0xa86d94317d3d0e04ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::ControlLoopParameters_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/ControlLoopParameters";
  }

  static const char* value(const ::kortex_driver::ControlLoopParameters_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::ControlLoopParameters_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 loop_selection\n"
"float32 error_saturation\n"
"float32 output_saturation\n"
"float32[] kAz\n"
"float32[] kBz\n"
"float32 error_dead_band\n"
;
  }

  static const char* value(const ::kortex_driver::ControlLoopParameters_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::ControlLoopParameters_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.loop_selection);
      stream.next(m.error_saturation);
      stream.next(m.output_saturation);
      stream.next(m.kAz);
      stream.next(m.kBz);
      stream.next(m.error_dead_band);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ControlLoopParameters_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::ControlLoopParameters_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::ControlLoopParameters_<ContainerAllocator>& v)
  {
    s << indent << "loop_selection: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.loop_selection);
    s << indent << "error_saturation: ";
    Printer<float>::stream(s, indent + "  ", v.error_saturation);
    s << indent << "output_saturation: ";
    Printer<float>::stream(s, indent + "  ", v.output_saturation);
    s << indent << "kAz[]" << std::endl;
    for (size_t i = 0; i < v.kAz.size(); ++i)
    {
      s << indent << "  kAz[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.kAz[i]);
    }
    s << indent << "kBz[]" << std::endl;
    for (size_t i = 0; i < v.kBz.size(); ++i)
    {
      s << indent << "  kBz[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.kBz[i]);
    }
    s << indent << "error_dead_band: ";
    Printer<float>::stream(s, indent + "  ", v.error_dead_band);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_CONTROLLOOPPARAMETERS_H
