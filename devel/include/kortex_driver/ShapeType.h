// Generated by gencpp from file kortex_driver/ShapeType.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_SHAPETYPE_H
#define KORTEX_DRIVER_MESSAGE_SHAPETYPE_H


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
struct ShapeType_
{
  typedef ShapeType_<ContainerAllocator> Type;

  ShapeType_()
    {
    }
  ShapeType_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }





// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(UNSPECIFIED_SHAPE)
  #undef UNSPECIFIED_SHAPE
#endif
#if defined(_WIN32) && defined(CYLINDER)
  #undef CYLINDER
#endif
#if defined(_WIN32) && defined(SPHERE)
  #undef SPHERE
#endif
#if defined(_WIN32) && defined(RECTANGULAR_PRISM)
  #undef RECTANGULAR_PRISM
#endif

  enum {
    UNSPECIFIED_SHAPE = 0u,
    CYLINDER = 1u,
    SPHERE = 2u,
    RECTANGULAR_PRISM = 3u,
  };


  typedef boost::shared_ptr< ::kortex_driver::ShapeType_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::ShapeType_<ContainerAllocator> const> ConstPtr;

}; // struct ShapeType_

typedef ::kortex_driver::ShapeType_<std::allocator<void> > ShapeType;

typedef boost::shared_ptr< ::kortex_driver::ShapeType > ShapeTypePtr;
typedef boost::shared_ptr< ::kortex_driver::ShapeType const> ShapeTypeConstPtr;

// constants requiring out of line definition

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::ShapeType_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::ShapeType_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ShapeType_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ShapeType_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ShapeType_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ShapeType_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ShapeType_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ShapeType_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::ShapeType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8cce3154d49982341a5a162e90cffa5a";
  }

  static const char* value(const ::kortex_driver::ShapeType_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8cce3154d4998234ULL;
  static const uint64_t static_value2 = 0x1a5a162e90cffa5aULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::ShapeType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/ShapeType";
  }

  static const char* value(const ::kortex_driver::ShapeType_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::ShapeType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 UNSPECIFIED_SHAPE = 0\n"
"\n"
"uint32 CYLINDER = 1\n"
"\n"
"uint32 SPHERE = 2\n"
"\n"
"uint32 RECTANGULAR_PRISM = 3\n"
;
  }

  static const char* value(const ::kortex_driver::ShapeType_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::ShapeType_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ShapeType_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::ShapeType_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::kortex_driver::ShapeType_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_SHAPETYPE_H
