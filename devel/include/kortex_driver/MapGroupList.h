// Generated by gencpp from file kortex_driver/MapGroupList.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_MAPGROUPLIST_H
#define KORTEX_DRIVER_MESSAGE_MAPGROUPLIST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/MapGroup.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct MapGroupList_
{
  typedef MapGroupList_<ContainerAllocator> Type;

  MapGroupList_()
    : map_groups()  {
    }
  MapGroupList_(const ContainerAllocator& _alloc)
    : map_groups(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::kortex_driver::MapGroup_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::kortex_driver::MapGroup_<ContainerAllocator> >::other >  _map_groups_type;
  _map_groups_type map_groups;





  typedef boost::shared_ptr< ::kortex_driver::MapGroupList_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::MapGroupList_<ContainerAllocator> const> ConstPtr;

}; // struct MapGroupList_

typedef ::kortex_driver::MapGroupList_<std::allocator<void> > MapGroupList;

typedef boost::shared_ptr< ::kortex_driver::MapGroupList > MapGroupListPtr;
typedef boost::shared_ptr< ::kortex_driver::MapGroupList const> MapGroupListConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::MapGroupList_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::MapGroupList_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::MapGroupList_<ContainerAllocator1> & lhs, const ::kortex_driver::MapGroupList_<ContainerAllocator2> & rhs)
{
  return lhs.map_groups == rhs.map_groups;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::MapGroupList_<ContainerAllocator1> & lhs, const ::kortex_driver::MapGroupList_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::MapGroupList_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::MapGroupList_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::MapGroupList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::MapGroupList_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::MapGroupList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::MapGroupList_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::MapGroupList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "60049e6988ccab2e80e3d3cdf02a81da";
  }

  static const char* value(const ::kortex_driver::MapGroupList_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x60049e6988ccab2eULL;
  static const uint64_t static_value2 = 0x80e3d3cdf02a81daULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::MapGroupList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/MapGroupList";
  }

  static const char* value(const ::kortex_driver::MapGroupList_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::MapGroupList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"MapGroup[] map_groups\n"
"================================================================================\n"
"MSG: kortex_driver/MapGroup\n"
"\n"
"MapGroupHandle group_handle\n"
"string name\n"
"MappingHandle related_mapping_handle\n"
"MapGroupHandle parent_group_handle\n"
"MapGroupHandle[] children_map_group_handles\n"
"MapHandle[] map_handles\n"
"string application_data\n"
"================================================================================\n"
"MSG: kortex_driver/MapGroupHandle\n"
"\n"
"uint32 identifier\n"
"uint32 permission\n"
"================================================================================\n"
"MSG: kortex_driver/MappingHandle\n"
"\n"
"uint32 identifier\n"
"uint32 permission\n"
"================================================================================\n"
"MSG: kortex_driver/MapHandle\n"
"\n"
"uint32 identifier\n"
"uint32 permission\n"
;
  }

  static const char* value(const ::kortex_driver::MapGroupList_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::MapGroupList_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.map_groups);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MapGroupList_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::MapGroupList_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::MapGroupList_<ContainerAllocator>& v)
  {
    s << indent << "map_groups[]" << std::endl;
    for (size_t i = 0; i < v.map_groups.size(); ++i)
    {
      s << indent << "  map_groups[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::kortex_driver::MapGroup_<ContainerAllocator> >::stream(s, indent + "    ", v.map_groups[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_MAPGROUPLIST_H
