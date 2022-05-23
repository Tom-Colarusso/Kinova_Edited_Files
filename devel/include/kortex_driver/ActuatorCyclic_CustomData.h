// Generated by gencpp from file kortex_driver/ActuatorCyclic_CustomData.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_ACTUATORCYCLIC_CUSTOMDATA_H
#define KORTEX_DRIVER_MESSAGE_ACTUATORCYCLIC_CUSTOMDATA_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/ActuatorCyclic_MessageId.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct ActuatorCyclic_CustomData_
{
  typedef ActuatorCyclic_CustomData_<ContainerAllocator> Type;

  ActuatorCyclic_CustomData_()
    : custom_data_id()
    , custom_data_0(0)
    , custom_data_1(0)
    , custom_data_2(0)
    , custom_data_3(0)
    , custom_data_4(0)
    , custom_data_5(0)
    , custom_data_6(0)
    , custom_data_7(0)
    , custom_data_8(0)
    , custom_data_9(0)
    , custom_data_10(0)
    , custom_data_11(0)
    , custom_data_12(0)
    , custom_data_13(0)
    , custom_data_14(0)
    , custom_data_15(0)  {
    }
  ActuatorCyclic_CustomData_(const ContainerAllocator& _alloc)
    : custom_data_id(_alloc)
    , custom_data_0(0)
    , custom_data_1(0)
    , custom_data_2(0)
    , custom_data_3(0)
    , custom_data_4(0)
    , custom_data_5(0)
    , custom_data_6(0)
    , custom_data_7(0)
    , custom_data_8(0)
    , custom_data_9(0)
    , custom_data_10(0)
    , custom_data_11(0)
    , custom_data_12(0)
    , custom_data_13(0)
    , custom_data_14(0)
    , custom_data_15(0)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::ActuatorCyclic_MessageId_<ContainerAllocator>  _custom_data_id_type;
  _custom_data_id_type custom_data_id;

   typedef uint32_t _custom_data_0_type;
  _custom_data_0_type custom_data_0;

   typedef uint32_t _custom_data_1_type;
  _custom_data_1_type custom_data_1;

   typedef uint32_t _custom_data_2_type;
  _custom_data_2_type custom_data_2;

   typedef uint32_t _custom_data_3_type;
  _custom_data_3_type custom_data_3;

   typedef uint32_t _custom_data_4_type;
  _custom_data_4_type custom_data_4;

   typedef uint32_t _custom_data_5_type;
  _custom_data_5_type custom_data_5;

   typedef uint32_t _custom_data_6_type;
  _custom_data_6_type custom_data_6;

   typedef uint32_t _custom_data_7_type;
  _custom_data_7_type custom_data_7;

   typedef uint32_t _custom_data_8_type;
  _custom_data_8_type custom_data_8;

   typedef uint32_t _custom_data_9_type;
  _custom_data_9_type custom_data_9;

   typedef uint32_t _custom_data_10_type;
  _custom_data_10_type custom_data_10;

   typedef uint32_t _custom_data_11_type;
  _custom_data_11_type custom_data_11;

   typedef uint32_t _custom_data_12_type;
  _custom_data_12_type custom_data_12;

   typedef uint32_t _custom_data_13_type;
  _custom_data_13_type custom_data_13;

   typedef uint32_t _custom_data_14_type;
  _custom_data_14_type custom_data_14;

   typedef uint32_t _custom_data_15_type;
  _custom_data_15_type custom_data_15;





  typedef boost::shared_ptr< ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator> const> ConstPtr;

}; // struct ActuatorCyclic_CustomData_

typedef ::kortex_driver::ActuatorCyclic_CustomData_<std::allocator<void> > ActuatorCyclic_CustomData;

typedef boost::shared_ptr< ::kortex_driver::ActuatorCyclic_CustomData > ActuatorCyclic_CustomDataPtr;
typedef boost::shared_ptr< ::kortex_driver::ActuatorCyclic_CustomData const> ActuatorCyclic_CustomDataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator1> & lhs, const ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator2> & rhs)
{
  return lhs.custom_data_id == rhs.custom_data_id &&
    lhs.custom_data_0 == rhs.custom_data_0 &&
    lhs.custom_data_1 == rhs.custom_data_1 &&
    lhs.custom_data_2 == rhs.custom_data_2 &&
    lhs.custom_data_3 == rhs.custom_data_3 &&
    lhs.custom_data_4 == rhs.custom_data_4 &&
    lhs.custom_data_5 == rhs.custom_data_5 &&
    lhs.custom_data_6 == rhs.custom_data_6 &&
    lhs.custom_data_7 == rhs.custom_data_7 &&
    lhs.custom_data_8 == rhs.custom_data_8 &&
    lhs.custom_data_9 == rhs.custom_data_9 &&
    lhs.custom_data_10 == rhs.custom_data_10 &&
    lhs.custom_data_11 == rhs.custom_data_11 &&
    lhs.custom_data_12 == rhs.custom_data_12 &&
    lhs.custom_data_13 == rhs.custom_data_13 &&
    lhs.custom_data_14 == rhs.custom_data_14 &&
    lhs.custom_data_15 == rhs.custom_data_15;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator1> & lhs, const ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "05eca88f613f992646a3cd017b934eba";
  }

  static const char* value(const ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x05eca88f613f9926ULL;
  static const uint64_t static_value2 = 0x46a3cd017b934ebaULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/ActuatorCyclic_CustomData";
  }

  static const char* value(const ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"ActuatorCyclic_MessageId custom_data_id\n"
"uint32 custom_data_0\n"
"uint32 custom_data_1\n"
"uint32 custom_data_2\n"
"uint32 custom_data_3\n"
"uint32 custom_data_4\n"
"uint32 custom_data_5\n"
"uint32 custom_data_6\n"
"uint32 custom_data_7\n"
"uint32 custom_data_8\n"
"uint32 custom_data_9\n"
"uint32 custom_data_10\n"
"uint32 custom_data_11\n"
"uint32 custom_data_12\n"
"uint32 custom_data_13\n"
"uint32 custom_data_14\n"
"uint32 custom_data_15\n"
"================================================================================\n"
"MSG: kortex_driver/ActuatorCyclic_MessageId\n"
"\n"
"uint32 identifier\n"
;
  }

  static const char* value(const ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.custom_data_id);
      stream.next(m.custom_data_0);
      stream.next(m.custom_data_1);
      stream.next(m.custom_data_2);
      stream.next(m.custom_data_3);
      stream.next(m.custom_data_4);
      stream.next(m.custom_data_5);
      stream.next(m.custom_data_6);
      stream.next(m.custom_data_7);
      stream.next(m.custom_data_8);
      stream.next(m.custom_data_9);
      stream.next(m.custom_data_10);
      stream.next(m.custom_data_11);
      stream.next(m.custom_data_12);
      stream.next(m.custom_data_13);
      stream.next(m.custom_data_14);
      stream.next(m.custom_data_15);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ActuatorCyclic_CustomData_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::ActuatorCyclic_CustomData_<ContainerAllocator>& v)
  {
    s << indent << "custom_data_id: ";
    s << std::endl;
    Printer< ::kortex_driver::ActuatorCyclic_MessageId_<ContainerAllocator> >::stream(s, indent + "  ", v.custom_data_id);
    s << indent << "custom_data_0: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.custom_data_0);
    s << indent << "custom_data_1: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.custom_data_1);
    s << indent << "custom_data_2: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.custom_data_2);
    s << indent << "custom_data_3: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.custom_data_3);
    s << indent << "custom_data_4: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.custom_data_4);
    s << indent << "custom_data_5: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.custom_data_5);
    s << indent << "custom_data_6: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.custom_data_6);
    s << indent << "custom_data_7: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.custom_data_7);
    s << indent << "custom_data_8: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.custom_data_8);
    s << indent << "custom_data_9: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.custom_data_9);
    s << indent << "custom_data_10: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.custom_data_10);
    s << indent << "custom_data_11: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.custom_data_11);
    s << indent << "custom_data_12: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.custom_data_12);
    s << indent << "custom_data_13: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.custom_data_13);
    s << indent << "custom_data_14: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.custom_data_14);
    s << indent << "custom_data_15: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.custom_data_15);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_ACTUATORCYCLIC_CUSTOMDATA_H
