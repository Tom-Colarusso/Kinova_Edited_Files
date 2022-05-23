// Generated by gencpp from file kortex_driver/SequenceInfoNotification.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_SEQUENCEINFONOTIFICATION_H
#define KORTEX_DRIVER_MESSAGE_SEQUENCEINFONOTIFICATION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/SequenceHandle.h>
#include <kortex_driver/Timestamp.h>
#include <kortex_driver/UserProfileHandle.h>
#include <kortex_driver/Connection.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct SequenceInfoNotification_
{
  typedef SequenceInfoNotification_<ContainerAllocator> Type;

  SequenceInfoNotification_()
    : event_identifier(0)
    , sequence_handle()
    , task_index(0)
    , group_identifier(0)
    , timestamp()
    , user_handle()
    , abort_details(0)
    , connection()  {
    }
  SequenceInfoNotification_(const ContainerAllocator& _alloc)
    : event_identifier(0)
    , sequence_handle(_alloc)
    , task_index(0)
    , group_identifier(0)
    , timestamp(_alloc)
    , user_handle(_alloc)
    , abort_details(0)
    , connection(_alloc)  {
  (void)_alloc;
    }



   typedef uint32_t _event_identifier_type;
  _event_identifier_type event_identifier;

   typedef  ::kortex_driver::SequenceHandle_<ContainerAllocator>  _sequence_handle_type;
  _sequence_handle_type sequence_handle;

   typedef uint32_t _task_index_type;
  _task_index_type task_index;

   typedef uint32_t _group_identifier_type;
  _group_identifier_type group_identifier;

   typedef  ::kortex_driver::Timestamp_<ContainerAllocator>  _timestamp_type;
  _timestamp_type timestamp;

   typedef  ::kortex_driver::UserProfileHandle_<ContainerAllocator>  _user_handle_type;
  _user_handle_type user_handle;

   typedef uint32_t _abort_details_type;
  _abort_details_type abort_details;

   typedef  ::kortex_driver::Connection_<ContainerAllocator>  _connection_type;
  _connection_type connection;





  typedef boost::shared_ptr< ::kortex_driver::SequenceInfoNotification_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::SequenceInfoNotification_<ContainerAllocator> const> ConstPtr;

}; // struct SequenceInfoNotification_

typedef ::kortex_driver::SequenceInfoNotification_<std::allocator<void> > SequenceInfoNotification;

typedef boost::shared_ptr< ::kortex_driver::SequenceInfoNotification > SequenceInfoNotificationPtr;
typedef boost::shared_ptr< ::kortex_driver::SequenceInfoNotification const> SequenceInfoNotificationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::SequenceInfoNotification_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::SequenceInfoNotification_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::SequenceInfoNotification_<ContainerAllocator1> & lhs, const ::kortex_driver::SequenceInfoNotification_<ContainerAllocator2> & rhs)
{
  return lhs.event_identifier == rhs.event_identifier &&
    lhs.sequence_handle == rhs.sequence_handle &&
    lhs.task_index == rhs.task_index &&
    lhs.group_identifier == rhs.group_identifier &&
    lhs.timestamp == rhs.timestamp &&
    lhs.user_handle == rhs.user_handle &&
    lhs.abort_details == rhs.abort_details &&
    lhs.connection == rhs.connection;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::SequenceInfoNotification_<ContainerAllocator1> & lhs, const ::kortex_driver::SequenceInfoNotification_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::SequenceInfoNotification_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::SequenceInfoNotification_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::SequenceInfoNotification_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::SequenceInfoNotification_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::SequenceInfoNotification_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::SequenceInfoNotification_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::SequenceInfoNotification_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cfd9f46ae5ef4976e20a716c9bac5aef";
  }

  static const char* value(const ::kortex_driver::SequenceInfoNotification_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcfd9f46ae5ef4976ULL;
  static const uint64_t static_value2 = 0xe20a716c9bac5aefULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::SequenceInfoNotification_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/SequenceInfoNotification";
  }

  static const char* value(const ::kortex_driver::SequenceInfoNotification_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::SequenceInfoNotification_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 event_identifier\n"
"SequenceHandle sequence_handle\n"
"uint32 task_index\n"
"uint32 group_identifier\n"
"Timestamp timestamp\n"
"UserProfileHandle user_handle\n"
"uint32 abort_details\n"
"Connection connection\n"
"================================================================================\n"
"MSG: kortex_driver/SequenceHandle\n"
"\n"
"uint32 identifier\n"
"uint32 permission\n"
"================================================================================\n"
"MSG: kortex_driver/Timestamp\n"
"\n"
"uint32 sec\n"
"uint32 usec\n"
"================================================================================\n"
"MSG: kortex_driver/UserProfileHandle\n"
"\n"
"uint32 identifier\n"
"uint32 permission\n"
"================================================================================\n"
"MSG: kortex_driver/Connection\n"
"\n"
"UserProfileHandle user_handle\n"
"string connection_information\n"
"uint32 connection_identifier\n"
;
  }

  static const char* value(const ::kortex_driver::SequenceInfoNotification_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::SequenceInfoNotification_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.event_identifier);
      stream.next(m.sequence_handle);
      stream.next(m.task_index);
      stream.next(m.group_identifier);
      stream.next(m.timestamp);
      stream.next(m.user_handle);
      stream.next(m.abort_details);
      stream.next(m.connection);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SequenceInfoNotification_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::SequenceInfoNotification_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::SequenceInfoNotification_<ContainerAllocator>& v)
  {
    s << indent << "event_identifier: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.event_identifier);
    s << indent << "sequence_handle: ";
    s << std::endl;
    Printer< ::kortex_driver::SequenceHandle_<ContainerAllocator> >::stream(s, indent + "  ", v.sequence_handle);
    s << indent << "task_index: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.task_index);
    s << indent << "group_identifier: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.group_identifier);
    s << indent << "timestamp: ";
    s << std::endl;
    Printer< ::kortex_driver::Timestamp_<ContainerAllocator> >::stream(s, indent + "  ", v.timestamp);
    s << indent << "user_handle: ";
    s << std::endl;
    Printer< ::kortex_driver::UserProfileHandle_<ContainerAllocator> >::stream(s, indent + "  ", v.user_handle);
    s << indent << "abort_details: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.abort_details);
    s << indent << "connection: ";
    s << std::endl;
    Printer< ::kortex_driver::Connection_<ContainerAllocator> >::stream(s, indent + "  ", v.connection);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_SEQUENCEINFONOTIFICATION_H
