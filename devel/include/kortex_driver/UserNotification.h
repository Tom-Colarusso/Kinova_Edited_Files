// Generated by gencpp from file kortex_driver/UserNotification.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_USERNOTIFICATION_H
#define KORTEX_DRIVER_MESSAGE_USERNOTIFICATION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/UserProfileHandle.h>
#include <kortex_driver/Timestamp.h>
#include <kortex_driver/UserProfileHandle.h>
#include <kortex_driver/Connection.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct UserNotification_
{
  typedef UserNotification_<ContainerAllocator> Type;

  UserNotification_()
    : user_event(0)
    , modified_user()
    , timestamp()
    , user_handle()
    , connection()  {
    }
  UserNotification_(const ContainerAllocator& _alloc)
    : user_event(0)
    , modified_user(_alloc)
    , timestamp(_alloc)
    , user_handle(_alloc)
    , connection(_alloc)  {
  (void)_alloc;
    }



   typedef uint32_t _user_event_type;
  _user_event_type user_event;

   typedef  ::kortex_driver::UserProfileHandle_<ContainerAllocator>  _modified_user_type;
  _modified_user_type modified_user;

   typedef  ::kortex_driver::Timestamp_<ContainerAllocator>  _timestamp_type;
  _timestamp_type timestamp;

   typedef  ::kortex_driver::UserProfileHandle_<ContainerAllocator>  _user_handle_type;
  _user_handle_type user_handle;

   typedef  ::kortex_driver::Connection_<ContainerAllocator>  _connection_type;
  _connection_type connection;





  typedef boost::shared_ptr< ::kortex_driver::UserNotification_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::UserNotification_<ContainerAllocator> const> ConstPtr;

}; // struct UserNotification_

typedef ::kortex_driver::UserNotification_<std::allocator<void> > UserNotification;

typedef boost::shared_ptr< ::kortex_driver::UserNotification > UserNotificationPtr;
typedef boost::shared_ptr< ::kortex_driver::UserNotification const> UserNotificationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::UserNotification_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::UserNotification_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::UserNotification_<ContainerAllocator1> & lhs, const ::kortex_driver::UserNotification_<ContainerAllocator2> & rhs)
{
  return lhs.user_event == rhs.user_event &&
    lhs.modified_user == rhs.modified_user &&
    lhs.timestamp == rhs.timestamp &&
    lhs.user_handle == rhs.user_handle &&
    lhs.connection == rhs.connection;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::UserNotification_<ContainerAllocator1> & lhs, const ::kortex_driver::UserNotification_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::UserNotification_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::UserNotification_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::UserNotification_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::UserNotification_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::UserNotification_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::UserNotification_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::UserNotification_<ContainerAllocator> >
{
  static const char* value()
  {
    return "deba2e21a3d1183442bf7c264989e80c";
  }

  static const char* value(const ::kortex_driver::UserNotification_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdeba2e21a3d11834ULL;
  static const uint64_t static_value2 = 0x42bf7c264989e80cULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::UserNotification_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/UserNotification";
  }

  static const char* value(const ::kortex_driver::UserNotification_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::UserNotification_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 user_event\n"
"UserProfileHandle modified_user\n"
"Timestamp timestamp\n"
"UserProfileHandle user_handle\n"
"Connection connection\n"
"================================================================================\n"
"MSG: kortex_driver/UserProfileHandle\n"
"\n"
"uint32 identifier\n"
"uint32 permission\n"
"================================================================================\n"
"MSG: kortex_driver/Timestamp\n"
"\n"
"uint32 sec\n"
"uint32 usec\n"
"================================================================================\n"
"MSG: kortex_driver/Connection\n"
"\n"
"UserProfileHandle user_handle\n"
"string connection_information\n"
"uint32 connection_identifier\n"
;
  }

  static const char* value(const ::kortex_driver::UserNotification_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::UserNotification_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.user_event);
      stream.next(m.modified_user);
      stream.next(m.timestamp);
      stream.next(m.user_handle);
      stream.next(m.connection);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct UserNotification_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::UserNotification_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::UserNotification_<ContainerAllocator>& v)
  {
    s << indent << "user_event: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.user_event);
    s << indent << "modified_user: ";
    s << std::endl;
    Printer< ::kortex_driver::UserProfileHandle_<ContainerAllocator> >::stream(s, indent + "  ", v.modified_user);
    s << indent << "timestamp: ";
    s << std::endl;
    Printer< ::kortex_driver::Timestamp_<ContainerAllocator> >::stream(s, indent + "  ", v.timestamp);
    s << indent << "user_handle: ";
    s << std::endl;
    Printer< ::kortex_driver::UserProfileHandle_<ContainerAllocator> >::stream(s, indent + "  ", v.user_handle);
    s << indent << "connection: ";
    s << std::endl;
    Printer< ::kortex_driver::Connection_<ContainerAllocator> >::stream(s, indent + "  ", v.connection);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_USERNOTIFICATION_H
