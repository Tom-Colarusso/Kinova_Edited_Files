// Generated by gencpp from file kortex_driver/ReadAllUsers.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_READALLUSERS_H
#define KORTEX_DRIVER_MESSAGE_READALLUSERS_H

#include <ros/service_traits.h>


#include <kortex_driver/ReadAllUsersRequest.h>
#include <kortex_driver/ReadAllUsersResponse.h>


namespace kortex_driver
{

struct ReadAllUsers
{

typedef ReadAllUsersRequest Request;
typedef ReadAllUsersResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ReadAllUsers
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::ReadAllUsers > {
  static const char* value()
  {
    return "f4e7c3e24386fc73fa17c3ea4d2a06c5";
  }

  static const char* value(const ::kortex_driver::ReadAllUsers&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::ReadAllUsers > {
  static const char* value()
  {
    return "kortex_driver/ReadAllUsers";
  }

  static const char* value(const ::kortex_driver::ReadAllUsers&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::ReadAllUsersRequest> should match
// service_traits::MD5Sum< ::kortex_driver::ReadAllUsers >
template<>
struct MD5Sum< ::kortex_driver::ReadAllUsersRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::ReadAllUsers >::value();
  }
  static const char* value(const ::kortex_driver::ReadAllUsersRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::ReadAllUsersRequest> should match
// service_traits::DataType< ::kortex_driver::ReadAllUsers >
template<>
struct DataType< ::kortex_driver::ReadAllUsersRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::ReadAllUsers >::value();
  }
  static const char* value(const ::kortex_driver::ReadAllUsersRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::ReadAllUsersResponse> should match
// service_traits::MD5Sum< ::kortex_driver::ReadAllUsers >
template<>
struct MD5Sum< ::kortex_driver::ReadAllUsersResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::ReadAllUsers >::value();
  }
  static const char* value(const ::kortex_driver::ReadAllUsersResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::ReadAllUsersResponse> should match
// service_traits::DataType< ::kortex_driver::ReadAllUsers >
template<>
struct DataType< ::kortex_driver::ReadAllUsersResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::ReadAllUsers >::value();
  }
  static const char* value(const ::kortex_driver::ReadAllUsersResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_READALLUSERS_H
