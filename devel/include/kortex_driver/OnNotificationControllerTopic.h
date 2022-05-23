// Generated by gencpp from file kortex_driver/OnNotificationControllerTopic.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_ONNOTIFICATIONCONTROLLERTOPIC_H
#define KORTEX_DRIVER_MESSAGE_ONNOTIFICATIONCONTROLLERTOPIC_H

#include <ros/service_traits.h>


#include <kortex_driver/OnNotificationControllerTopicRequest.h>
#include <kortex_driver/OnNotificationControllerTopicResponse.h>


namespace kortex_driver
{

struct OnNotificationControllerTopic
{

typedef OnNotificationControllerTopicRequest Request;
typedef OnNotificationControllerTopicResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct OnNotificationControllerTopic
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::OnNotificationControllerTopic > {
  static const char* value()
  {
    return "6fefdd07c6cb63a94f7b48e7e07e815b";
  }

  static const char* value(const ::kortex_driver::OnNotificationControllerTopic&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::OnNotificationControllerTopic > {
  static const char* value()
  {
    return "kortex_driver/OnNotificationControllerTopic";
  }

  static const char* value(const ::kortex_driver::OnNotificationControllerTopic&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::OnNotificationControllerTopicRequest> should match
// service_traits::MD5Sum< ::kortex_driver::OnNotificationControllerTopic >
template<>
struct MD5Sum< ::kortex_driver::OnNotificationControllerTopicRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::OnNotificationControllerTopic >::value();
  }
  static const char* value(const ::kortex_driver::OnNotificationControllerTopicRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::OnNotificationControllerTopicRequest> should match
// service_traits::DataType< ::kortex_driver::OnNotificationControllerTopic >
template<>
struct DataType< ::kortex_driver::OnNotificationControllerTopicRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::OnNotificationControllerTopic >::value();
  }
  static const char* value(const ::kortex_driver::OnNotificationControllerTopicRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::OnNotificationControllerTopicResponse> should match
// service_traits::MD5Sum< ::kortex_driver::OnNotificationControllerTopic >
template<>
struct MD5Sum< ::kortex_driver::OnNotificationControllerTopicResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::OnNotificationControllerTopic >::value();
  }
  static const char* value(const ::kortex_driver::OnNotificationControllerTopicResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::OnNotificationControllerTopicResponse> should match
// service_traits::DataType< ::kortex_driver::OnNotificationControllerTopic >
template<>
struct DataType< ::kortex_driver::OnNotificationControllerTopicResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::OnNotificationControllerTopic >::value();
  }
  static const char* value(const ::kortex_driver::OnNotificationControllerTopicResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_ONNOTIFICATIONCONTROLLERTOPIC_H
