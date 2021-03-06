// Generated by gencpp from file kortex_driver/SetCoggingFeedforwardMode.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_SETCOGGINGFEEDFORWARDMODE_H
#define KORTEX_DRIVER_MESSAGE_SETCOGGINGFEEDFORWARDMODE_H

#include <ros/service_traits.h>


#include <kortex_driver/SetCoggingFeedforwardModeRequest.h>
#include <kortex_driver/SetCoggingFeedforwardModeResponse.h>


namespace kortex_driver
{

struct SetCoggingFeedforwardMode
{

typedef SetCoggingFeedforwardModeRequest Request;
typedef SetCoggingFeedforwardModeResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetCoggingFeedforwardMode
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::SetCoggingFeedforwardMode > {
  static const char* value()
  {
    return "156e5a43117ffd12af553796e457d579";
  }

  static const char* value(const ::kortex_driver::SetCoggingFeedforwardMode&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::SetCoggingFeedforwardMode > {
  static const char* value()
  {
    return "kortex_driver/SetCoggingFeedforwardMode";
  }

  static const char* value(const ::kortex_driver::SetCoggingFeedforwardMode&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::SetCoggingFeedforwardModeRequest> should match
// service_traits::MD5Sum< ::kortex_driver::SetCoggingFeedforwardMode >
template<>
struct MD5Sum< ::kortex_driver::SetCoggingFeedforwardModeRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SetCoggingFeedforwardMode >::value();
  }
  static const char* value(const ::kortex_driver::SetCoggingFeedforwardModeRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SetCoggingFeedforwardModeRequest> should match
// service_traits::DataType< ::kortex_driver::SetCoggingFeedforwardMode >
template<>
struct DataType< ::kortex_driver::SetCoggingFeedforwardModeRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SetCoggingFeedforwardMode >::value();
  }
  static const char* value(const ::kortex_driver::SetCoggingFeedforwardModeRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::SetCoggingFeedforwardModeResponse> should match
// service_traits::MD5Sum< ::kortex_driver::SetCoggingFeedforwardMode >
template<>
struct MD5Sum< ::kortex_driver::SetCoggingFeedforwardModeResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SetCoggingFeedforwardMode >::value();
  }
  static const char* value(const ::kortex_driver::SetCoggingFeedforwardModeResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SetCoggingFeedforwardModeResponse> should match
// service_traits::DataType< ::kortex_driver::SetCoggingFeedforwardMode >
template<>
struct DataType< ::kortex_driver::SetCoggingFeedforwardModeResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SetCoggingFeedforwardMode >::value();
  }
  static const char* value(const ::kortex_driver::SetCoggingFeedforwardModeResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_SETCOGGINGFEEDFORWARDMODE_H
