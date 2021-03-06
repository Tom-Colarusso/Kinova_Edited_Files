// Generated by gencpp from file kortex_driver/SetSafetyErrorThreshold.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_SETSAFETYERRORTHRESHOLD_H
#define KORTEX_DRIVER_MESSAGE_SETSAFETYERRORTHRESHOLD_H

#include <ros/service_traits.h>


#include <kortex_driver/SetSafetyErrorThresholdRequest.h>
#include <kortex_driver/SetSafetyErrorThresholdResponse.h>


namespace kortex_driver
{

struct SetSafetyErrorThreshold
{

typedef SetSafetyErrorThresholdRequest Request;
typedef SetSafetyErrorThresholdResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetSafetyErrorThreshold
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::SetSafetyErrorThreshold > {
  static const char* value()
  {
    return "0d6527f8c0e63583f63fafeb1405804a";
  }

  static const char* value(const ::kortex_driver::SetSafetyErrorThreshold&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::SetSafetyErrorThreshold > {
  static const char* value()
  {
    return "kortex_driver/SetSafetyErrorThreshold";
  }

  static const char* value(const ::kortex_driver::SetSafetyErrorThreshold&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::SetSafetyErrorThresholdRequest> should match
// service_traits::MD5Sum< ::kortex_driver::SetSafetyErrorThreshold >
template<>
struct MD5Sum< ::kortex_driver::SetSafetyErrorThresholdRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SetSafetyErrorThreshold >::value();
  }
  static const char* value(const ::kortex_driver::SetSafetyErrorThresholdRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SetSafetyErrorThresholdRequest> should match
// service_traits::DataType< ::kortex_driver::SetSafetyErrorThreshold >
template<>
struct DataType< ::kortex_driver::SetSafetyErrorThresholdRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SetSafetyErrorThreshold >::value();
  }
  static const char* value(const ::kortex_driver::SetSafetyErrorThresholdRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::SetSafetyErrorThresholdResponse> should match
// service_traits::MD5Sum< ::kortex_driver::SetSafetyErrorThreshold >
template<>
struct MD5Sum< ::kortex_driver::SetSafetyErrorThresholdResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SetSafetyErrorThreshold >::value();
  }
  static const char* value(const ::kortex_driver::SetSafetyErrorThresholdResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SetSafetyErrorThresholdResponse> should match
// service_traits::DataType< ::kortex_driver::SetSafetyErrorThreshold >
template<>
struct DataType< ::kortex_driver::SetSafetyErrorThresholdResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SetSafetyErrorThreshold >::value();
  }
  static const char* value(const ::kortex_driver::SetSafetyErrorThresholdResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_SETSAFETYERRORTHRESHOLD_H
