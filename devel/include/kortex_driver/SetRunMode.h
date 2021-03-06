// Generated by gencpp from file kortex_driver/SetRunMode.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_SETRUNMODE_H
#define KORTEX_DRIVER_MESSAGE_SETRUNMODE_H

#include <ros/service_traits.h>


#include <kortex_driver/SetRunModeRequest.h>
#include <kortex_driver/SetRunModeResponse.h>


namespace kortex_driver
{

struct SetRunMode
{

typedef SetRunModeRequest Request;
typedef SetRunModeResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetRunMode
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::SetRunMode > {
  static const char* value()
  {
    return "9b8a67627c37962f119f97d74affc2cb";
  }

  static const char* value(const ::kortex_driver::SetRunMode&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::SetRunMode > {
  static const char* value()
  {
    return "kortex_driver/SetRunMode";
  }

  static const char* value(const ::kortex_driver::SetRunMode&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::SetRunModeRequest> should match
// service_traits::MD5Sum< ::kortex_driver::SetRunMode >
template<>
struct MD5Sum< ::kortex_driver::SetRunModeRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SetRunMode >::value();
  }
  static const char* value(const ::kortex_driver::SetRunModeRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SetRunModeRequest> should match
// service_traits::DataType< ::kortex_driver::SetRunMode >
template<>
struct DataType< ::kortex_driver::SetRunModeRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SetRunMode >::value();
  }
  static const char* value(const ::kortex_driver::SetRunModeRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::SetRunModeResponse> should match
// service_traits::MD5Sum< ::kortex_driver::SetRunMode >
template<>
struct MD5Sum< ::kortex_driver::SetRunModeResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SetRunMode >::value();
  }
  static const char* value(const ::kortex_driver::SetRunModeResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SetRunModeResponse> should match
// service_traits::DataType< ::kortex_driver::SetRunMode >
template<>
struct DataType< ::kortex_driver::SetRunModeResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SetRunMode >::value();
  }
  static const char* value(const ::kortex_driver::SetRunModeResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_SETRUNMODE_H
