// Generated by gencpp from file kortex_driver/GetRunMode.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETRUNMODE_H
#define KORTEX_DRIVER_MESSAGE_GETRUNMODE_H

#include <ros/service_traits.h>


#include <kortex_driver/GetRunModeRequest.h>
#include <kortex_driver/GetRunModeResponse.h>


namespace kortex_driver
{

struct GetRunMode
{

typedef GetRunModeRequest Request;
typedef GetRunModeResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetRunMode
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::GetRunMode > {
  static const char* value()
  {
    return "dd5c8711b1fd2894a1cd52ad67ed6ff2";
  }

  static const char* value(const ::kortex_driver::GetRunMode&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::GetRunMode > {
  static const char* value()
  {
    return "kortex_driver/GetRunMode";
  }

  static const char* value(const ::kortex_driver::GetRunMode&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::GetRunModeRequest> should match
// service_traits::MD5Sum< ::kortex_driver::GetRunMode >
template<>
struct MD5Sum< ::kortex_driver::GetRunModeRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetRunMode >::value();
  }
  static const char* value(const ::kortex_driver::GetRunModeRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetRunModeRequest> should match
// service_traits::DataType< ::kortex_driver::GetRunMode >
template<>
struct DataType< ::kortex_driver::GetRunModeRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetRunMode >::value();
  }
  static const char* value(const ::kortex_driver::GetRunModeRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::GetRunModeResponse> should match
// service_traits::MD5Sum< ::kortex_driver::GetRunMode >
template<>
struct MD5Sum< ::kortex_driver::GetRunModeResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetRunMode >::value();
  }
  static const char* value(const ::kortex_driver::GetRunModeResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetRunModeResponse> should match
// service_traits::DataType< ::kortex_driver::GetRunMode >
template<>
struct DataType< ::kortex_driver::GetRunModeResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetRunMode >::value();
  }
  static const char* value(const ::kortex_driver::GetRunModeResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETRUNMODE_H
