// Generated by gencpp from file kortex_driver/GetAllConfiguredWifis.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETALLCONFIGUREDWIFIS_H
#define KORTEX_DRIVER_MESSAGE_GETALLCONFIGUREDWIFIS_H

#include <ros/service_traits.h>


#include <kortex_driver/GetAllConfiguredWifisRequest.h>
#include <kortex_driver/GetAllConfiguredWifisResponse.h>


namespace kortex_driver
{

struct GetAllConfiguredWifis
{

typedef GetAllConfiguredWifisRequest Request;
typedef GetAllConfiguredWifisResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetAllConfiguredWifis
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::GetAllConfiguredWifis > {
  static const char* value()
  {
    return "00cc4429e5b474aecc287723cc4eff69";
  }

  static const char* value(const ::kortex_driver::GetAllConfiguredWifis&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::GetAllConfiguredWifis > {
  static const char* value()
  {
    return "kortex_driver/GetAllConfiguredWifis";
  }

  static const char* value(const ::kortex_driver::GetAllConfiguredWifis&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::GetAllConfiguredWifisRequest> should match
// service_traits::MD5Sum< ::kortex_driver::GetAllConfiguredWifis >
template<>
struct MD5Sum< ::kortex_driver::GetAllConfiguredWifisRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetAllConfiguredWifis >::value();
  }
  static const char* value(const ::kortex_driver::GetAllConfiguredWifisRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetAllConfiguredWifisRequest> should match
// service_traits::DataType< ::kortex_driver::GetAllConfiguredWifis >
template<>
struct DataType< ::kortex_driver::GetAllConfiguredWifisRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetAllConfiguredWifis >::value();
  }
  static const char* value(const ::kortex_driver::GetAllConfiguredWifisRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::GetAllConfiguredWifisResponse> should match
// service_traits::MD5Sum< ::kortex_driver::GetAllConfiguredWifis >
template<>
struct MD5Sum< ::kortex_driver::GetAllConfiguredWifisResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetAllConfiguredWifis >::value();
  }
  static const char* value(const ::kortex_driver::GetAllConfiguredWifisResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetAllConfiguredWifisResponse> should match
// service_traits::DataType< ::kortex_driver::GetAllConfiguredWifis >
template<>
struct DataType< ::kortex_driver::GetAllConfiguredWifisResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetAllConfiguredWifis >::value();
  }
  static const char* value(const ::kortex_driver::GetAllConfiguredWifisResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETALLCONFIGUREDWIFIS_H
