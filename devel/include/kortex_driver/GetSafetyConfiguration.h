// Generated by gencpp from file kortex_driver/GetSafetyConfiguration.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETSAFETYCONFIGURATION_H
#define KORTEX_DRIVER_MESSAGE_GETSAFETYCONFIGURATION_H

#include <ros/service_traits.h>


#include <kortex_driver/GetSafetyConfigurationRequest.h>
#include <kortex_driver/GetSafetyConfigurationResponse.h>


namespace kortex_driver
{

struct GetSafetyConfiguration
{

typedef GetSafetyConfigurationRequest Request;
typedef GetSafetyConfigurationResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetSafetyConfiguration
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::GetSafetyConfiguration > {
  static const char* value()
  {
    return "943b22eac92db7c1fc8c0b5bbdfb1565";
  }

  static const char* value(const ::kortex_driver::GetSafetyConfiguration&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::GetSafetyConfiguration > {
  static const char* value()
  {
    return "kortex_driver/GetSafetyConfiguration";
  }

  static const char* value(const ::kortex_driver::GetSafetyConfiguration&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::GetSafetyConfigurationRequest> should match
// service_traits::MD5Sum< ::kortex_driver::GetSafetyConfiguration >
template<>
struct MD5Sum< ::kortex_driver::GetSafetyConfigurationRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetSafetyConfiguration >::value();
  }
  static const char* value(const ::kortex_driver::GetSafetyConfigurationRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetSafetyConfigurationRequest> should match
// service_traits::DataType< ::kortex_driver::GetSafetyConfiguration >
template<>
struct DataType< ::kortex_driver::GetSafetyConfigurationRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetSafetyConfiguration >::value();
  }
  static const char* value(const ::kortex_driver::GetSafetyConfigurationRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::GetSafetyConfigurationResponse> should match
// service_traits::MD5Sum< ::kortex_driver::GetSafetyConfiguration >
template<>
struct MD5Sum< ::kortex_driver::GetSafetyConfigurationResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetSafetyConfiguration >::value();
  }
  static const char* value(const ::kortex_driver::GetSafetyConfigurationResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetSafetyConfigurationResponse> should match
// service_traits::DataType< ::kortex_driver::GetSafetyConfiguration >
template<>
struct DataType< ::kortex_driver::GetSafetyConfigurationResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetSafetyConfiguration >::value();
  }
  static const char* value(const ::kortex_driver::GetSafetyConfigurationResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETSAFETYCONFIGURATION_H
