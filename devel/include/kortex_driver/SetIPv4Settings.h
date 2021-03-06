// Generated by gencpp from file kortex_driver/SetIPv4Settings.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_SETIPV4SETTINGS_H
#define KORTEX_DRIVER_MESSAGE_SETIPV4SETTINGS_H

#include <ros/service_traits.h>


#include <kortex_driver/SetIPv4SettingsRequest.h>
#include <kortex_driver/SetIPv4SettingsResponse.h>


namespace kortex_driver
{

struct SetIPv4Settings
{

typedef SetIPv4SettingsRequest Request;
typedef SetIPv4SettingsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetIPv4Settings
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::SetIPv4Settings > {
  static const char* value()
  {
    return "361b42ee5a195ec231ecbbedd2098401";
  }

  static const char* value(const ::kortex_driver::SetIPv4Settings&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::SetIPv4Settings > {
  static const char* value()
  {
    return "kortex_driver/SetIPv4Settings";
  }

  static const char* value(const ::kortex_driver::SetIPv4Settings&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::SetIPv4SettingsRequest> should match
// service_traits::MD5Sum< ::kortex_driver::SetIPv4Settings >
template<>
struct MD5Sum< ::kortex_driver::SetIPv4SettingsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SetIPv4Settings >::value();
  }
  static const char* value(const ::kortex_driver::SetIPv4SettingsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SetIPv4SettingsRequest> should match
// service_traits::DataType< ::kortex_driver::SetIPv4Settings >
template<>
struct DataType< ::kortex_driver::SetIPv4SettingsRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SetIPv4Settings >::value();
  }
  static const char* value(const ::kortex_driver::SetIPv4SettingsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::SetIPv4SettingsResponse> should match
// service_traits::MD5Sum< ::kortex_driver::SetIPv4Settings >
template<>
struct MD5Sum< ::kortex_driver::SetIPv4SettingsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SetIPv4Settings >::value();
  }
  static const char* value(const ::kortex_driver::SetIPv4SettingsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SetIPv4SettingsResponse> should match
// service_traits::DataType< ::kortex_driver::SetIPv4Settings >
template<>
struct DataType< ::kortex_driver::SetIPv4SettingsResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SetIPv4Settings >::value();
  }
  static const char* value(const ::kortex_driver::SetIPv4SettingsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_SETIPV4SETTINGS_H
