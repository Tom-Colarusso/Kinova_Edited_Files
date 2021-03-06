// Generated by gencpp from file kortex_driver/SetSafetyEnable.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_SETSAFETYENABLE_H
#define KORTEX_DRIVER_MESSAGE_SETSAFETYENABLE_H

#include <ros/service_traits.h>


#include <kortex_driver/SetSafetyEnableRequest.h>
#include <kortex_driver/SetSafetyEnableResponse.h>


namespace kortex_driver
{

struct SetSafetyEnable
{

typedef SetSafetyEnableRequest Request;
typedef SetSafetyEnableResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetSafetyEnable
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::SetSafetyEnable > {
  static const char* value()
  {
    return "81bf2032f72f340390e3e57ea5d159db";
  }

  static const char* value(const ::kortex_driver::SetSafetyEnable&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::SetSafetyEnable > {
  static const char* value()
  {
    return "kortex_driver/SetSafetyEnable";
  }

  static const char* value(const ::kortex_driver::SetSafetyEnable&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::SetSafetyEnableRequest> should match
// service_traits::MD5Sum< ::kortex_driver::SetSafetyEnable >
template<>
struct MD5Sum< ::kortex_driver::SetSafetyEnableRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SetSafetyEnable >::value();
  }
  static const char* value(const ::kortex_driver::SetSafetyEnableRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SetSafetyEnableRequest> should match
// service_traits::DataType< ::kortex_driver::SetSafetyEnable >
template<>
struct DataType< ::kortex_driver::SetSafetyEnableRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SetSafetyEnable >::value();
  }
  static const char* value(const ::kortex_driver::SetSafetyEnableRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::SetSafetyEnableResponse> should match
// service_traits::MD5Sum< ::kortex_driver::SetSafetyEnable >
template<>
struct MD5Sum< ::kortex_driver::SetSafetyEnableResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SetSafetyEnable >::value();
  }
  static const char* value(const ::kortex_driver::SetSafetyEnableResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SetSafetyEnableResponse> should match
// service_traits::DataType< ::kortex_driver::SetSafetyEnable >
template<>
struct DataType< ::kortex_driver::SetSafetyEnableResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SetSafetyEnable >::value();
  }
  static const char* value(const ::kortex_driver::SetSafetyEnableResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_SETSAFETYENABLE_H
