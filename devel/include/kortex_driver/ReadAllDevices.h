// Generated by gencpp from file kortex_driver/ReadAllDevices.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_READALLDEVICES_H
#define KORTEX_DRIVER_MESSAGE_READALLDEVICES_H

#include <ros/service_traits.h>


#include <kortex_driver/ReadAllDevicesRequest.h>
#include <kortex_driver/ReadAllDevicesResponse.h>


namespace kortex_driver
{

struct ReadAllDevices
{

typedef ReadAllDevicesRequest Request;
typedef ReadAllDevicesResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ReadAllDevices
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::ReadAllDevices > {
  static const char* value()
  {
    return "9f8a3530be8da22ec93eee7ab47ef9a0";
  }

  static const char* value(const ::kortex_driver::ReadAllDevices&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::ReadAllDevices > {
  static const char* value()
  {
    return "kortex_driver/ReadAllDevices";
  }

  static const char* value(const ::kortex_driver::ReadAllDevices&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::ReadAllDevicesRequest> should match
// service_traits::MD5Sum< ::kortex_driver::ReadAllDevices >
template<>
struct MD5Sum< ::kortex_driver::ReadAllDevicesRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::ReadAllDevices >::value();
  }
  static const char* value(const ::kortex_driver::ReadAllDevicesRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::ReadAllDevicesRequest> should match
// service_traits::DataType< ::kortex_driver::ReadAllDevices >
template<>
struct DataType< ::kortex_driver::ReadAllDevicesRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::ReadAllDevices >::value();
  }
  static const char* value(const ::kortex_driver::ReadAllDevicesRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::ReadAllDevicesResponse> should match
// service_traits::MD5Sum< ::kortex_driver::ReadAllDevices >
template<>
struct MD5Sum< ::kortex_driver::ReadAllDevicesResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::ReadAllDevices >::value();
  }
  static const char* value(const ::kortex_driver::ReadAllDevicesResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::ReadAllDevicesResponse> should match
// service_traits::DataType< ::kortex_driver::ReadAllDevices >
template<>
struct DataType< ::kortex_driver::ReadAllDevicesResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::ReadAllDevices >::value();
  }
  static const char* value(const ::kortex_driver::ReadAllDevicesResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_READALLDEVICES_H
