// Generated by gencpp from file kortex_driver/GetIPv4Information.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETIPV4INFORMATION_H
#define KORTEX_DRIVER_MESSAGE_GETIPV4INFORMATION_H

#include <ros/service_traits.h>


#include <kortex_driver/GetIPv4InformationRequest.h>
#include <kortex_driver/GetIPv4InformationResponse.h>


namespace kortex_driver
{

struct GetIPv4Information
{

typedef GetIPv4InformationRequest Request;
typedef GetIPv4InformationResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetIPv4Information
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::GetIPv4Information > {
  static const char* value()
  {
    return "cf6aba6a62d331d4b104d99fe292f36c";
  }

  static const char* value(const ::kortex_driver::GetIPv4Information&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::GetIPv4Information > {
  static const char* value()
  {
    return "kortex_driver/GetIPv4Information";
  }

  static const char* value(const ::kortex_driver::GetIPv4Information&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::GetIPv4InformationRequest> should match
// service_traits::MD5Sum< ::kortex_driver::GetIPv4Information >
template<>
struct MD5Sum< ::kortex_driver::GetIPv4InformationRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetIPv4Information >::value();
  }
  static const char* value(const ::kortex_driver::GetIPv4InformationRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetIPv4InformationRequest> should match
// service_traits::DataType< ::kortex_driver::GetIPv4Information >
template<>
struct DataType< ::kortex_driver::GetIPv4InformationRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetIPv4Information >::value();
  }
  static const char* value(const ::kortex_driver::GetIPv4InformationRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::GetIPv4InformationResponse> should match
// service_traits::MD5Sum< ::kortex_driver::GetIPv4Information >
template<>
struct MD5Sum< ::kortex_driver::GetIPv4InformationResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetIPv4Information >::value();
  }
  static const char* value(const ::kortex_driver::GetIPv4InformationResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetIPv4InformationResponse> should match
// service_traits::DataType< ::kortex_driver::GetIPv4Information >
template<>
struct DataType< ::kortex_driver::GetIPv4InformationResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetIPv4Information >::value();
  }
  static const char* value(const ::kortex_driver::GetIPv4InformationResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETIPV4INFORMATION_H
