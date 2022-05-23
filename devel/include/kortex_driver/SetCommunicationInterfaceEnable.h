// Generated by gencpp from file kortex_driver/SetCommunicationInterfaceEnable.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_SETCOMMUNICATIONINTERFACEENABLE_H
#define KORTEX_DRIVER_MESSAGE_SETCOMMUNICATIONINTERFACEENABLE_H

#include <ros/service_traits.h>


#include <kortex_driver/SetCommunicationInterfaceEnableRequest.h>
#include <kortex_driver/SetCommunicationInterfaceEnableResponse.h>


namespace kortex_driver
{

struct SetCommunicationInterfaceEnable
{

typedef SetCommunicationInterfaceEnableRequest Request;
typedef SetCommunicationInterfaceEnableResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetCommunicationInterfaceEnable
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::SetCommunicationInterfaceEnable > {
  static const char* value()
  {
    return "ea61eebb1ea7afcd99f7fa2e0cb8d9db";
  }

  static const char* value(const ::kortex_driver::SetCommunicationInterfaceEnable&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::SetCommunicationInterfaceEnable > {
  static const char* value()
  {
    return "kortex_driver/SetCommunicationInterfaceEnable";
  }

  static const char* value(const ::kortex_driver::SetCommunicationInterfaceEnable&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::SetCommunicationInterfaceEnableRequest> should match
// service_traits::MD5Sum< ::kortex_driver::SetCommunicationInterfaceEnable >
template<>
struct MD5Sum< ::kortex_driver::SetCommunicationInterfaceEnableRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SetCommunicationInterfaceEnable >::value();
  }
  static const char* value(const ::kortex_driver::SetCommunicationInterfaceEnableRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SetCommunicationInterfaceEnableRequest> should match
// service_traits::DataType< ::kortex_driver::SetCommunicationInterfaceEnable >
template<>
struct DataType< ::kortex_driver::SetCommunicationInterfaceEnableRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SetCommunicationInterfaceEnable >::value();
  }
  static const char* value(const ::kortex_driver::SetCommunicationInterfaceEnableRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::SetCommunicationInterfaceEnableResponse> should match
// service_traits::MD5Sum< ::kortex_driver::SetCommunicationInterfaceEnable >
template<>
struct MD5Sum< ::kortex_driver::SetCommunicationInterfaceEnableResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SetCommunicationInterfaceEnable >::value();
  }
  static const char* value(const ::kortex_driver::SetCommunicationInterfaceEnableResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SetCommunicationInterfaceEnableResponse> should match
// service_traits::DataType< ::kortex_driver::SetCommunicationInterfaceEnable >
template<>
struct DataType< ::kortex_driver::SetCommunicationInterfaceEnableResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SetCommunicationInterfaceEnable >::value();
  }
  static const char* value(const ::kortex_driver::SetCommunicationInterfaceEnableResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_SETCOMMUNICATIONINTERFACEENABLE_H
