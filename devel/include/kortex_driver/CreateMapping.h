// Generated by gencpp from file kortex_driver/CreateMapping.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_CREATEMAPPING_H
#define KORTEX_DRIVER_MESSAGE_CREATEMAPPING_H

#include <ros/service_traits.h>


#include <kortex_driver/CreateMappingRequest.h>
#include <kortex_driver/CreateMappingResponse.h>


namespace kortex_driver
{

struct CreateMapping
{

typedef CreateMappingRequest Request;
typedef CreateMappingResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct CreateMapping
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::CreateMapping > {
  static const char* value()
  {
    return "1672eeb2532b125a8c2350096e2628b8";
  }

  static const char* value(const ::kortex_driver::CreateMapping&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::CreateMapping > {
  static const char* value()
  {
    return "kortex_driver/CreateMapping";
  }

  static const char* value(const ::kortex_driver::CreateMapping&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::CreateMappingRequest> should match
// service_traits::MD5Sum< ::kortex_driver::CreateMapping >
template<>
struct MD5Sum< ::kortex_driver::CreateMappingRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::CreateMapping >::value();
  }
  static const char* value(const ::kortex_driver::CreateMappingRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::CreateMappingRequest> should match
// service_traits::DataType< ::kortex_driver::CreateMapping >
template<>
struct DataType< ::kortex_driver::CreateMappingRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::CreateMapping >::value();
  }
  static const char* value(const ::kortex_driver::CreateMappingRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::CreateMappingResponse> should match
// service_traits::MD5Sum< ::kortex_driver::CreateMapping >
template<>
struct MD5Sum< ::kortex_driver::CreateMappingResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::CreateMapping >::value();
  }
  static const char* value(const ::kortex_driver::CreateMappingResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::CreateMappingResponse> should match
// service_traits::DataType< ::kortex_driver::CreateMapping >
template<>
struct DataType< ::kortex_driver::CreateMappingResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::CreateMapping >::value();
  }
  static const char* value(const ::kortex_driver::CreateMappingResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_CREATEMAPPING_H
