// Generated by gencpp from file kortex_driver/GetExtrinsicParameters.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETEXTRINSICPARAMETERS_H
#define KORTEX_DRIVER_MESSAGE_GETEXTRINSICPARAMETERS_H

#include <ros/service_traits.h>


#include <kortex_driver/GetExtrinsicParametersRequest.h>
#include <kortex_driver/GetExtrinsicParametersResponse.h>


namespace kortex_driver
{

struct GetExtrinsicParameters
{

typedef GetExtrinsicParametersRequest Request;
typedef GetExtrinsicParametersResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetExtrinsicParameters
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::GetExtrinsicParameters > {
  static const char* value()
  {
    return "769b6e0a4e2cbed0ce69ce84dc1b50e3";
  }

  static const char* value(const ::kortex_driver::GetExtrinsicParameters&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::GetExtrinsicParameters > {
  static const char* value()
  {
    return "kortex_driver/GetExtrinsicParameters";
  }

  static const char* value(const ::kortex_driver::GetExtrinsicParameters&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::GetExtrinsicParametersRequest> should match
// service_traits::MD5Sum< ::kortex_driver::GetExtrinsicParameters >
template<>
struct MD5Sum< ::kortex_driver::GetExtrinsicParametersRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetExtrinsicParameters >::value();
  }
  static const char* value(const ::kortex_driver::GetExtrinsicParametersRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetExtrinsicParametersRequest> should match
// service_traits::DataType< ::kortex_driver::GetExtrinsicParameters >
template<>
struct DataType< ::kortex_driver::GetExtrinsicParametersRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetExtrinsicParameters >::value();
  }
  static const char* value(const ::kortex_driver::GetExtrinsicParametersRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::GetExtrinsicParametersResponse> should match
// service_traits::MD5Sum< ::kortex_driver::GetExtrinsicParameters >
template<>
struct MD5Sum< ::kortex_driver::GetExtrinsicParametersResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetExtrinsicParameters >::value();
  }
  static const char* value(const ::kortex_driver::GetExtrinsicParametersResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetExtrinsicParametersResponse> should match
// service_traits::DataType< ::kortex_driver::GetExtrinsicParameters >
template<>
struct DataType< ::kortex_driver::GetExtrinsicParametersResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetExtrinsicParameters >::value();
  }
  static const char* value(const ::kortex_driver::GetExtrinsicParametersResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETEXTRINSICPARAMETERS_H
