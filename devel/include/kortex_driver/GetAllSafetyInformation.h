// Generated by gencpp from file kortex_driver/GetAllSafetyInformation.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETALLSAFETYINFORMATION_H
#define KORTEX_DRIVER_MESSAGE_GETALLSAFETYINFORMATION_H

#include <ros/service_traits.h>


#include <kortex_driver/GetAllSafetyInformationRequest.h>
#include <kortex_driver/GetAllSafetyInformationResponse.h>


namespace kortex_driver
{

struct GetAllSafetyInformation
{

typedef GetAllSafetyInformationRequest Request;
typedef GetAllSafetyInformationResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetAllSafetyInformation
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::GetAllSafetyInformation > {
  static const char* value()
  {
    return "6cd621443d851423fb32151d65f5f576";
  }

  static const char* value(const ::kortex_driver::GetAllSafetyInformation&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::GetAllSafetyInformation > {
  static const char* value()
  {
    return "kortex_driver/GetAllSafetyInformation";
  }

  static const char* value(const ::kortex_driver::GetAllSafetyInformation&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::GetAllSafetyInformationRequest> should match
// service_traits::MD5Sum< ::kortex_driver::GetAllSafetyInformation >
template<>
struct MD5Sum< ::kortex_driver::GetAllSafetyInformationRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetAllSafetyInformation >::value();
  }
  static const char* value(const ::kortex_driver::GetAllSafetyInformationRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetAllSafetyInformationRequest> should match
// service_traits::DataType< ::kortex_driver::GetAllSafetyInformation >
template<>
struct DataType< ::kortex_driver::GetAllSafetyInformationRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetAllSafetyInformation >::value();
  }
  static const char* value(const ::kortex_driver::GetAllSafetyInformationRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::GetAllSafetyInformationResponse> should match
// service_traits::MD5Sum< ::kortex_driver::GetAllSafetyInformation >
template<>
struct MD5Sum< ::kortex_driver::GetAllSafetyInformationResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetAllSafetyInformation >::value();
  }
  static const char* value(const ::kortex_driver::GetAllSafetyInformationResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetAllSafetyInformationResponse> should match
// service_traits::DataType< ::kortex_driver::GetAllSafetyInformation >
template<>
struct DataType< ::kortex_driver::GetAllSafetyInformationResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetAllSafetyInformation >::value();
  }
  static const char* value(const ::kortex_driver::GetAllSafetyInformationResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETALLSAFETYINFORMATION_H
