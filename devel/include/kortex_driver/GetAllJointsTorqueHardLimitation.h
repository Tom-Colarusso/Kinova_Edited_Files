// Generated by gencpp from file kortex_driver/GetAllJointsTorqueHardLimitation.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETALLJOINTSTORQUEHARDLIMITATION_H
#define KORTEX_DRIVER_MESSAGE_GETALLJOINTSTORQUEHARDLIMITATION_H

#include <ros/service_traits.h>


#include <kortex_driver/GetAllJointsTorqueHardLimitationRequest.h>
#include <kortex_driver/GetAllJointsTorqueHardLimitationResponse.h>


namespace kortex_driver
{

struct GetAllJointsTorqueHardLimitation
{

typedef GetAllJointsTorqueHardLimitationRequest Request;
typedef GetAllJointsTorqueHardLimitationResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetAllJointsTorqueHardLimitation
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::GetAllJointsTorqueHardLimitation > {
  static const char* value()
  {
    return "80ab5247f79a646a096a6bb5bc451a7d";
  }

  static const char* value(const ::kortex_driver::GetAllJointsTorqueHardLimitation&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::GetAllJointsTorqueHardLimitation > {
  static const char* value()
  {
    return "kortex_driver/GetAllJointsTorqueHardLimitation";
  }

  static const char* value(const ::kortex_driver::GetAllJointsTorqueHardLimitation&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::GetAllJointsTorqueHardLimitationRequest> should match
// service_traits::MD5Sum< ::kortex_driver::GetAllJointsTorqueHardLimitation >
template<>
struct MD5Sum< ::kortex_driver::GetAllJointsTorqueHardLimitationRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetAllJointsTorqueHardLimitation >::value();
  }
  static const char* value(const ::kortex_driver::GetAllJointsTorqueHardLimitationRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetAllJointsTorqueHardLimitationRequest> should match
// service_traits::DataType< ::kortex_driver::GetAllJointsTorqueHardLimitation >
template<>
struct DataType< ::kortex_driver::GetAllJointsTorqueHardLimitationRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetAllJointsTorqueHardLimitation >::value();
  }
  static const char* value(const ::kortex_driver::GetAllJointsTorqueHardLimitationRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::GetAllJointsTorqueHardLimitationResponse> should match
// service_traits::MD5Sum< ::kortex_driver::GetAllJointsTorqueHardLimitation >
template<>
struct MD5Sum< ::kortex_driver::GetAllJointsTorqueHardLimitationResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetAllJointsTorqueHardLimitation >::value();
  }
  static const char* value(const ::kortex_driver::GetAllJointsTorqueHardLimitationResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetAllJointsTorqueHardLimitationResponse> should match
// service_traits::DataType< ::kortex_driver::GetAllJointsTorqueHardLimitation >
template<>
struct DataType< ::kortex_driver::GetAllJointsTorqueHardLimitationResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetAllJointsTorqueHardLimitation >::value();
  }
  static const char* value(const ::kortex_driver::GetAllJointsTorqueHardLimitationResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETALLJOINTSTORQUEHARDLIMITATION_H
