// Generated by gencpp from file kortex_driver/SendJointSpeedsCommand.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_SENDJOINTSPEEDSCOMMAND_H
#define KORTEX_DRIVER_MESSAGE_SENDJOINTSPEEDSCOMMAND_H

#include <ros/service_traits.h>


#include <kortex_driver/SendJointSpeedsCommandRequest.h>
#include <kortex_driver/SendJointSpeedsCommandResponse.h>


namespace kortex_driver
{

struct SendJointSpeedsCommand
{

typedef SendJointSpeedsCommandRequest Request;
typedef SendJointSpeedsCommandResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SendJointSpeedsCommand
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::SendJointSpeedsCommand > {
  static const char* value()
  {
    return "35bff15135e19b4099e6a92d5e7d08d5";
  }

  static const char* value(const ::kortex_driver::SendJointSpeedsCommand&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::SendJointSpeedsCommand > {
  static const char* value()
  {
    return "kortex_driver/SendJointSpeedsCommand";
  }

  static const char* value(const ::kortex_driver::SendJointSpeedsCommand&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::SendJointSpeedsCommandRequest> should match
// service_traits::MD5Sum< ::kortex_driver::SendJointSpeedsCommand >
template<>
struct MD5Sum< ::kortex_driver::SendJointSpeedsCommandRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SendJointSpeedsCommand >::value();
  }
  static const char* value(const ::kortex_driver::SendJointSpeedsCommandRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SendJointSpeedsCommandRequest> should match
// service_traits::DataType< ::kortex_driver::SendJointSpeedsCommand >
template<>
struct DataType< ::kortex_driver::SendJointSpeedsCommandRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SendJointSpeedsCommand >::value();
  }
  static const char* value(const ::kortex_driver::SendJointSpeedsCommandRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::SendJointSpeedsCommandResponse> should match
// service_traits::MD5Sum< ::kortex_driver::SendJointSpeedsCommand >
template<>
struct MD5Sum< ::kortex_driver::SendJointSpeedsCommandResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SendJointSpeedsCommand >::value();
  }
  static const char* value(const ::kortex_driver::SendJointSpeedsCommandResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SendJointSpeedsCommandResponse> should match
// service_traits::DataType< ::kortex_driver::SendJointSpeedsCommand >
template<>
struct DataType< ::kortex_driver::SendJointSpeedsCommandResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SendJointSpeedsCommand >::value();
  }
  static const char* value(const ::kortex_driver::SendJointSpeedsCommandResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_SENDJOINTSPEEDSCOMMAND_H
