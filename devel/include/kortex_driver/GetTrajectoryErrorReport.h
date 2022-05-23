// Generated by gencpp from file kortex_driver/GetTrajectoryErrorReport.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETTRAJECTORYERRORREPORT_H
#define KORTEX_DRIVER_MESSAGE_GETTRAJECTORYERRORREPORT_H

#include <ros/service_traits.h>


#include <kortex_driver/GetTrajectoryErrorReportRequest.h>
#include <kortex_driver/GetTrajectoryErrorReportResponse.h>


namespace kortex_driver
{

struct GetTrajectoryErrorReport
{

typedef GetTrajectoryErrorReportRequest Request;
typedef GetTrajectoryErrorReportResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetTrajectoryErrorReport
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::GetTrajectoryErrorReport > {
  static const char* value()
  {
    return "256dc258079fa086775c57ef6b093f4a";
  }

  static const char* value(const ::kortex_driver::GetTrajectoryErrorReport&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::GetTrajectoryErrorReport > {
  static const char* value()
  {
    return "kortex_driver/GetTrajectoryErrorReport";
  }

  static const char* value(const ::kortex_driver::GetTrajectoryErrorReport&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::GetTrajectoryErrorReportRequest> should match
// service_traits::MD5Sum< ::kortex_driver::GetTrajectoryErrorReport >
template<>
struct MD5Sum< ::kortex_driver::GetTrajectoryErrorReportRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetTrajectoryErrorReport >::value();
  }
  static const char* value(const ::kortex_driver::GetTrajectoryErrorReportRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetTrajectoryErrorReportRequest> should match
// service_traits::DataType< ::kortex_driver::GetTrajectoryErrorReport >
template<>
struct DataType< ::kortex_driver::GetTrajectoryErrorReportRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetTrajectoryErrorReport >::value();
  }
  static const char* value(const ::kortex_driver::GetTrajectoryErrorReportRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::GetTrajectoryErrorReportResponse> should match
// service_traits::MD5Sum< ::kortex_driver::GetTrajectoryErrorReport >
template<>
struct MD5Sum< ::kortex_driver::GetTrajectoryErrorReportResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetTrajectoryErrorReport >::value();
  }
  static const char* value(const ::kortex_driver::GetTrajectoryErrorReportResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetTrajectoryErrorReportResponse> should match
// service_traits::DataType< ::kortex_driver::GetTrajectoryErrorReport >
template<>
struct DataType< ::kortex_driver::GetTrajectoryErrorReportResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetTrajectoryErrorReport >::value();
  }
  static const char* value(const ::kortex_driver::GetTrajectoryErrorReportResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETTRAJECTORYERRORREPORT_H
