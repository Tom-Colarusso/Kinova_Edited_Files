// Generated by gencpp from file kortex_driver/PlaySequence.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_PLAYSEQUENCE_H
#define KORTEX_DRIVER_MESSAGE_PLAYSEQUENCE_H

#include <ros/service_traits.h>


#include <kortex_driver/PlaySequenceRequest.h>
#include <kortex_driver/PlaySequenceResponse.h>


namespace kortex_driver
{

struct PlaySequence
{

typedef PlaySequenceRequest Request;
typedef PlaySequenceResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct PlaySequence
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::PlaySequence > {
  static const char* value()
  {
    return "04dfaeca45772f660e0913aa84774e13";
  }

  static const char* value(const ::kortex_driver::PlaySequence&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::PlaySequence > {
  static const char* value()
  {
    return "kortex_driver/PlaySequence";
  }

  static const char* value(const ::kortex_driver::PlaySequence&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::PlaySequenceRequest> should match
// service_traits::MD5Sum< ::kortex_driver::PlaySequence >
template<>
struct MD5Sum< ::kortex_driver::PlaySequenceRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::PlaySequence >::value();
  }
  static const char* value(const ::kortex_driver::PlaySequenceRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::PlaySequenceRequest> should match
// service_traits::DataType< ::kortex_driver::PlaySequence >
template<>
struct DataType< ::kortex_driver::PlaySequenceRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::PlaySequence >::value();
  }
  static const char* value(const ::kortex_driver::PlaySequenceRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::PlaySequenceResponse> should match
// service_traits::MD5Sum< ::kortex_driver::PlaySequence >
template<>
struct MD5Sum< ::kortex_driver::PlaySequenceResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::PlaySequence >::value();
  }
  static const char* value(const ::kortex_driver::PlaySequenceResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::PlaySequenceResponse> should match
// service_traits::DataType< ::kortex_driver::PlaySequence >
template<>
struct DataType< ::kortex_driver::PlaySequenceResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::PlaySequence >::value();
  }
  static const char* value(const ::kortex_driver::PlaySequenceResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_PLAYSEQUENCE_H
