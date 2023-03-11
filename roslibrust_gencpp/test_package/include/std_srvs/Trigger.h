#ifndef STD_SRVS_MESSAGE_TRIGGER
#define STD_SRVS_MESSAGE_TRIGGER

#include <ros/service_traits.h>

#include <std_srvs/TriggerRequest.h>
#include <std_srvs/TriggerResponse.h>

namespace std_srvs {

struct Trigger
{

  typedef TriggerRequest Request;
  typedef TriggerResponse Response;

  Request request;
  Response response;

  typedef Request RequestType;
  typedef Response ResponseType;

}; // struct Trigger

} // namespace std_srvs

namespace ros
{
namespace service_traits
{

template<>
struct MD5Sum< ::std_srvs::Trigger > {
  static const char* value()
  {
    return "937c9679a518e3a18d831e57125ea522";
  }

  static const char* value(const ::std_srvs::Trigger &) { return value(); }
};

template<>
struct DataType< ::std_srvs::Trigger > {
  static const char* value()
  {
    return "std_srvs/Trigger";
  }

  static const char* value(const ::std_srvs::Trigger &) { return value(); }
};

template<>
struct MD5Sum< ::std_srvs::TriggerRequest > {
  static const char* value()
  {
    return MD5Sum< ::std_srvs::Trigger>::value();
  }
  static const char* value(const ::std_srvs::TriggerRequest &)
  { 
    return value();
  }
};

template<>
struct DataType< ::std_srvs::TriggerRequest > {
  static const char* value()
  {
    return DataType< ::std_srvs::Trigger>::value();
  }

  static const char* value(const ::std_srvs::TriggerRequest &)
  { 
    return value();
  }
};

template<>
struct MD5Sum< ::std_srvs::TriggerResponse > {
  static const char* value()
  {
    return MD5Sum< ::std_srvs::Trigger>::value();
  }
  static const char* value(const ::std_srvs::TriggerResponse &)
  { 
    return value();
  }
};

template<>
struct DataType< ::std_srvs::TriggerResponse > {
  static const char* value()
  {
    return DataType< ::std_srvs::Trigger>::value();
  }

  static const char* value(const ::std_srvs::TriggerResponse &)
  { 
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // STD_SRVS_MESSAGE_TRIGGER_H