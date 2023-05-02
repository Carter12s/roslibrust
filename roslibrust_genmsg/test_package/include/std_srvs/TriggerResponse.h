#ifndef STD_SRVS_MESSAGE_TRIGGERRESPONSE
#define STD_SRVS_MESSAGE_TRIGGERRESPONSE

#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>



namespace std_srvs {

template <class ContainerAllocator>
struct TriggerResponse_
{
  typedef TriggerResponse_<ContainerAllocator> Type;

  
  
    TriggerResponse_()
      : success()
    , message() {
    }

    TriggerResponse_(const ContainerAllocator& _alloc)
      : success(_alloc)
    , message(_alloc) {
      (void)_alloc;
    }
  

  
  
    
    
      
        typedef bool _success_type;
    _success_type success;
    
    
      
        typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _message_type;
    _message_type message;

  
  

  
  

  typedef boost::shared_ptr< ::std_srvs::TriggerResponse_<ContainerAllocator>> Ptr;
  typedef boost::shared_ptr< ::std_srvs::TriggerResponse_<ContainerAllocator> const> ConstPtr;

}; // struct TriggerResponse_

typedef ::std_srvs::TriggerResponse_<std::allocator<void>> TriggerResponse;

typedef boost::shared_ptr< ::std_srvs::TriggerResponse> TriggerResponsePtr;
typedef boost::shared_ptr< ::std_srvs::TriggerResponse const> TriggerResponseConstPtr;

// constants requiring out of line definition


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::std_srvs::TriggerResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::std_srvs::TriggerResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::std_srvs::TriggerResponse_<ContainerAllocator1> & lhs, const ::std_srvs::TriggerResponse_<ContainerAllocator2> & rhs)
{
  return
    lhs.success == rhs.success &&
    lhs.message == rhs.message &&
    true;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::std_srvs::TriggerResponse_<ContainerAllocator1> & lhs, const ::std_srvs::TriggerResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace std_srvs

namespace ros
{
namespace message_traits
{
template <class ContainerAllocator>
struct IsMessage< ::std_srvs::TriggerResponse_<ContainerAllocator>>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::std_srvs::TriggerResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::std_srvs::TriggerResponse_<ContainerAllocator>>
  : FalseType
    { };

template <class ContainerAllocator>
struct IsFixedSize< ::std_srvs::TriggerResponse_<ContainerAllocator> const>
  : FalseType
    { };

template <class ContainerAllocator>
struct HasHeader< ::std_srvs::TriggerResponse_<ContainerAllocator>>
  : FalseType
    { };

template <class ContainerAllocator>
struct HasHeader< ::std_srvs::TriggerResponse_<ContainerAllocator> const>
  : FalseType
    { };

template<class ContainerAllocator>
struct MD5Sum< ::std_srvs::TriggerResponse_<ContainerAllocator>>
{
  static const char* value()
  {
    return "937c9679a518e3a18d831e57125ea522";
  }

  static const char* value(const ::std_srvs::TriggerResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x937c9679a518e3a1ULL;
  static const uint64_t static_value2 = 0x8d831e57125ea522ULL;
};

template<class ContainerAllocator>
struct DataType< ::std_srvs::TriggerResponse_<ContainerAllocator>>
{
  static const char* value()
  {
    return "std_srvs/TriggerResponse";
  }

  static const char* value(const ::std_srvs::TriggerResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::std_srvs::TriggerResponse_<ContainerAllocator>>
{
  static const char* value()
  {
    return "";
  }

  static const char* value(const ::std_srvs::TriggerResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator>
struct Serializer< ::std_srvs::TriggerResponse_<ContainerAllocator>>
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.success);
    stream.next(m.message);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct TriggerResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::std_srvs::TriggerResponse_<ContainerAllocator>>
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::std_srvs::TriggerResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    s << std::endl;
    
    Printer< bool>::stream(s, indent + "  ", v.success);
    
    s << indent << "message: ";
    s << std::endl;
    
    Printer< std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.message);
    
  }
};

} // namespace message_operations
} // namespace ros

#endif // STD_SRVS_MESSAGE_TRIGGERRESPONSE