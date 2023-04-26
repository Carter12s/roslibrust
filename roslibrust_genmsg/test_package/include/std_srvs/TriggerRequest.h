#ifndef STD_SRVS_MESSAGE_TRIGGERREQUEST
#define STD_SRVS_MESSAGE_TRIGGERREQUEST

#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>



namespace std_srvs {

template <class ContainerAllocator>
struct TriggerRequest_
{
  typedef TriggerRequest_<ContainerAllocator> Type;

  
  
    TriggerRequest_()
    {
    }

    TriggerRequest_(const ContainerAllocator& _alloc)
    {
      (void)_alloc;
    }
  

  
  

  
  

  
  

  typedef boost::shared_ptr< ::std_srvs::TriggerRequest_<ContainerAllocator>> Ptr;
  typedef boost::shared_ptr< ::std_srvs::TriggerRequest_<ContainerAllocator> const> ConstPtr;

}; // struct TriggerRequest_

typedef ::std_srvs::TriggerRequest_<std::allocator<void>> TriggerRequest;

typedef boost::shared_ptr< ::std_srvs::TriggerRequest> TriggerRequestPtr;
typedef boost::shared_ptr< ::std_srvs::TriggerRequest const> TriggerRequestConstPtr;

// constants requiring out of line definition


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::std_srvs::TriggerRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::std_srvs::TriggerRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::std_srvs::TriggerRequest_<ContainerAllocator1> & lhs, const ::std_srvs::TriggerRequest_<ContainerAllocator2> & rhs)
{
  return
    true;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::std_srvs::TriggerRequest_<ContainerAllocator1> & lhs, const ::std_srvs::TriggerRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace std_srvs

namespace ros
{
namespace message_traits
{
template <class ContainerAllocator>
struct IsMessage< ::std_srvs::TriggerRequest_<ContainerAllocator>>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::std_srvs::TriggerRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::std_srvs::TriggerRequest_<ContainerAllocator>>
  : TrueType
    { };

template <class ContainerAllocator>
struct IsFixedSize< ::std_srvs::TriggerRequest_<ContainerAllocator> const>
  : TrueType
    { };

template <class ContainerAllocator>
struct HasHeader< ::std_srvs::TriggerRequest_<ContainerAllocator>>
  : FalseType
    { };

template <class ContainerAllocator>
struct HasHeader< ::std_srvs::TriggerRequest_<ContainerAllocator> const>
  : FalseType
    { };

template<class ContainerAllocator>
struct MD5Sum< ::std_srvs::TriggerRequest_<ContainerAllocator>>
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::std_srvs::TriggerRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::std_srvs::TriggerRequest_<ContainerAllocator>>
{
  static const char* value()
  {
    return "std_srvs/TriggerRequest";
  }

  static const char* value(const ::std_srvs::TriggerRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::std_srvs::TriggerRequest_<ContainerAllocator>>
{
  static const char* value()
  {
    return "";
  }

  static const char* value(const ::std_srvs::TriggerRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator>
struct Serializer< ::std_srvs::TriggerRequest_<ContainerAllocator>>
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct TriggerRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::std_srvs::TriggerRequest_<ContainerAllocator>>
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::std_srvs::TriggerRequest_<ContainerAllocator>& v)
  {
  }
};

} // namespace message_operations
} // namespace ros

#endif // STD_SRVS_MESSAGE_TRIGGERREQUEST