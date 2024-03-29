#ifndef STD_MSGS_MESSAGE_CHAR
#define STD_MSGS_MESSAGE_CHAR

#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace std_msgs {

template <class ContainerAllocator>
struct Char_
{
  typedef Char_<ContainerAllocator> Type;
  Char_()
    : data() {
  }

  Char_(const ContainerAllocator& _alloc)
    : data(_alloc) {
    (void)_alloc;
  }
    
        typedef uint8_t _data_type;
    _data_type data;

  

  typedef boost::shared_ptr< ::std_msgs::Char_<ContainerAllocator>> Ptr;
  typedef boost::shared_ptr< ::std_msgs::Char_<ContainerAllocator> const> ConstPtr;

}; // struct Char_

typedef ::std_msgs::Char_<std::allocator<void>> Char;

typedef boost::shared_ptr< ::std_msgs::Char> CharPtr;
typedef boost::shared_ptr< ::std_msgs::Char const> CharConstPtr;

// constants requiring out of line definition

template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::std_msgs::Char_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::std_msgs::Char_<ContainerAllocator> >::stream(s, "", v);
return s;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::std_msgs::Char_<ContainerAllocator1> & lhs, const ::std_msgs::Char_<ContainerAllocator2> & rhs)
{
  return
    lhs.data == rhs.data &&
    true;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::std_msgs::Char_<ContainerAllocator1> & lhs, const ::std_msgs::Char_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace std_msgs

namespace ros
{
namespace message_traits
{
template <class ContainerAllocator>
struct IsMessage< ::std_msgs::Char_<ContainerAllocator>>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::std_msgs::Char_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::std_msgs::Char_<ContainerAllocator>>
  : TrueType
    { };

template <class ContainerAllocator>
struct IsFixedSize< ::std_msgs::Char_<ContainerAllocator> const>
  : TrueType
    { };

template <class ContainerAllocator>
struct HasHeader< ::std_msgs::Char_<ContainerAllocator>>
  : FalseType
    { };

template <class ContainerAllocator>
struct HasHeader< ::std_msgs::Char_<ContainerAllocator> const>
  : FalseType
    { };

template<class ContainerAllocator>
struct MD5Sum< ::std_msgs::Char_<ContainerAllocator>>
{
  static constexpr char const * value()
  {
    return "1bf77f25acecdedba0e224b162199717";
  }

  static const char* value(const ::std_msgs::Char_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1bf77f25acecdedbULL;
  static const uint64_t static_value2 = 0xa0e224b162199717ULL;
};

template<class ContainerAllocator>
struct DataType< ::std_msgs::Char_<ContainerAllocator>>
{
  static constexpr char const* value()
  {
    return "std_msgs/Char";
  }

  static const char* value(const ::std_msgs::Char_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::std_msgs::Char_<ContainerAllocator>>
{
  static constexpr char const* value()
  {
    return "char data";
  }

  static const char* value(const ::std_msgs::Char_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator>
struct Serializer< ::std_msgs::Char_<ContainerAllocator>>
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.data);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct Char_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::std_msgs::Char_<ContainerAllocator>>
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::std_msgs::Char_<ContainerAllocator>& v)
  {

    s << indent << "data: ";
    Printer< uint8_t>::stream(s, indent + "  ", v.data);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STD_MSGS_MESSAGE_CHAR