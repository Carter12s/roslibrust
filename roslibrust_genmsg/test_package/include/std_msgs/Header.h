#ifndef STD_MSGS_MESSAGE_HEADER
#define STD_MSGS_MESSAGE_HEADER

#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace std_msgs {

template <class ContainerAllocator>
struct Header_
{
  typedef Header_<ContainerAllocator> Type;
  Header_()
    : seq()
    , stamp()
    , frame_id() {
  }

  Header_(const ContainerAllocator& _alloc)
    : seq(_alloc)
    , stamp(_alloc)
    , frame_id(_alloc) {
    (void)_alloc;
  }
    
        typedef uint32_t _seq_type;
    _seq_type seq;
    
        typedef ::ros::Time _stamp_type;
    _stamp_type stamp;
    
        typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _frame_id_type;
    _frame_id_type frame_id;

  

  typedef boost::shared_ptr< ::std_msgs::Header_<ContainerAllocator>> Ptr;
  typedef boost::shared_ptr< ::std_msgs::Header_<ContainerAllocator> const> ConstPtr;

}; // struct Header_

typedef ::std_msgs::Header_<std::allocator<void>> Header;

typedef boost::shared_ptr< ::std_msgs::Header> HeaderPtr;
typedef boost::shared_ptr< ::std_msgs::Header const> HeaderConstPtr;

// constants requiring out of line definition

template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::std_msgs::Header_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, "", v);
return s;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::std_msgs::Header_<ContainerAllocator1> & lhs, const ::std_msgs::Header_<ContainerAllocator2> & rhs)
{
  return
    lhs.seq == rhs.seq &&
    lhs.stamp == rhs.stamp &&
    lhs.frame_id == rhs.frame_id &&
    true;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::std_msgs::Header_<ContainerAllocator1> & lhs, const ::std_msgs::Header_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace std_msgs

namespace ros
{
namespace message_traits
{
template <class ContainerAllocator>
struct IsMessage< ::std_msgs::Header_<ContainerAllocator>>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::std_msgs::Header_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::std_msgs::Header_<ContainerAllocator>>
  : FalseType
    { };

template <class ContainerAllocator>
struct IsFixedSize< ::std_msgs::Header_<ContainerAllocator> const>
  : FalseType
    { };

template <class ContainerAllocator>
struct HasHeader< ::std_msgs::Header_<ContainerAllocator>>
  : FalseType
    { };

template <class ContainerAllocator>
struct HasHeader< ::std_msgs::Header_<ContainerAllocator> const>
  : FalseType
    { };

template<class ContainerAllocator>
struct MD5Sum< ::std_msgs::Header_<ContainerAllocator>>
{
  static constexpr char const * value()
  {
    return "2176decaecbce78abc3b96ef049fabed";
  }

  static const char* value(const ::std_msgs::Header_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2176decaecbce78aULL;
  static const uint64_t static_value2 = 0xbc3b96ef049fabedULL;
};

template<class ContainerAllocator>
struct DataType< ::std_msgs::Header_<ContainerAllocator>>
{
  static constexpr char const* value()
  {
    return "std_msgs/Header";
  }

  static const char* value(const ::std_msgs::Header_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::std_msgs::Header_<ContainerAllocator>>
{
  static constexpr char const* value()
  {
    return "# Standard metadata for higher-level stamped data types."
"# This is generally used to communicate timestamped data "
"# in a particular coordinate frame."
"# "
"# sequence ID: consecutively increasing ID "
"uint32 seq"
"#Two-integer timestamp that is expressed as:"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')"
"# time-handling sugar is provided by the client library"
"time stamp"
"#Frame this data is associated with"
"string frame_id";
  }

  static const char* value(const ::std_msgs::Header_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator>
struct Serializer< ::std_msgs::Header_<ContainerAllocator>>
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.seq);
    stream.next(m.stamp);
    stream.next(m.frame_id);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct Header_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::std_msgs::Header_<ContainerAllocator>>
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::std_msgs::Header_<ContainerAllocator>& v)
  {

    s << indent << "seq: ";
    Printer< uint32_t>::stream(s, indent + "  ", v.seq);

    s << indent << "stamp: ";
    Printer< ::ros::Time>::stream(s, indent + "  ", v.stamp);

    s << indent << "frame_id: ";
    Printer< std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.frame_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STD_MSGS_MESSAGE_HEADER