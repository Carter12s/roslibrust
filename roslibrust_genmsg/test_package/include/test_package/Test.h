#ifndef TEST_PACKAGE_MESSAGE_TEST
#define TEST_PACKAGE_MESSAGE_TEST

#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point32.h>
#include <std_msgs/Header.h>

namespace test_package {

template <class ContainerAllocator>
struct Test_
{
  typedef Test_<ContainerAllocator> Type;
  Test_()
    : points()
    , test_a()
    , header() {
  }

  Test_(const ContainerAllocator& _alloc)
    : points(_alloc)
    , test_a(_alloc)
    , header(_alloc) {
    (void)_alloc;
  }
    
        typedef std::vector<::geometry_msgs::Point32_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<::geometry_msgs::Point32_<ContainerAllocator>>> _points_type;
    _points_type points;
    
        typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _test_a_type;
    _test_a_type test_a;
    
        typedef ::std_msgs::Header_<ContainerAllocator> _header_type;
    _header_type header;

  
    enum {
    };
      static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> RUNNING_STATE;
      static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> STOPPED_STATE;

  typedef boost::shared_ptr< ::test_package::Test_<ContainerAllocator>> Ptr;
  typedef boost::shared_ptr< ::test_package::Test_<ContainerAllocator> const> ConstPtr;

}; // struct Test_

typedef ::test_package::Test_<std::allocator<void>> Test;

typedef boost::shared_ptr< ::test_package::Test> TestPtr;
typedef boost::shared_ptr< ::test_package::Test const> TestConstPtr;

// constants requiring out of line definition
    template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      Test_<ContainerAllocator>::RUNNING_STATE =
          u8"RUNNING";
  
    template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      Test_<ContainerAllocator>::STOPPED_STATE =
          u8"STOPPED";
  

template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::test_package::Test_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::test_package::Test_<ContainerAllocator> >::stream(s, "", v);
return s;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::test_package::Test_<ContainerAllocator1> & lhs, const ::test_package::Test_<ContainerAllocator2> & rhs)
{
  return
    lhs.points == rhs.points &&
    lhs.test_a == rhs.test_a &&
    lhs.header == rhs.header &&
    true;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::test_package::Test_<ContainerAllocator1> & lhs, const ::test_package::Test_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace test_package

namespace ros
{
namespace message_traits
{
template <class ContainerAllocator>
struct IsMessage< ::test_package::Test_<ContainerAllocator>>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::test_package::Test_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::test_package::Test_<ContainerAllocator>>
  : FalseType
    { };

template <class ContainerAllocator>
struct IsFixedSize< ::test_package::Test_<ContainerAllocator> const>
  : FalseType
    { };

template <class ContainerAllocator>
struct HasHeader< ::test_package::Test_<ContainerAllocator>>
  : TrueType
    { };

template <class ContainerAllocator>
struct HasHeader< ::test_package::Test_<ContainerAllocator> const>
  : TrueType
    { };

template<class ContainerAllocator>
struct MD5Sum< ::test_package::Test_<ContainerAllocator>>
{
  static const char* value()
  {
    return "55941ae53e1a50a2d50c1c9ceab9efc0";
  }

  static const char* value(const ::test_package::Test_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x55941ae53e1a50a2ULL;
  static const uint64_t static_value2 = 0xd50c1c9ceab9efc0ULL;
};

template<class ContainerAllocator>
struct DataType< ::test_package::Test_<ContainerAllocator>>
{
  static const char* value()
  {
    return "test_package/Test";
  }

  static const char* value(const ::test_package::Test_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::test_package::Test_<ContainerAllocator>>
{
  static const char* value()
  {
    return "";
  }

  static const char* value(const ::test_package::Test_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator>
struct Serializer< ::test_package::Test_<ContainerAllocator>>
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.points);
    stream.next(m.test_a);
    stream.next(m.header);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct Test_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::test_package::Test_<ContainerAllocator>>
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::test_package::Test_<ContainerAllocator>& v)
  {
    
    s << indent << "points[]" << std::endl;
    for (size_t i = 0; i < v.points.size(); ++i)
    {
      s << indent << "  points[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer<::geometry_msgs::Point32>::stream(s, indent + "    ", v.points[i]);
    }

    s << indent << "test_a: ";
    Printer< std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.test_a);

    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator>>::stream(s, indent + "  ", v.header);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TEST_PACKAGE_MESSAGE_TEST