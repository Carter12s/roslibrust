#ifndef GEOMETRY_MSGS_MESSAGE_POINT32
#define GEOMETRY_MSGS_MESSAGE_POINT32

#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>



namespace geometry_msgs {

template <class ContainerAllocator>
struct Point32_
{
  typedef Point32_<ContainerAllocator> Type;

  
  
    Point32_()
      : x()
    , y()
    , z() {
    }

    Point32_(const ContainerAllocator& _alloc)
      : x(_alloc)
    , y(_alloc)
    , z(_alloc) {
      (void)_alloc;
    }
  

  
  
    
    
      
        typedef float _x_type;
    _x_type x;
    
    
      
        typedef float _y_type;
    _y_type y;
    
    
      
        typedef float _z_type;
    _z_type z;

  
  

  
  

  typedef boost::shared_ptr< ::geometry_msgs::Point32_<ContainerAllocator>> Ptr;
  typedef boost::shared_ptr< ::geometry_msgs::Point32_<ContainerAllocator> const> ConstPtr;

}; // struct Point32_

typedef ::geometry_msgs::Point32_<std::allocator<void>> Point32;

typedef boost::shared_ptr< ::geometry_msgs::Point32> Point32Ptr;
typedef boost::shared_ptr< ::geometry_msgs::Point32 const> Point32ConstPtr;

// constants requiring out of line definition


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::geometry_msgs::Point32_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::geometry_msgs::Point32_<ContainerAllocator> >::stream(s, "", v);
return s;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::geometry_msgs::Point32_<ContainerAllocator1> & lhs, const ::geometry_msgs::Point32_<ContainerAllocator2> & rhs)
{
  return
    lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z &&
    true;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::geometry_msgs::Point32_<ContainerAllocator1> & lhs, const ::geometry_msgs::Point32_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace geometry_msgs

namespace ros
{
namespace message_traits
{
template <class ContainerAllocator>
struct IsMessage< ::geometry_msgs::Point32_<ContainerAllocator>>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::geometry_msgs::Point32_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::geometry_msgs::Point32_<ContainerAllocator>>
  : TrueType
    { };

template <class ContainerAllocator>
struct IsFixedSize< ::geometry_msgs::Point32_<ContainerAllocator> const>
  : TrueType
    { };

template <class ContainerAllocator>
struct HasHeader< ::geometry_msgs::Point32_<ContainerAllocator>>
  : FalseType
    { };

template <class ContainerAllocator>
struct HasHeader< ::geometry_msgs::Point32_<ContainerAllocator> const>
  : FalseType
    { };

template<class ContainerAllocator>
struct MD5Sum< ::geometry_msgs::Point32_<ContainerAllocator>>
{
  static const char* value()
  {
    return "cc153912f1453b708d221682bc23d9ac";
  }

  static const char* value(const ::geometry_msgs::Point32_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcc153912f1453b70ULL;
  static const uint64_t static_value2 = 0x8d221682bc23d9acULL;
};

template<class ContainerAllocator>
struct DataType< ::geometry_msgs::Point32_<ContainerAllocator>>
{
  static const char* value()
  {
    return "geometry_msgs/Point32";
  }

  static const char* value(const ::geometry_msgs::Point32_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::geometry_msgs::Point32_<ContainerAllocator>>
{
  static const char* value()
  {
    return "";
  }

  static const char* value(const ::geometry_msgs::Point32_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator>
struct Serializer< ::geometry_msgs::Point32_<ContainerAllocator>>
{
  
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.z);
  }
  

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct Point32_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::geometry_msgs::Point32_<ContainerAllocator>>
{
  
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::geometry_msgs::Point32_<ContainerAllocator>& v)
  {
    

    s << indent << "x: ";
    
    Printer< float>::stream(s, indent + "  ", v.x);
    

    
    

    s << indent << "y: ";
    
    Printer< float>::stream(s, indent + "  ", v.y);
    

    
    

    s << indent << "z: ";
    
    Printer< float>::stream(s, indent + "  ", v.z);
    

    
  }
  
};

} // namespace message_operations
} // namespace ros

#endif // GEOMETRY_MSGS_MESSAGE_POINT32