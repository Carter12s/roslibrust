#ifndef SENSOR_MSGS_MESSAGE_BATTERYSTATE
#define SENSOR_MSGS_MESSAGE_BATTERYSTATE

#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


#include <std_msgs/Header.h>

namespace sensor_msgs {

template <class ContainerAllocator>
struct BatteryState_
{
  typedef BatteryState_<ContainerAllocator> Type;

  
  
    BatteryState_()
      : header()
    , voltage()
    , temperature()
    , current()
    , charge()
    , capacity()
    , design_capacity()
    , percentage()
    , power_supply_status()
    , power_supply_health()
    , power_supply_technology()
    , present()
    , cell_voltage()
    , cell_temperature()
    , location()
    , serial_number() {
    }

    BatteryState_(const ContainerAllocator& _alloc)
      : header(_alloc)
    , voltage(_alloc)
    , temperature(_alloc)
    , current(_alloc)
    , charge(_alloc)
    , capacity(_alloc)
    , design_capacity(_alloc)
    , percentage(_alloc)
    , power_supply_status(_alloc)
    , power_supply_health(_alloc)
    , power_supply_technology(_alloc)
    , present(_alloc)
    , cell_voltage(_alloc)
    , cell_temperature(_alloc)
    , location(_alloc)
    , serial_number(_alloc) {
      (void)_alloc;
    }
  

  
  
    
    
      
        typedef ::std_msgs::Header_<ContainerAllocator> _header_type;
    _header_type header;
    
    
      
        typedef float _voltage_type;
    _voltage_type voltage;
    
    
      
        typedef float _temperature_type;
    _temperature_type temperature;
    
    
      
        typedef float _current_type;
    _current_type current;
    
    
      
        typedef float _charge_type;
    _charge_type charge;
    
    
      
        typedef float _capacity_type;
    _capacity_type capacity;
    
    
      
        typedef float _design_capacity_type;
    _design_capacity_type design_capacity;
    
    
      
        typedef float _percentage_type;
    _percentage_type percentage;
    
    
      
        typedef uint8_t _power_supply_status_type;
    _power_supply_status_type power_supply_status;
    
    
      
        typedef uint8_t _power_supply_health_type;
    _power_supply_health_type power_supply_health;
    
    
      
        typedef uint8_t _power_supply_technology_type;
    _power_supply_technology_type power_supply_technology;
    
    
      
        typedef bool _present_type;
    _present_type present;
    
    
      
        typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _cell_voltage_type;
    _cell_voltage_type cell_voltage;
    
    
      
        typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _cell_temperature_type;
    _cell_temperature_type cell_temperature;
    
    
      
        typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _location_type;
    _location_type location;
    
    
      
        typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _serial_number_type;
    _serial_number_type serial_number;

  
  
    enum {
    
        POWER_SUPPLY_STATUS_UNKNOWN = 0u,
        POWER_SUPPLY_STATUS_CHARGING = 1u,
        POWER_SUPPLY_STATUS_DISCHARGING = 2u,
        POWER_SUPPLY_STATUS_NOT_CHARGING = 3u,
        POWER_SUPPLY_STATUS_FULL = 4u,
        POWER_SUPPLY_HEALTH_UNKNOWN = 0u,
        POWER_SUPPLY_HEALTH_GOOD = 1u,
        POWER_SUPPLY_HEALTH_OVERHEAT = 2u,
        POWER_SUPPLY_HEALTH_DEAD = 3u,
        POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4u,
        POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5u,
        POWER_SUPPLY_HEALTH_COLD = 6u,
        POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7u,
        POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8u,
        POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0u,
        POWER_SUPPLY_TECHNOLOGY_NIMH = 1u,
        POWER_SUPPLY_TECHNOLOGY_LION = 2u,
        POWER_SUPPLY_TECHNOLOGY_LIPO = 3u,
        POWER_SUPPLY_TECHNOLOGY_LIFE = 4u,
        POWER_SUPPLY_TECHNOLOGY_NICD = 5u,
        POWER_SUPPLY_TECHNOLOGY_LIMN = 6u,
    };
  

  
  

  typedef boost::shared_ptr< ::sensor_msgs::BatteryState_<ContainerAllocator>> Ptr;
  typedef boost::shared_ptr< ::sensor_msgs::BatteryState_<ContainerAllocator> const> ConstPtr;

}; // struct BatteryState_

typedef ::sensor_msgs::BatteryState_<std::allocator<void>> BatteryState;

typedef boost::shared_ptr< ::sensor_msgs::BatteryState> BatteryStatePtr;
typedef boost::shared_ptr< ::sensor_msgs::BatteryState const> BatteryStateConstPtr;

// constants requiring out of line definition


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sensor_msgs::BatteryState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sensor_msgs::BatteryState_<ContainerAllocator> >::stream(s, "", v);
return s;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::sensor_msgs::BatteryState_<ContainerAllocator1> & lhs, const ::sensor_msgs::BatteryState_<ContainerAllocator2> & rhs)
{
  return
    lhs.header == rhs.header &&
    lhs.voltage == rhs.voltage &&
    lhs.temperature == rhs.temperature &&
    lhs.current == rhs.current &&
    lhs.charge == rhs.charge &&
    lhs.capacity == rhs.capacity &&
    lhs.design_capacity == rhs.design_capacity &&
    lhs.percentage == rhs.percentage &&
    lhs.power_supply_status == rhs.power_supply_status &&
    lhs.power_supply_health == rhs.power_supply_health &&
    lhs.power_supply_technology == rhs.power_supply_technology &&
    lhs.present == rhs.present &&
    lhs.cell_voltage == rhs.cell_voltage &&
    lhs.cell_temperature == rhs.cell_temperature &&
    lhs.location == rhs.location &&
    lhs.serial_number == rhs.serial_number &&
    true;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::sensor_msgs::BatteryState_<ContainerAllocator1> & lhs, const ::sensor_msgs::BatteryState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace sensor_msgs

namespace ros
{
namespace message_traits
{
template <class ContainerAllocator>
struct IsMessage< ::sensor_msgs::BatteryState_<ContainerAllocator>>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sensor_msgs::BatteryState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sensor_msgs::BatteryState_<ContainerAllocator>>
  : FalseType
    { };

template <class ContainerAllocator>
struct IsFixedSize< ::sensor_msgs::BatteryState_<ContainerAllocator> const>
  : FalseType
    { };

template <class ContainerAllocator>
struct HasHeader< ::sensor_msgs::BatteryState_<ContainerAllocator>>
  : TrueType
    { };

template <class ContainerAllocator>
struct HasHeader< ::sensor_msgs::BatteryState_<ContainerAllocator> const>
  : TrueType
    { };

template<class ContainerAllocator>
struct MD5Sum< ::sensor_msgs::BatteryState_<ContainerAllocator>>
{
  static const char* value()
  {
    return "4ddae7f048e32fda22cac764685e3974";
  }

  static const char* value(const ::sensor_msgs::BatteryState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4ddae7f048e32fdaULL;
  static const uint64_t static_value2 = 0x22cac764685e3974ULL;
};

template<class ContainerAllocator>
struct DataType< ::sensor_msgs::BatteryState_<ContainerAllocator>>
{
  static const char* value()
  {
    return "sensor_msgs/BatteryState";
  }

  static const char* value(const ::sensor_msgs::BatteryState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sensor_msgs::BatteryState_<ContainerAllocator>>
{
  static const char* value()
  {
    return "";
  }

  static const char* value(const ::sensor_msgs::BatteryState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator>
struct Serializer< ::sensor_msgs::BatteryState_<ContainerAllocator>>
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.voltage);
    stream.next(m.temperature);
    stream.next(m.current);
    stream.next(m.charge);
    stream.next(m.capacity);
    stream.next(m.design_capacity);
    stream.next(m.percentage);
    stream.next(m.power_supply_status);
    stream.next(m.power_supply_health);
    stream.next(m.power_supply_technology);
    stream.next(m.present);
    stream.next(m.cell_voltage);
    stream.next(m.cell_temperature);
    stream.next(m.location);
    stream.next(m.serial_number);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct BatteryState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sensor_msgs::BatteryState_<ContainerAllocator>>
{
  
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sensor_msgs::BatteryState_<ContainerAllocator>& v)
  {
    

    s << indent << "header: ";
    
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator>>::stream(s, indent + "  ", v.header);
    

    
    

    s << indent << "voltage: ";
    
    Printer< float>::stream(s, indent + "  ", v.voltage);
    

    
    

    s << indent << "temperature: ";
    
    Printer< float>::stream(s, indent + "  ", v.temperature);
    

    
    

    s << indent << "current: ";
    
    Printer< float>::stream(s, indent + "  ", v.current);
    

    
    

    s << indent << "charge: ";
    
    Printer< float>::stream(s, indent + "  ", v.charge);
    

    
    

    s << indent << "capacity: ";
    
    Printer< float>::stream(s, indent + "  ", v.capacity);
    

    
    

    s << indent << "design_capacity: ";
    
    Printer< float>::stream(s, indent + "  ", v.design_capacity);
    

    
    

    s << indent << "percentage: ";
    
    Printer< float>::stream(s, indent + "  ", v.percentage);
    

    
    

    s << indent << "power_supply_status: ";
    
    Printer< uint8_t>::stream(s, indent + "  ", v.power_supply_status);
    

    
    

    s << indent << "power_supply_health: ";
    
    Printer< uint8_t>::stream(s, indent + "  ", v.power_supply_health);
    

    
    

    s << indent << "power_supply_technology: ";
    
    Printer< uint8_t>::stream(s, indent + "  ", v.power_supply_technology);
    

    
    

    s << indent << "present: ";
    
    Printer< bool>::stream(s, indent + "  ", v.present);
    

    
    
    
    s << indent << "cell_voltage[]" << std::endl;
    for (size_t i = 0; i < v.cell_voltage.size(); ++i)
    {
      s << indent << "  cell_voltage[" << i << "]: ";
      
      Printer<float>::stream(s, indent + "  ", v.cell_voltage[i]);
      
    }
    
    
    
    s << indent << "cell_temperature[]" << std::endl;
    for (size_t i = 0; i < v.cell_temperature.size(); ++i)
    {
      s << indent << "  cell_temperature[" << i << "]: ";
      
      Printer<float>::stream(s, indent + "  ", v.cell_temperature[i]);
      
    }
    
    

    s << indent << "location: ";
    
    Printer< std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.location);
    

    
    

    s << indent << "serial_number: ";
    
    Printer< std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.serial_number);
    

    
  }
  
};

} // namespace message_operations
} // namespace ros

#endif // SENSOR_MSGS_MESSAGE_BATTERYSTATE