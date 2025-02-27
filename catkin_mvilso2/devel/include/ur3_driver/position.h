// Generated by gencpp from file ur3_driver/position.msg
// DO NOT EDIT!


#ifndef UR3_DRIVER_MESSAGE_POSITION_H
#define UR3_DRIVER_MESSAGE_POSITION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ur3_driver
{
template <class ContainerAllocator>
struct position_
{
  typedef position_<ContainerAllocator> Type;

  position_()
    : position()
    , isReady(false)  {
    }
  position_(const ContainerAllocator& _alloc)
    : position(_alloc)
    , isReady(false)  {
  (void)_alloc;
    }



   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _position_type;
  _position_type position;

   typedef uint8_t _isReady_type;
  _isReady_type isReady;





  typedef boost::shared_ptr< ::ur3_driver::position_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ur3_driver::position_<ContainerAllocator> const> ConstPtr;

}; // struct position_

typedef ::ur3_driver::position_<std::allocator<void> > position;

typedef boost::shared_ptr< ::ur3_driver::position > positionPtr;
typedef boost::shared_ptr< ::ur3_driver::position const> positionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ur3_driver::position_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ur3_driver::position_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ur3_driver

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'ur3_driver': ['/home/ur3/catkin_mvilso2/src/lab2andDriver/drivers/ur3_driver/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ur3_driver::position_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ur3_driver::position_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur3_driver::position_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur3_driver::position_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur3_driver::position_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur3_driver::position_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ur3_driver::position_<ContainerAllocator> >
{
  static const char* value()
  {
    return "86ad7fb432e90c4fcc2fa98c89466a9b";
  }

  static const char* value(const ::ur3_driver::position_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x86ad7fb432e90c4fULL;
  static const uint64_t static_value2 = 0xcc2fa98c89466a9bULL;
};

template<class ContainerAllocator>
struct DataType< ::ur3_driver::position_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ur3_driver/position";
  }

  static const char* value(const ::ur3_driver::position_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ur3_driver::position_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[] position\n\
bool isReady\n\
";
  }

  static const char* value(const ::ur3_driver::position_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ur3_driver::position_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.position);
      stream.next(m.isReady);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct position_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ur3_driver::position_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ur3_driver::position_<ContainerAllocator>& v)
  {
    s << indent << "position[]" << std::endl;
    for (size_t i = 0; i < v.position.size(); ++i)
    {
      s << indent << "  position[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.position[i]);
    }
    s << indent << "isReady: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.isReady);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UR3_DRIVER_MESSAGE_POSITION_H
