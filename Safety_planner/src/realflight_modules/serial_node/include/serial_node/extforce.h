// Generated by gencpp from file serial_node/extforce.msg
// DO NOT EDIT!


#ifndef SERIAL_NODE_MESSAGE_EXTFORCE_H
#define SERIAL_NODE_MESSAGE_EXTFORCE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

namespace serial_node
{
template <class ContainerAllocator>
struct extforce_
{
  typedef extforce_<ContainerAllocator> Type;

  extforce_()
    : exf_x(0.0)
    , exf_y(0.0)
    , exf_z(0.0)  {
    }
  extforce_(const ContainerAllocator& _alloc)
    : exf_x(0.0)
    , exf_y(0.0)
    , exf_z(0.0)  {
  (void)_alloc;
    }


   typedef float _exf_x_type;
  _exf_x_type exf_x;

   typedef float _exf_y_type;
  _exf_y_type exf_y;

   typedef float _exf_z_type;
  _exf_z_type exf_z;





  typedef boost::shared_ptr< ::serial_node::extforce_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::serial_node::extforce_<ContainerAllocator> const> ConstPtr;

}; // struct extforce_

typedef ::serial_node::extforce_<std::allocator<void> > extforce;

typedef boost::shared_ptr< ::serial_node::extforce > extforcePtr;
typedef boost::shared_ptr< ::serial_node::extforce const> extforceConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::serial_node::extforce_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::serial_node::extforce_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::serial_node::extforce_<ContainerAllocator1> & lhs, const ::serial_node::extforce_<ContainerAllocator2> & rhs)
{
  return lhs.exf_x == rhs.exf_x &&
    lhs.exf_y == rhs.exf_y &&
    lhs.exf_z == rhs.exf_z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::serial_node::extforce_<ContainerAllocator1> & lhs, const ::serial_node::extforce_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace serial_node

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::serial_node::extforce_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::serial_node::extforce_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::serial_node::extforce_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::serial_node::extforce_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::serial_node::extforce_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::serial_node::extforce_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::serial_node::extforce_<ContainerAllocator> >
{
  static const char* value()
  {
    return "154390d2d287758b9c984a5d84ea2d34";
  }

  static const char* value(const ::serial_node::extforce_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x154390d2d287758bULL;
  static const uint64_t static_value2 = 0x9c984a5d84ea2d34ULL;
};

template<class ContainerAllocator>
struct DataType< ::serial_node::extforce_<ContainerAllocator> >
{
  static const char* value()
  {
    return "serial_node/extforce";
  }

  static const char* value(const ::serial_node::extforce_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::serial_node::extforce_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 exf_x\n"
"float32 exf_y\n"
"float32 exf_z\n"
;
  }

  static const char* value(const ::serial_node::extforce_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::serial_node::extforce_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.exf_x);
      stream.next(m.exf_y);
      stream.next(m.exf_z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct extforce_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::serial_node::extforce_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::serial_node::extforce_<ContainerAllocator>& v)
  {
    s << indent << "exf_x: ";
    Printer<float>::stream(s, indent + "  ", v.exf_x);
    s << indent << "exf_y: ";
    Printer<float>::stream(s, indent + "  ", v.exf_y);
    s << indent << "exf_z: ";
    Printer<float>::stream(s, indent + "  ", v.exf_z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SERIAL_NODE_MESSAGE_EXTFORCE_H
