// Generated by gencpp from file kortex_driver/FollowCartesianTrajectoryActionGoal.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_FOLLOWCARTESIANTRAJECTORYACTIONGOAL_H
#define KORTEX_DRIVER_MESSAGE_FOLLOWCARTESIANTRAJECTORYACTIONGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <actionlib_msgs/GoalID.h>
#include <kortex_driver/FollowCartesianTrajectoryGoal.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct FollowCartesianTrajectoryActionGoal_
{
  typedef FollowCartesianTrajectoryActionGoal_<ContainerAllocator> Type;

  FollowCartesianTrajectoryActionGoal_()
    : header()
    , goal_id()
    , goal()  {
    }
  FollowCartesianTrajectoryActionGoal_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , goal_id(_alloc)
    , goal(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::actionlib_msgs::GoalID_<ContainerAllocator>  _goal_id_type;
  _goal_id_type goal_id;

   typedef  ::kortex_driver::FollowCartesianTrajectoryGoal_<ContainerAllocator>  _goal_type;
  _goal_type goal;





  typedef boost::shared_ptr< ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator> const> ConstPtr;

}; // struct FollowCartesianTrajectoryActionGoal_

typedef ::kortex_driver::FollowCartesianTrajectoryActionGoal_<std::allocator<void> > FollowCartesianTrajectoryActionGoal;

typedef boost::shared_ptr< ::kortex_driver::FollowCartesianTrajectoryActionGoal > FollowCartesianTrajectoryActionGoalPtr;
typedef boost::shared_ptr< ::kortex_driver::FollowCartesianTrajectoryActionGoal const> FollowCartesianTrajectoryActionGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator1> & lhs, const ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.goal_id == rhs.goal_id &&
    lhs.goal == rhs.goal;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator1> & lhs, const ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "60bf486344cb17b10eba10bac039409d";
  }

  static const char* value(const ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x60bf486344cb17b1ULL;
  static const uint64_t static_value2 = 0x0eba10bac039409dULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/FollowCartesianTrajectoryActionGoal";
  }

  static const char* value(const ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalID goal_id\n"
"FollowCartesianTrajectoryGoal goal\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: actionlib_msgs/GoalID\n"
"# The stamp should store the time at which this goal was requested.\n"
"# It is used by an action server when it tries to preempt all\n"
"# goals that were requested before a certain time\n"
"time stamp\n"
"\n"
"# The id provides a way to associate feedback and\n"
"# result message with specific goal requests. The id\n"
"# specified must be unique.\n"
"string id\n"
"\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/FollowCartesianTrajectoryGoal\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"#The trajectory to follow\n"
"CartesianWaypoint[] trajectory\n"
"duration goal_time_tolerance\n"
"bool use_optimal_blending\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/CartesianWaypoint\n"
"\n"
"Pose pose\n"
"uint32 reference_frame\n"
"float32 maximum_linear_velocity\n"
"float32 maximum_angular_velocity\n"
"float32 blending_radius\n"
"================================================================================\n"
"MSG: kortex_driver/Pose\n"
"\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
"float32 theta_x\n"
"float32 theta_y\n"
"float32 theta_z\n"
;
  }

  static const char* value(const ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.goal_id);
      stream.next(m.goal);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FollowCartesianTrajectoryActionGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::FollowCartesianTrajectoryActionGoal_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "goal_id: ";
    s << std::endl;
    Printer< ::actionlib_msgs::GoalID_<ContainerAllocator> >::stream(s, indent + "  ", v.goal_id);
    s << indent << "goal: ";
    s << std::endl;
    Printer< ::kortex_driver::FollowCartesianTrajectoryGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.goal);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_FOLLOWCARTESIANTRAJECTORYACTIONGOAL_H
