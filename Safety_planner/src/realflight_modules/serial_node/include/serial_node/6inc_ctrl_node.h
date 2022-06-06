#ifndef _6INC_CTRL_NODE_H_
#define _6INC_CTRL_NODE_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial/serial.h"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <uav_utils/utils.h>
#include <uav_utils/converters.h>
#include <sensor_msgs/Imu.h>
#include "serial_node/my_msg.h"
#include "serial_node/rawpose.h"
#include "serial_node/rc.h"
#include "serial_node/extforce.h"
#include "mavlink/common/mavlink.h"
#include "mavlink/common/mavlink_msg_attitude_quaternion.h"
#include "mavlink/mavlink_types.h"
#include "mavlink/common/mavlink_msg_position_target_local_ned.h"

#include <Eigen/Core>
#include <Eigen/Dense>


using namespace std;
#define buf_size 128

ros::Subscriber odom_sub;
ros::Subscriber cmd_sub;
ros::Subscriber rc_sub;

serial::Serial my_serial;

ros::Publisher read_pub;
ros::Publisher rawpose_pub;
ros::Publisher rc_pub;
ros::Publisher extf_pub;
ros::Publisher traj_start_trigger_pub;
// raw
serial_node::my_msg ros_msg;  //接收到的数据
serial_node::rawpose raw_pose;
// ros
sensor_msgs::Imu imu_msg;
nav_msgs::Odometry nmg;
serial_node::rc rc_msg;
serial_node::extforce extf_msg;
//


// function
void publish_trigger(const nav_msgs::Odometry &odom_msg);
void pub_rc(ros::NodeHandle nh);
void process_mode();
void pub_raw(ros::NodeHandle nh);
int initSerial();
void processFromSerial();
int mavlinkCharParse(uint8_t value);
void vio_pos_vel(Eigen::Vector3d p,Eigen::Vector3d v);
void vio_att_vel(Eigen::Quaterniond q,Eigen::Vector3d w);
void vio_send(Eigen::Vector3d p,Eigen::Vector3d v,Eigen::Quaterniond q,Eigen::Vector3d w);
void plan_send(Eigen::Vector3d p,Eigen::Vector3d v,Eigen::Vector3d a,double yaw,double yaw_rate);
void plan_jerk_send(Eigen::Vector3d j);
void sendRCCallback(serial_node::rcConstPtr pMsg);
void sendPLANCallback(quadrotor_msgs::PositionCommandConstPtr pMsg);
void sendVIOCallback(nav_msgs::OdometryConstPtr pMsg);
#endif