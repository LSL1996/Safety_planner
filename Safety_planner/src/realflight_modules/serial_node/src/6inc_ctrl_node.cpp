#include <ros/ros.h>
#include <iostream>
#include <string>
#include "serial_node/6inc_ctrl_node.h"
using namespace std;

void publish_trigger(const nav_msgs::Odometry &odom_msg)
{
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.pose = odom_msg.pose.pose;
    traj_start_trigger_pub.publish(msg);
}

void pub_rc(ros::NodeHandle nh)
{
    traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("traj_start_trigger", 10);  ///
}

void process_mode()
{
    /// 悬停模式 而且 自主控制命令  轨迹开始

}

void pub_raw(ros::NodeHandle nh)
{
    read_pub = nh.advertise<sensor_msgs::Imu>("rawimu",1000);
    rawpose_pub = nh.advertise<sensor_msgs::Imu>("rawpose",1000);
    rc_pub = nh.advertise<serial_node::rc>("rc",1000);
}

int initSerial()
{
    try {
        my_serial.setPort("/dev/ttyUSB0");
        my_serial.setBaudrate(921600);
        serial::Timeout to=serial::Timeout::simpleTimeout(100);
        my_serial.setTimeout(to);
        my_serial.open();
    }
    catch(serial::IOException& ioe)
    {
        ROS_INFO_STREAM("Unable to open serial port ");
        return 0;
    }
    if(my_serial.isOpen())
        ROS_INFO_STREAM("serial port is opened");
    else
        return 0;
    return 1;
};

void processFromSerial()
{
    ros::Rate loop_rate(200);
    while(ros::ok())
    {
        int isparse = 0;
        unsigned char totalbuf[buf_size];
        size_t n=my_serial.available();
        //ROS_INFO_STREAM("n                    "<< int(n));
        my_serial.read(totalbuf,n);

        for(size_t i=0;i<n;i++) {
            isparse=mavlinkCharParse(totalbuf[i]);

            if (isparse == 1) {
                read_pub.publish(imu_msg);
            }
            else if(isparse == 2)  {
                rawpose_pub.publish(imu_msg);
            }
            else if(isparse == 3){
                rc_pub.publish(rc_msg);
            }
            else if(isparse == 4){
                extf_pub.publish(extf_msg);
                break;
            }
        }
        //ROS_INFO_STREAM("ros_msg.gyro_x  "<<ros_msg.gyro_x);
        ros::spinOnce();
        loop_rate.sleep();
    }
};


int mavlinkCharParse(uint8_t value)
{
    static mavlink_message_t RxMsg;
    static mavlink_status_t  RxStatus = {msg_received:0,buffer_overrun:0,parse_error:0,packet_idx:0,
            current_rx_seq:0,current_tx_seq:0,packet_rx_success_count:0,
            packet_rx_drop_count:0,
            parse_state:MAVLINK_PARSE_STATE_UNINIT};
    mavlink_message_t msg;
    mavlink_status_t  status;
    if (mavlink_frame_char_buffer(&RxMsg, &RxStatus, value, &msg, &status) == MAVLINK_FRAMING_OK) {
        switch(RxMsg.msgid){
            case MAVLINK_MSG_ID_SCALED_IMU:{
                mavlink_scaled_imu_t stm32_msg;
                mavlink_msg_scaled_imu_decode(&RxMsg,&stm32_msg);

                imu_msg.header.stamp=  ros::Time::now();
                imu_msg.header.frame_id = "base_link";

                imu_msg.angular_velocity.x = stm32_msg.xgyro/1000.0;
                imu_msg.angular_velocity.y = -stm32_msg.ygyro/1000.0;
                imu_msg.angular_velocity.z = -stm32_msg.zgyro/1000.0;
                imu_msg.linear_acceleration.x = stm32_msg.xacc/1000.0;
                imu_msg.linear_acceleration.y = -stm32_msg.yacc/1000.0;
                imu_msg.linear_acceleration.z = -stm32_msg.zacc/1000.0;

                imu_msg.orientation.x = 0;
                imu_msg.orientation.y = 0;
                imu_msg.orientation.z = 0;
                imu_msg.orientation.w = 0;
                // ROS_INFO_STREAM("1 succesful  ");
                return 1;
            }
            case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:{
                mavlink_attitude_quaternion_t maqt;
                mavlink_msg_attitude_quaternion_decode(&RxMsg,&maqt);

                imu_msg.header.frame_id = "base_link";
                imu_msg.header.stamp =  ros::Time::now();
                imu_msg.orientation.x = maqt.q1;
                imu_msg.orientation.y = maqt.q2;
                imu_msg.orientation.z = maqt.q3;
                imu_msg.orientation.w = maqt.q4;

                //ROS_INFO_STREAM("2 succesful  ");
                return 2;
            }
            case MAVLINK_MSG_ID_SET_MODE:{
                mavlink_set_mode_t rcmsg;
                mavlink_msg_set_mode_decode(&RxMsg,&rcmsg);

                rc_msg.ishover=rcmsg.target_system;
                rc_msg.isauto= rcmsg.base_mode;

                //ROS_INFO_STREAM("3 succesful  ");
                return 3;
            }
            case MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE:{
                mavlink_vision_speed_estimate_t extfmsg;
                mavlink_msg_vision_speed_estimate_decode(&RxMsg,&extfmsg);

                extf_msg.exf_x = extfmsg.x  / 1000.0;
                extf_msg.exf_y = -extfmsg.y / 1000.0;
                extf_msg.exf_z = -extfmsg.z / 1000.0;
                return 4;
            }
            default:
                break;
        }
    }
    return -1;
};

///
void vio_pos_vel(Eigen::Vector3d p,Eigen::Vector3d v)
{
    mavlink_message_t mmt;
    mavlink_vision_position_estimate_t vio_pv;
    vio_pv.usec = ros::Time::now().toSec();
    vio_pv.x=p(0)*1000;
    vio_pv.y=p(1)*1000;
    vio_pv.z=p(2)*1000;
    vio_pv.roll=v(0)*1000;
    vio_pv.pitch=v(1)*1000;
    vio_pv.yaw=v(2)*1000;
    mavlink_msg_vision_position_estimate_encode(0,0,&mmt,&vio_pv);
    unsigned char send_buf[128];
    int buf_len = mavlink_msg_to_send_buffer(send_buf, &mmt);
    my_serial.write(send_buf,sizeof(send_buf));
};
void vio_att_vel(Eigen::Quaterniond q,Eigen::Vector3d w)
{
    mavlink_message_t mmt;
    mavlink_attitude_quaternion_t vio_qw;

    vio_qw.time_boot_ms = ros::Time::now().toSec();
    vio_qw.q1=q.w()*1000;
    vio_qw.q2=q.x()*1000;
    vio_qw.q3=q.y()*1000;
    vio_qw.q4=q.z()*1000;
    vio_qw.rollspeed = w(0)*1000;
    vio_qw.pitchspeed =w(1)*1000;
    vio_qw.yawspeed = w(2)*1000;

    mavlink_msg_attitude_quaternion_encode(0,0, &mmt, &vio_qw);
    unsigned char send_buf[128];
    int buf_len = mavlink_msg_to_send_buffer(send_buf, &mmt);
    my_serial.write(send_buf,sizeof(send_buf));
};

void vio_send(Eigen::Vector3d p,Eigen::Vector3d v,Eigen::Quaterniond q,Eigen::Vector3d w)
{
    vio_pos_vel(p,v);
    vio_att_vel(q,w);
};

void plan_send(Eigen::Vector3d p,Eigen::Vector3d v,Eigen::Vector3d a,double yaw,double yaw_rate)
{
    mavlink_message_t mmt;
    mavlink_position_target_local_ned_t plan_data;

    plan_data.time_boot_ms = ros::Time::now().toSec();
    plan_data.x= p(0)*1000;
    plan_data.y= p(1)*1000;
    plan_data.z= p(2)*1000;
    plan_data.vx= v(0)*1000;
    plan_data.vy= v(1)*1000;
    plan_data.vz= v(2)*1000;
    plan_data.afx= a(0)*1000;
    plan_data.afy= a(1)*1000;
    plan_data.afz= a(2)*1000;
    plan_data.yaw= yaw*1000;
    plan_data.yaw_rate= yaw_rate*1000;
    plan_data.type_mask= 0;
    plan_data.coordinate_frame= 0;

    mavlink_msg_position_target_local_ned_encode(0,0,&mmt,&plan_data);
    unsigned char send_buf[128];
    int buf_len = mavlink_msg_to_send_buffer(send_buf, &mmt);
    my_serial.write(send_buf,sizeof(send_buf));
};
void plan_jerk_send(Eigen::Vector3d j)
{
    mavlink_message_t mmt;
    mavlink_vision_speed_estimate_t planjerk;
    planjerk.usec = ros::Time::now().toSec();
    planjerk.x = j(0)*1000;
    planjerk.y = j(1)*1000;
    planjerk.z = j(2)*1000;

    mavlink_msg_vision_speed_estimate_encode(0,0,&mmt,&planjerk);
    unsigned char send_buf[128];
    int buf_len = mavlink_msg_to_send_buffer(send_buf, &mmt);
    my_serial.write(send_buf,sizeof(send_buf));
}
void sendVIOCallback(nav_msgs::OdometryConstPtr pMsg)
{
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Quaterniond q;
    Eigen::Vector3d w;

    nmg = *pMsg;

    uav_utils::extract_odometry(pMsg, p, v, q, w);   ///

    /// send
    vio_send(p,v,q,w);   /// new
    //ROS_INFO_STREAM("                   send vio");
}


void sendPLANCallback(quadrotor_msgs::PositionCommandConstPtr pMsg)
{
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    double yaw;
    double yaw_rate;

    quadrotor_msgs::PositionCommand msg;
    msg = *pMsg;

    p(0) = msg.position.x;
    p(1) = msg.position.y;
    p(2) = msg.position.z;
    //cout << "p   \n" <<  p << endl;
    v(0) = msg.velocity.x;
    v(1) = msg.velocity.y;
    v(2) = msg.velocity.z;
    //cout << "v  \n" << v <<endl;
    a(0) = msg.acceleration.x;
    a(1) = msg.acceleration.y;
    a(2) = msg.acceleration.z;

    //j(0) = msg.jerk.x;
    //j(1) = msg.jerk.y;
    //j(2) = msg.jerk.z;

    j(0) = 0;
    j(1) = 0;
    j(2) = 0;

    yaw = uav_utils::normalize_angle(msg.yaw);
    yaw_rate = msg.yaw_dot;

    ///

    plan_send(p,v,a,yaw,yaw_rate);  /// new
    plan_jerk_send(j);

    //ROS_INFO_STREAM("          send plan  ");
}


void sendRCCallback(serial_node::rcConstPtr pMsg)
{
    int is_hover;
    int is_auto;

    is_auto = pMsg->isauto;
    is_hover = pMsg->ishover;
    if(is_auto == 1 && is_hover == 1) {
        //ROS_INFO_STREAM("------------------plan begin----------------------- " );
        publish_trigger(nmg);
    }

}


///
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sixinc_ctrl");
    ros::NodeHandle nh("sixinc");
    //pub_raw(nh);
    read_pub = nh.advertise<sensor_msgs::Imu>("rawimu",1000);
    rawpose_pub = nh.advertise<sensor_msgs::Imu>("rawpose",1000);
    rc_pub = nh.advertise<serial_node::rc>("rc",10);
    extf_pub = nh.advertise<serial_node::extforce>("extforce",100);

    //pub_rc(nh);
    traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("traj_start_trigger", 10);  ///

    ///  vio：实际的信息  p v q dq       world
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/vins_estimator/imu_propagate",100, sendVIOCallback);

    /// plan: 期望信息 p v a da yaw dyaw     world
    cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/position_cmd",100,sendPLANCallback);

    /// 遥控指令
    rc_sub = nh.subscribe<serial_node::rc>("rc", 100, sendRCCallback);

    int isopen = initSerial();
    if(isopen)
        processFromSerial();

    ros::Duration(1.0).sleep();
    ros::spin();
    return 0;
}
