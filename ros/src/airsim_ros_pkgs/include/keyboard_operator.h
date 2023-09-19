#ifndef AIRSIM_ROS_KEYBOARD_OPERATOR_H
#define AIRSIM_ROS_KEYBOARD_OPERATOR_H
#endif //AIRSIM_ROS_KEYBOARD_OPERATOR_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <chrono>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <std_srvs/Empty.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#define KEYBOARD_O 0x6f
#define KEYBOARD_P 0x70
#define KEYBOARD_K 0x6b
#define KEYBOARD_L 0x6c
#define KEYBOARD_W 0x77
#define KEYBOARD_S 0x73
#define KEYBOARD_A 0x61
#define KEYBOARD_D 0x64
#define KEYBOARD_Z 0x7A
#define KEYBOARD_C 0x63
#define KEYBOARD_Q 0x71
#define KEYBOARD_E 0x65

namespace keyboard_operator
{
void quit(int sig);

class KeyboardOperator
{
private:
    ros::NodeHandle nh;
    //ros::Publisher pose_drone;
    geometry_msgs::PoseStamped pose_des_keyboard;
    ros::ServiceServer server_key_input;
    std::string vehicle_name;
    std::string world_frame_id;
    bool isENU_;
    ros::Publisher airsim_local_goal_pub;

    double increment_xyz = 0.02;
    double increment_yaw = 3.141592 / 12;
    bool move_mav(double dx, double dy, double dz, double dyaw);

public:
    KeyboardOperator();
    void keyLoop();
};

}
