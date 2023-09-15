//
// Created by larr-planning on 23. 9. 15.
//
#include "pd_position_controller_simple_topic.h"

using namespace std;

PDController::PDController()
    : nh_("~"), has_odom_(false), has_local_goal_(false)
{
    nh_.param<double>("kp_x", params_.kp_x_, 1.0);
    nh_.param<double>("kp_y", params_.kp_y_, 1.0);
    nh_.param<double>("kp_z", params_.kp_z_, 1.0);
    nh_.param<double>("kp_yaw", params_.kp_yaw_, 1.0);
    nh_.param<double>("kd_x", params_.kd_x_, 0.05);
    nh_.param<double>("kd_y", params_.kd_y_, 0.05);
    nh_.param<double>("kd_z", params_.kd_z_, 0.05);
    nh_.param<double>("kd_yaw", params_.kd_yaw_, 0.05);
    nh_.param<double>("max_vel_horz_abs", params_.max_vel_horz_abs_, 1.0);
    nh_.param<double>("max_vel_vert_abs", params_.max_vel_vert_abs_, 0.5);
    nh_.param<double>("max_yaw_rate_degree", params_.max_yaw_rate_degree_, 10);
    nh_.param<string>("vehicle_name", params_.vehicle_name_, "drone");
    nh_.param<double>("control_update_rate", control_update_rate_, 100);
    nh_.param<double>("cmd_smoothing_weight", params_.smoothing_weight_, 0.3);
    nh_.param<string>("odom_frame_id", params_.odom_frame_id_, "world_ned");
    nh_.param<string>("goal_frame_id", params_.goal_frame_id_, "world_enu");
    vel_cmd_ = airsim_ros_pkgs::VelCmd();

    string goal_frame_id_;
    string odom_frame_id_;

    if (params_.odom_frame_id_ == "world_enu") {
        odom_frame_id_ = "odom_local_enu";
    }
    else {
        odom_frame_id_ = "odom_local_ned";
    }

    if (params_.goal_frame_id_ == "world_enu") {
        goal_frame_id_ = "local_goal_enu";
    }
    else {
        goal_frame_id_ = "local_goal_ned";
    }

    // ROS Subscribers
    airsim_odom_sub_ = nh_.subscribe("/airsim_node/" + params_.vehicle_name_ + "/" + odom_frame_id_, 1, &PDController::airsim_odom_cb, this);
    airsim_local_goal_sub_ = nh_.subscribe("/airsim_node/" + params_.vehicle_name_ + "/" + goal_frame_id_, 1, &PDController::airsim_local_goal_cb, this);
    // ROS Publishers
    airsim_vel_cmd_world_frame_pub_ = nh_.advertise<airsim_ros_pkgs::VelCmd>("/airsim_node/" + params_.vehicle_name_ + "/vel_cmd_world_frame", 1);
}
void PDController::airsim_odom_cb(const nav_msgs::Odometry& odom_msg)
{
    has_odom_ = true;
    //curr_odom_ = odom_msg;
    if (params_.odom_frame_id_ == "world_enu") {
        curr_position_.x = odom_msg.pose.pose.position.y;
        curr_position_.y = odom_msg.pose.pose.position.x;
        curr_position_.z = -odom_msg.pose.pose.position.z;
        curr_position_.yaw = 0.5 * M_PI - utils::get_yaw_from_quat_msg(odom_msg.pose.pose.orientation);
    }
    else {
        curr_position_.x = odom_msg.pose.pose.position.x;
        curr_position_.y = odom_msg.pose.pose.position.y;
        curr_position_.z = odom_msg.pose.pose.position.z;
        curr_position_.yaw = utils::get_yaw_from_quat_msg(odom_msg.pose.pose.orientation);
    }
}
void PDController::airsim_local_goal_cb(const nav_msgs::Odometry& odom_msg)
{
    has_local_goal_ = true;
    if (params_.goal_frame_id_ == "world_enu") {
        //curr_local_goal_ = odom_msg;
        curr_goal_position_.x = odom_msg.pose.pose.position.y;
        curr_goal_position_.y = odom_msg.pose.pose.position.x;
        curr_goal_position_.z = -odom_msg.pose.pose.position.z;
        curr_goal_position_.yaw = 0.5 * M_PI - utils::get_yaw_from_quat_msg(odom_msg.pose.pose.orientation);
    }
    else {
        //curr_local_goal_ = odom_msg;
        curr_goal_position_.x = odom_msg.pose.pose.position.x;
        curr_goal_position_.y = odom_msg.pose.pose.position.y;
        curr_goal_position_.z = odom_msg.pose.pose.position.z;
        curr_goal_position_.yaw = utils::get_yaw_from_quat_msg(odom_msg.pose.pose.orientation);
    }
}
void PDController::compute_control_cmd()
{
    curr_error_.x = curr_goal_position_.x - curr_position_.x;
    curr_error_.y = curr_goal_position_.y - curr_position_.y;
    curr_error_.z = curr_goal_position_.z - curr_position_.z;
    curr_error_.yaw = math_common::angular_dist(curr_position_.yaw, curr_goal_position_.yaw);

    double p_term_x = params_.kp_x_ * curr_error_.x;
    double p_term_y = params_.kp_y_ * curr_error_.y;
    double p_term_z = params_.kp_z_ * curr_error_.z;
    double p_term_yaw = params_.kp_yaw_ * curr_error_.yaw;

    double d_term_x = params_.kd_x_ * prev_error_.x;
    double d_term_y = params_.kd_y_ * prev_error_.y;
    double d_term_z = params_.kd_z_ * prev_error_.z;
    double d_term_yaw = params_.kp_yaw_ * prev_error_.yaw;

    prev_error_ = curr_error_;

    vel_cmd_.twist.linear.x = (1 - params_.smoothing_weight_) * (p_term_x + d_term_x) + params_.smoothing_weight_ * (vel_cmd_.twist.linear.x);
    vel_cmd_.twist.linear.y = (1 - params_.smoothing_weight_) * (p_term_y + d_term_y) + params_.smoothing_weight_ * (vel_cmd_.twist.linear.y);
    vel_cmd_.twist.linear.z = (1 - params_.smoothing_weight_) * (p_term_z + d_term_z) + params_.smoothing_weight_ * (vel_cmd_.twist.linear.z);
    vel_cmd_.twist.angular.z = p_term_yaw + d_term_yaw;
    // todo

    //if(params_.world_frame_id == "world_enu")
    //{
    //    std::swap(vel_cmd_.twist.linear.x, vel_cmd_.twist.linear.y);
    //    vel_cmd_.twist.linear.z = -vel_cmd_.twist.linear.z;
    //    vel_cmd_.twist.angular.z = -vel_cmd_.twist.angular.z;
    //}
}
void PDController::enforce_dynamic_constraints()
{
    double vel_norm_horz = sqrt((vel_cmd_.twist.linear.x * vel_cmd_.twist.linear.x) + (vel_cmd_.twist.linear.y * vel_cmd_.twist.linear.y));

    if (vel_norm_horz > params_.max_vel_horz_abs_) {
        vel_cmd_.twist.linear.x = (vel_cmd_.twist.linear.x / vel_norm_horz) * params_.max_vel_horz_abs_;
        vel_cmd_.twist.linear.y = (vel_cmd_.twist.linear.y / vel_norm_horz) * params_.max_vel_horz_abs_;
    }

    if (std::fabs(vel_cmd_.twist.linear.z) > params_.max_vel_vert_abs_) {
        // todo just add a sgn funciton in common utils? return double to be safe.
        // template <typename T> double sgn(T val) { return (T(0) < val) - (val < T(0)); }
        vel_cmd_.twist.linear.z = (vel_cmd_.twist.linear.z / std::fabs(vel_cmd_.twist.linear.z)) * params_.max_vel_vert_abs_;
    }
    // todo yaw limits
    if (std::fabs(vel_cmd_.twist.linear.z) > params_.max_yaw_rate_degree_) {
        // todo just add a sgn funciton in common utils? return double to be safe.
        // template <typename T> double sgn(T val) { return (T(0) < val) - (val < T(0)); }
        vel_cmd_.twist.linear.z = (vel_cmd_.twist.linear.z / std::fabs(vel_cmd_.twist.linear.z)) * params_.max_yaw_rate_degree_;
    }
}

void PDController::publish_cmd()
{
    airsim_vel_cmd_world_frame_pub_.publish(vel_cmd_);
}

void PDController::run()
{
    ros::Rate loop_rate(control_update_rate_);
    while (ros::ok()) {
        if (has_local_goal_) {
            compute_control_cmd();
            enforce_dynamic_constraints();
            publish_cmd();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}