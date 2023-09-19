#include "keyboard_operator.h"

using namespace std;
using namespace Eigen;
using namespace keyboard_operator;
KeyboardOperator::KeyboardOperator():nh("~"){
    nh.param<string>("vehicle_name",vehicle_name,"");
    nh.param<string>("world_frame_id",world_frame_id,"world_ned");
    pose_des_keyboard.pose.position.x = 0;
    pose_des_keyboard.pose.position.y = 0;
    pose_des_keyboard.pose.position.z = 0;
    pose_des_keyboard.pose.orientation.x = 0;
    pose_des_keyboard.pose.orientation.y = 0;
    pose_des_keyboard.pose.orientation.z = 0;
    pose_des_keyboard.pose.orientation.w = 1;
    string goal_frame_id;
    if(world_frame_id=="world_ned")
    {
        goal_frame_id="local_goal_ned";
        isENU_ =false;
    }
    else
    {
        goal_frame_id="local_goal_enu";
        isENU_ = true;
        tf::Quaternion q_init = tf::Quaternion();
        q_init.setRPY(0,0,1.570796326794897);
        pose_des_keyboard.pose.orientation.x = q_init.getX();
        pose_des_keyboard.pose.orientation.y = q_init.getY();
        pose_des_keyboard.pose.orientation.z = q_init.getZ();
        pose_des_keyboard.pose.orientation.w = q_init.getW();

    }
    airsim_local_goal_pub =nh.advertise<nav_msgs::Odometry>("/airsim_node/"+vehicle_name+"/"+goal_frame_id,2);
}
bool KeyboardOperator::move_mav(double dx, double dy, double dz, double dyaw)
{
    Vector3d dpose(dx,dy,dz);
    tf::Quaternion q_cur_des;
    q_cur_des.setX(pose_des_keyboard.pose.orientation.x);
    q_cur_des.setY(pose_des_keyboard.pose.orientation.y);
    q_cur_des.setZ(pose_des_keyboard.pose.orientation.z);
    q_cur_des.setW(pose_des_keyboard.pose.orientation.w);
    tf::Vector3 t_cur_des(pose_des_keyboard.pose.position.x,pose_des_keyboard.pose.position.y,pose_des_keyboard.pose.position.z);
    tf::Transform transform_cur;
    transform_cur.setIdentity();
    transform_cur.setRotation(q_cur_des);

    //transform matrix
    Eigen::Isometry3d  Twb;
    tf::transformTFToEigen(transform_cur,Twb);
    Vector3d dpose_w = Twb*dpose;

    pose_des_keyboard.pose.position.x += dpose_w[0];
    pose_des_keyboard.pose.position.y += dpose_w[1];
    pose_des_keyboard.pose.position.z += dpose_w[2];

    double roll, pitch, yaw;
    tf::Matrix3x3(q_cur_des).getEulerYPR(yaw,pitch,roll);
    yaw += dyaw;
    tf::Quaternion q_des = tf::Quaternion();
    q_des.setRPY(roll,pitch,yaw);
    pose_des_keyboard.pose.orientation.x = q_des.getX();
    pose_des_keyboard.pose.orientation.y = q_des.getY();
    pose_des_keyboard.pose.orientation.z = q_des.getZ();
    pose_des_keyboard.pose.orientation.w = q_des.getW();

    nav_msgs::Odometry local_goal_info;
    local_goal_info.header.frame_id=world_frame_id;
    local_goal_info.pose.pose.position.x = pose_des_keyboard.pose.position.x;
    local_goal_info.pose.pose.position.y = pose_des_keyboard.pose.position.y;
    local_goal_info.pose.pose.position.z = pose_des_keyboard.pose.position.z;
    local_goal_info.pose.pose.orientation.x = pose_des_keyboard.pose.orientation.x;
    local_goal_info.pose.pose.orientation.y = pose_des_keyboard.pose.orientation.y;
    local_goal_info.pose.pose.orientation.z = pose_des_keyboard.pose.orientation.z;
    local_goal_info.pose.pose.orientation.w = pose_des_keyboard.pose.orientation.w;
    airsim_local_goal_pub.publish(local_goal_info);
    return true;
}

int kfd = 0;
struct termios cooked, raw;
void keyboard_operator::quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}


void KeyboardOperator::keyLoop()
{
    char c;
    bool dirty = false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the turtle.");

    for(;;)
    {
        //get the next event from the keyboard
        if(read(kfd,&c,1)<0)
        {
            perror("read():");
            exit(-1);
        }
        Eigen::Vector4d move_pose; move_pose.setZero();
        if (isENU_ == false)
        {
            switch (c)
            {
            case KEYBOARD_O :{
                increment_xyz -= 0.02;
                ROS_INFO("Decreased linear step. current: %f",increment_xyz);
                break;
            }
            case KEYBOARD_P :{
                increment_xyz += 0.02;
                ROS_INFO("Increased linear step. current: %f",increment_xyz);
                break;
            }
            case KEYBOARD_K : {
                increment_yaw -= 3.141592/12;
                ROS_INFO("Decreased yaw step. current: %f", increment_yaw);
                break;
            }
            case KEYBOARD_L : {
                increment_yaw += 3.141592/12;
                ROS_INFO("Increased yaw step. current: %f", increment_yaw);
                break;
            }
            case KEYBOARD_W: {move_pose(0) = increment_xyz; break;}
            case KEYBOARD_S: {move_pose(0) = -increment_xyz; break;}
            case KEYBOARD_A: {move_pose(1) = -increment_xyz; break;}
            case KEYBOARD_D: {move_pose(1) = increment_xyz; break;}
            case KEYBOARD_Z: {move_pose(2) = increment_xyz; break;}
            case KEYBOARD_C: {move_pose(2) = -increment_xyz; break;}
            case KEYBOARD_Q: {move_pose(3) = -increment_xyz; break;}
            case KEYBOARD_E: {move_pose(3) = increment_xyz; break;}
            }
        }
        else
        {
            switch (c)
            {
            case KEYBOARD_O :{
                increment_xyz -= 0.02;
                ROS_INFO("Decreased linear step. current: %f",increment_xyz);
                break;
            }
            case KEYBOARD_P :{
                increment_xyz += 0.02;
                ROS_INFO("Increased linear step. current: %f",increment_xyz);
                break;
            }
            case KEYBOARD_K : {
                increment_yaw -= 3.141592/12;
                ROS_INFO("Decreased yaw step. current: %f", increment_yaw);
                break;
            }
            case KEYBOARD_L : {
                increment_yaw += 3.141592/12;
                ROS_INFO("Increased yaw step. current: %f", increment_yaw);
                break;
            }
            case KEYBOARD_W: {move_pose(0) = increment_xyz; break;}
            case KEYBOARD_S: {move_pose(0) = -increment_xyz; break;}
            case KEYBOARD_A: {move_pose(1) = increment_xyz; break;}
            case KEYBOARD_D: {move_pose(1) = -increment_xyz; break;}
            case KEYBOARD_Z: {move_pose(2) = -increment_xyz; break;}
            case KEYBOARD_C: {move_pose(2) = increment_xyz; break;}
            case KEYBOARD_Q: {move_pose(3) = increment_xyz; break;}
            case KEYBOARD_E: {move_pose(3) = -increment_xyz; break;}
            }
        }
        bool flag_temp;
        flag_temp = move_mav(move_pose(0),move_pose(1),move_pose(2),move_pose(3));
    }
}


