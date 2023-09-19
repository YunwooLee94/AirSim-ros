#include "keyboard_operator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_drone");
    keyboard_operator::KeyboardOperator keyboard_drone;
    signal(SIGINT,keyboard_operator::quit);
    keyboard_drone.keyLoop();
    return 0;
}