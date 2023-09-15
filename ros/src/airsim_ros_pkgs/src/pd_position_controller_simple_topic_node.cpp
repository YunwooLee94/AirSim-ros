//
// Created by larr-planning on 23. 9. 15.
//
#include "pd_position_controller_simple_topic.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pd_position_controller_simple_topic_node");
    PDController controller;
    controller.run();
}