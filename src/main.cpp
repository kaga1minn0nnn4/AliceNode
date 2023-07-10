/**
 * @file main.cpp
 * @author daito tatesawa
 * @brief main node
 */

#include "alice.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mainnode");
    ros::NodeHandle nh("~");

    AliceLib::Alice alice{nh, 0.1};

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
