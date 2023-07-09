/**
 * @file main.cpp
 * @author daito tatesawa
 * @brief main node
 */

#include "alice.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mainnode");
    ros::NodeHandle nh("~");

    ros::Rate r(10);

    AliceLib::Alice alice{nh};

    while (ros::ok()) {
        alice.Run();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
