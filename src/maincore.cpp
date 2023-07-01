/**
 * @file maincore.cpp
 * @author your name (you@domain.com)
 * @brief main core
 */

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <iostream>

enum class RobotStatus {
    kStart,
    kMoveToPoint,
    kRotateInPlace,
    kEndSearch
};

bool searching_is_finished = false;
bool navigation_is_finished = true;

void joystick_callback(const sensor_msgs::Joy::ConstPtr& joy_msg) {

}

void yolo_result_callback(const std_msgs::String::ConstPtr& result_msg) {
    searching_is_finished = false;
}

void movebase_status_callback(const actionlib_msgs::GoalStatusArray::ConstPtr& status) {
    navigation_is_finished = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "main_core");
    ros::NodeHandle nh("~");

    ros::Publisher rover_pub = nh.advertise<geometry_msgs::Twist>("/rover_twist", 10);
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

    ros::Subscriber joystick_sub = nh.subscribe("/joy", 10, joystick_callback);
    ros::Subscriber yolo_result_sub = nh.subscribe("/result", 10, yolo_result_callback);
    ros::Subscriber movebase_status_sub = nh.subscribe("/move_base/status", 10, movebase_status_callback);

    ros::Rate r(10);

    RobotStatus rstate = RobotStatus::kStart;

    while (ros::ok()) {
        switch (rstate) {
          case RobotStatus::kStart: {
            searching_is_finished = false;
            navigation_is_finished = false;
            rstate = RobotStatus::kMoveToPoint;
            break;
          }
          case RobotStatus::kMoveToPoint: {
            if (searching_is_finished){
                rstate = RobotStatus::kEndSearch;
                break;
            }
            if (navigation_is_finished) {
                rstate = RobotStatus::kRotateInPlace;
            } else {
                // goal_pub.Publish(...)
            }
            break;
          }
          case RobotStatus::kRotateInPlace: {
            if (searching_is_finished){
                rstate = RobotStatus::kEndSearch;
                break;
            }
            if (navigation_is_finished) {
                rstate = RobotStatus::kMoveToPoint;
            } else {
                // goal_pub.Publish(...)
            }
            break;
          }
          case RobotStatus::kEndSearch: {
            break;
          }
          default: {
            break;
          }
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
