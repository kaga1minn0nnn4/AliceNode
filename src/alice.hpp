#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <nav_msgs/OccupancyGrid.h>

#include <iostream>
#include <vector>
#include <queue>
#include <mutex>

#include "goal_point_2D.hpp"
#include "map_processing.hpp"

enum class RobotStatus {
    kStart,
    kMoveToPoint,
    kRotateInPlace,
    kEndSearch
};

namespace AliceLib {
    class Alice {
     public:
        Alice(ros::NodeHandle& nh, double t) :
          nh_{nh},
          rstate{RobotStatus::kStart},
          searching_is_finished_{false},
          navigation_is_finished_{false},
          mapdata_is_read_{false} {

            rover_pub_ = nh_.advertise<geometry_msgs::Twist>("/rover_twist", 10);
            goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

            yolo_result_sub_ = nh_.subscribe("/result", 10, &Alice::YOLOv7ResultCallback, this);
            movebase_status_sub_ = nh_.subscribe("/move_base/status", 10, &Alice::MovebaseStatusCallback, this);
            mapdata_sub_ = nh_.subscribe("/premaked_map", 10, &Alice::MapdataCallback, this);

            mainloop_timer_ = nh_.createTimer(ros::Duration(t), &Alice::Run, this);
        }

     private:
        static constexpr uint16_t kLatticeN = 4;
        static constexpr uint16_t kLatticeM = 3;

        ros::NodeHandle& nh_;

        RobotStatus rstate;

        ros::Publisher rover_pub_;
        ros::Publisher goal_pub_;

        ros::Subscriber joystick_sub_;
        ros::Subscriber yolo_result_sub_;
        ros::Subscriber movebase_status_sub_;
        ros::Subscriber mapdata_sub_;

        ros::Timer mainloop_timer_;

        bool searching_is_finished_;
        bool navigation_is_finished_;
        bool mapdata_is_read_;

        std::queue<GoalPoint2DLib::GoalPoint2D> goal_points_;

        cv::Mat mapimage_;

        std::mutex mtx_;

        void YOLOv7ResultCallback(const std_msgs::String::ConstPtr& result_msg);
        void MovebaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status);
        void MapdataCallback(const nav_msgs::OccupancyGrid::ConstPtr& mapdata);

        void ClipMapdata(const cv::Mat& mapimg, cv::Mat& cliped_mapimg);

        void Run(const ros::TimerEvent& e);
    };
}
