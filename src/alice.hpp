/**
 * @file alice.hpp
 * @brief main process
 */

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
#include "FangFSM/fsm.hpp"
#include "tour_points.hpp"

enum class RobotStatus {
    kStart,
    kMoveToPoint,
    kWaitMoveBase,
    kEndOfSearch
};

namespace AliceLib {
    class Alice {
     public:
        Alice(ros::NodeHandle& nh, double t) :
          nh_{nh},
          rstate{RobotStatus::kStart},
          searching_is_finished_{false},
          navigation_is_finished_{true},
          mapdata_is_read_{false},
          pub_initial_{true},
          fsm_(RobotStatus::kStart, RobotStatus::kEndOfSearch) {

            rover_pub_ = nh_.advertise<geometry_msgs::Twist>("/rover_twist", 10);
            goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

            yolo_result_sub_ = nh_.subscribe("/result", 10, &Alice::YOLOv7ResultCallback, this);
            movebase_status_sub_ = nh_.subscribe("/move_base/status", 10, &Alice::MovebaseStatusCallback, this);
            mapdata_sub_ = nh_.subscribe("/premaked_map", 10, &Alice::MapdataCallback, this);

            mainloop_timer_ = nh_.createTimer(ros::Duration(t), &Alice::Run, this);

            fsm_.RegisterStateFct(RobotStatus::kStart, std::bind(&Alice::StartStateTask, this));
            fsm_.RegisterTransitionFct(RobotStatus::kStart, std::bind(&Alice::TransToStart, this));

            fsm_.RegisterStateFct(RobotStatus::kMoveToPoint, std::bind(&Alice::MoveToPointStateTask, this));
            fsm_.RegisterTransitionFct(RobotStatus::kMoveToPoint, std::bind(&Alice::TransToMove, this));

            fsm_.RegisterStateFct(RobotStatus::kWaitMoveBase, std::bind(&Alice::WaitMovebaseTask, this));
            fsm_.RegisterTransitionFct(RobotStatus::kWaitMoveBase, std::bind(&Alice::TransToWait, this));

            fsm_.RegisterStateFct(RobotStatus::kEndOfSearch, std::bind(&Alice::EndOfSearchTask, this));
            fsm_.RegisterTransitionFct(RobotStatus::kEndOfSearch, std::bind(&Alice::TransToEnd, this));
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

        bool pub_initial_;

        uint8_t status_id_ = 0;

        TourPointsLib::TourPoints tp_;

        cv::Mat mapimage_;

        std::mutex mtx_;

        FangFSMLibrary::FangFSM<RobotStatus> fsm_;

        void YOLOv7ResultCallback(const std_msgs::String::ConstPtr& result_msg);
        void MovebaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status);
        void MapdataCallback(const nav_msgs::OccupancyGrid::ConstPtr& mapdata);

        void ClipMapdata(const cv::Mat& mapimg, cv::Mat& cliped_mapimg);

        void Run(const ros::TimerEvent& e);

        void StartStateTask();
        void MoveToPointStateTask();
        void WaitMovebaseTask();
        void EndOfSearchTask();

        RobotStatus TransToStart();
        RobotStatus TransToMove();
        RobotStatus TransToWait();
        RobotStatus TransToEnd();
    };
}
