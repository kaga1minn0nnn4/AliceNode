#include "alice.hpp"

namespace AliceLib {

    void Alice::Run(const ros::TimerEvent& e) {
        std::lock_guard<std::mutex> lock(mtx_);
        fsm_.Run();
    }

    void Alice::YOLOv7ResultCallback(const std_msgs::String::ConstPtr& result_msg) {
        // read result_msg process ...
        {
            std::lock_guard<std::mutex> lock(mtx_);
            ROS_INFO(result_msg->data.c_str());
            if (result_msg->data.c_str() == std::string("detect")) {
                searching_is_finished_ = true;
                ROS_INFO("Detect !");
            }
        }
    }

    void Alice::MovebaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status) {
        uint8_t status_id = 0;
        if (!status->status_list.empty()) {
            actionlib_msgs::GoalStatus goal_status = status->status_list[0];
            status_id = goal_status.status;
        }

        status_id_ = status_id;
        std::printf("sid : %d\n", status_id);

        {
            std::lock_guard<std::mutex> lock(mtx_);
            switch (status_id) {
            case 1: {
                navigation_is_finished_ = false;
                break;
            }
             case 2: {
                navigation_is_finished_ = false;
                break;
             }
            case 3: {
                navigation_is_finished_ = true;
                break;
            }
            default:
                break;
            }
        }
    }

    void Alice::StartStateTask() {
        ROS_INFO("kStart");
    }

    RobotStatus Alice::TransFromStart() {
        RobotStatus next = RobotStatus::kMoveToPoint;

        searching_is_finished_ = false;
        navigation_is_finished_ = false;

        tp_.Register(GoalPoint2DLib::GoalPoint2D(0.44, -0.47, 1.0));
        tp_.Register(GoalPoint2DLib::GoalPoint2D(0.44, 0.43, 1.0));
        tp_.Register(GoalPoint2DLib::GoalPoint2D(1.88, -0.47, 1.0));
        tp_.Register(GoalPoint2DLib::GoalPoint2D(1.88, 0.43, 1.0));
        tp_.Register(GoalPoint2DLib::GoalPoint2D(3.32, -0.47, 1.0));
        tp_.Register(GoalPoint2DLib::GoalPoint2D(3.32, 0.43, 1.0));

        tp_.Show();

        return next;
    }

    void Alice::MoveToPointStateTask() {
        ROS_INFO("kMoveToPoint");
        if (pub_initial_) {
            geometry_msgs::PoseStamped p = tp_.RandomAccess();
            goal_pub_.publish(p);
            pub_initial_ = false;
        }
    }

    RobotStatus Alice::TransFromMove() {
        RobotStatus next = RobotStatus::kWaitMoveBase;

        if (searching_is_finished_) {
            next = RobotStatus::kEndOfSearch;
        } else if (status_id_ == 1) {
            next = RobotStatus::kWaitMoveBase;
        }

        return next;
    }

    void Alice::WaitMovebaseTask() {
        ROS_INFO("kWaitMovebase");
    }

    RobotStatus Alice::TransFromWait() {
        RobotStatus next = RobotStatus::kWaitMoveBase;

        if (searching_is_finished_) {
            next = RobotStatus::kEndOfSearch;
        } else if (status_id_ == 3) {
            next = RobotStatus::kMoveToPoint;
            pub_initial_ = true;
            navigation_is_finished_ = false;
        }

        return next;
    }

    void Alice::EndOfSearchTask() {
        ROS_INFO("kEndOfSearch");
    }

    RobotStatus Alice::TransFromEnd() {
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client("move_base");
        client.waitForServer();
        client.cancelAllGoals();

        return RobotStatus::kEndOfSearch;
    }

}
