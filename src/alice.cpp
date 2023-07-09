#include "alice.hpp"

namespace AliceLib {
    void Alice::Run() {
        switch (rstate) {
          case RobotStatus::kStart: {
            ROS_INFO("kStart");
            if (mapdata_is_read_) {
                searching_is_finished_ = false;
                navigation_is_finished_ = false;
                rstate = RobotStatus::kMoveToPoint;

                MapProcessingLib::MapProcessing mp(mapimage, kLatticeN, kLatticeM);
                goal_points = mp.GetGoalPoints();
            }
            break;
          }
          case RobotStatus::kMoveToPoint: {
            ROS_INFO("kMoveToPoint");
            if (searching_is_finished_){
                rstate = RobotStatus::kEndSearch;
                break;
            }
            if (navigation_is_finished_) {
                goal_points.pop();
                rstate = RobotStatus::kRotateInPlace;
            } else {
                if (!goal_points.empty()) {
                    geometry_msgs::PoseStamped p = goal_points.front();
                    goal_pub_.publish(p);
                    rstate = RobotStatus::kEndSearch;
                } else {
                    rstate = RobotStatus::kEndSearch;
                }
            }
            break;
          }
          case RobotStatus::kRotateInPlace: {
            ROS_INFO("kRotateInPlace");
            if (searching_is_finished_){
                rstate = RobotStatus::kEndSearch;
                break;
            }
            if (navigation_is_finished_) {
                rstate = RobotStatus::kMoveToPoint;
            } else {
                // goal_pub.Publish(...)
            }
            break;
            }
            case RobotStatus::kEndSearch: {
            ROS_INFO("kEndSearch");
            break;
          }
          default: {
            break;
          }
        }
    }

    void Alice::YOLOv7ResultCallback(const std_msgs::String::ConstPtr& result_msg) {
        searching_is_finished_ = false;
    }

    void Alice::MovebaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status) {
        uint8_t status_id = 0;
        if (!status->status_list.empty()) {
            actionlib_msgs::GoalStatus goal_status = status->status_list[0];
            status_id = goal_status.status;
        }

        switch (status_id) {
        case 1: {
            navigation_is_finished_ = false;
            break;
        }
        // case 0:
        case 3: {
            navigation_is_finished_ = true;
            break;
        }
        default:
            break;
        }

        std::printf("id: %d\n", status_id);
    }

    void Alice::ClipMapdata(const cv::Mat& mapimg, cv::Mat& cliped_mapimg) {
        cv::Mat dst;
        cv::adaptiveThreshold(mapimg, dst, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 51, 0);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(dst, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        std::vector<uint16_t> x_list, y_list;

        for (auto i : contours) {
            for (auto j : i) {
                x_list.push_back(j.x);
                y_list.push_back(j.y);
            }
        }

        uint16_t right = static_cast<int>(*std::max_element(x_list.begin(), x_list.end()));
        uint16_t left = static_cast<int>(*std::min_element(x_list.begin(), x_list.end()));
        uint16_t upper = static_cast<int>(*std::max_element(y_list.begin(), y_list.end()));
        uint16_t lower = static_cast<int>(*std::min_element(y_list.begin(), y_list.end()));

        cv::Rect roi(cv::Point(left, lower), cv::Size(right-left, upper-lower));
        cliped_mapimg = dst(roi);
    }

    void Alice::MapdataCallback(const nav_msgs::OccupancyGrid::ConstPtr& mapdata) {
        uint8_t img_rawdata[2048*2048];

        for (int i = 0; i < 2048*2048; i++) {
            if(mapdata->data[i] == -1) {
                img_rawdata[i] = 0;
            } else {
                img_rawdata[i] = mapdata->data[i];
            }
        }

        cv::Mat img(2048, 2048, CV_8UC1, img_rawdata);

        cv::Mat dst;
        cv::flip(img, dst, 0);

        ClipMapdata(dst, mapimage);

        cv::imwrite("/home/tenshi/catkin_ws/src/AliceNode/map/map.png", img);
        cv::imwrite("/home/tenshi/catkin_ws/src/AliceNode/map/map_clip.png", mapimage);

        mapdata_is_read_ = true;
    }

}