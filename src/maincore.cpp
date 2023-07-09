/**
 * @file maincore.cpp
 * @author daito tatesawa
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
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <iostream>
#include <vector>
#include <queue>

#include <opencv2/opencv.hpp>

enum class RobotStatus {
    kStart,
    kMoveToPoint,
    kRotateInPlace,
    kEndSearch
};

static constexpr uint16_t lattice_n = 4;
static constexpr uint16_t lattice_m = 3;
static constexpr double distance_per_cells = 0.05;

bool searching_is_finished = false;
bool navigation_is_finished = true;
bool mapdata_is_read = false;

std::vector<cv::Point> target_points;
cv::Mat mapimage;

class GoalPoint2D {
 public:
    GoalPoint2D(double x, double y, double angle);

    operator geometry_msgs::PoseStamped();

    void Show() const;

 private:
    geometry_msgs::PoseStamped goal_point_;

};

std::queue<GoalPoint2D> goal_points;

GoalPoint2D::GoalPoint2D(double x, double y, double angle) {
    goal_point_.pose.position.x = x;
    goal_point_.pose.position.y = y;
    goal_point_.pose.position.z = 0.0;
    goal_point_.pose.orientation.w = angle;

    goal_point_.header.stamp = ros::Time::now();
    goal_point_.header.frame_id = "map";
}

GoalPoint2D::operator geometry_msgs::PoseStamped() {
    return goal_point_;
}

void GoalPoint2D::Show() const {
    std::printf("x: %5.2f, y: %5.2f, angle: %5.2f\n",
        goal_point_.pose.position.x,
        goal_point_.pose.position.y,
        goal_point_.pose.orientation.w);
}

std::vector<cv::Point> calcu_lattice_point(cv::Mat& image) {
    uint16_t w = image.size().width;
    uint16_t h = image.size().height;

    std::vector<cv::Point> lattice_points;

    cv::Mat3b img_temp = image;

    for (int i = 0; i < (lattice_n - 1); i++) {
        for (int j = 0; j < (lattice_m - 1); j++) {
            cv::Point p;
            p.x = static_cast<int>(static_cast<double>(w) / static_cast<double>(lattice_n) * (i + 1));
            p.y = static_cast<int>(static_cast<double>(h) / static_cast<double>(lattice_m) * (j + 1));

            if (static_cast<uint16_t>(img_temp(p)[0]) != 255) {
                lattice_points.push_back(p);
                ROS_INFO("x: %d, y: %d", lattice_points.back().x, lattice_points.back().y);
            }
        }
    }

    return lattice_points;
}

std::queue<GoalPoint2D> convert_coordinate(const std::vector<cv::Point> points) {
    std::queue<GoalPoint2D> converted_points;

    std::printf("%d, %d\n", points[3].x, points[3].y);

    for (int i = 0; i < points.size(); i++) {
        double x = static_cast<double>(points[i].x) * distance_per_cells;
        double y = static_cast<double>(points[i].y) * distance_per_cells;
        converted_points.push(GoalPoint2D(1.0, 1.0, 1.0));

        converted_points.back().Show();
    }

    return converted_points;
}

void joystick_callback(const sensor_msgs::Joy::ConstPtr& joy_msg) {

}

void yolo_result_callback(const std_msgs::String::ConstPtr& result_msg) {
    searching_is_finished = false;
}

void movebase_status_callback(const actionlib_msgs::GoalStatusArray::ConstPtr& status) {
    uint8_t status_id = 0;
    if (!status->status_list.empty()) {
        actionlib_msgs::GoalStatus goal_status = status->status_list[0];
        status_id = goal_status.status;
    }

    switch (status_id) {
      case 1: {
        navigation_is_finished = false;
        break;
      }
      // case 0:
      case 3: {
        navigation_is_finished = true;
        break;
      }
      default:
        break;
    }

    std::printf("id: %d\n", status_id);
}

void clip_mapdata(const cv::Mat& mapimg, cv::Mat& cliped_mapimg) {
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

void mapdata_callback(const nav_msgs::OccupancyGrid::ConstPtr& mapdata) {
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

    clip_mapdata(dst, mapimage);

    cv::imwrite("/home/tenshi/catkin_ws/src/aris_maincore/map/map.png", img);
    cv::imwrite("/home/tenshi/catkin_ws/src/aris_maincore/map/map_clip.png", mapimage);

    mapdata_is_read = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "main_core");
    ros::NodeHandle nh("~");

    ros::Publisher rover_pub = nh.advertise<geometry_msgs::Twist>("/rover_twist", 10);
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

    ros::Subscriber joystick_sub = nh.subscribe("/joy", 10, joystick_callback);
    ros::Subscriber yolo_result_sub = nh.subscribe("/result", 10, yolo_result_callback);
    ros::Subscriber movebase_status_sub = nh.subscribe("/move_base/status", 10, movebase_status_callback);
    ros::Subscriber mapdata_sub = nh.subscribe("/premaked_map", 10, mapdata_callback);

    ros::Rate r(10);

    RobotStatus rstate = RobotStatus::kStart;

    while (ros::ok()) {
        switch (rstate) {
          case RobotStatus::kStart: {
            ROS_INFO("kStart");
            if (mapdata_is_read) {
                searching_is_finished = false;
                navigation_is_finished = false;
                rstate = RobotStatus::kMoveToPoint;

                target_points = calcu_lattice_point(mapimage);

                goal_points = convert_coordinate(target_points);
            }
            break;
          }
          case RobotStatus::kMoveToPoint: {
            ROS_INFO("kMoveToPoint");
            if (searching_is_finished){
                rstate = RobotStatus::kEndSearch;
                break;
            }
            if (navigation_is_finished) {
                goal_points.pop();
                rstate = RobotStatus::kRotateInPlace;
            } else {
                if (!goal_points.empty()) {
                    geometry_msgs::PoseStamped p = goal_points.front();
                    goal_pub.publish(p);
                    rstate = RobotStatus::kEndSearch;
                } else {
                    rstate = RobotStatus::kEndSearch;
                }
            }
            break;
          }
          case RobotStatus::kRotateInPlace: {
            ROS_INFO("kRotateInPlace");
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
            ROS_INFO("kEndSearch");
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
