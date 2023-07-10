#include "map_processing.hpp"

namespace MapProcessingLib {

    MapProcessing::MapProcessing(cv::Mat& img, uint16_t ln, uint16_t lm) {
        std::vector<cv::Point> target_points = CalcuLatticePoints(img, ln, lm);
        goal_points_ = ConvertCoordinate(target_points);
    }

    std::vector<cv::Point> MapProcessing::CalcuLatticePoints(cv::Mat& image, uint16_t lattice_n, uint16_t lattice_m) {
        uint16_t w_ = image.size().width;
        uint16_t h_ = image.size().height;

        cv::Rect roi(cv::Point(0, h_/2), cv::Size(w_, h_/2));
        cv::Mat img_clip = image(roi);

        uint16_t w = img_clip.size().width;
        uint16_t h = img_clip.size().height;

        std::vector<cv::Point> lattice_points;

        cv::Mat3b img_temp = img_clip;

        for (int i = 0; i < (lattice_n - 1); i++) {
            for (int j = 0; j < (lattice_m - 1); j++) {
                cv::Point p;
                p.x = static_cast<int>(static_cast<double>(w) / static_cast<double>(lattice_n) * (i + 1));
                p.y = static_cast<int>(static_cast<double>(h) / static_cast<double>(lattice_m) * (j + 1));

                if (static_cast<uint16_t>(img_temp(p)[0]) != 255) {
                    lattice_points.push_back(p);
                    ROS_INFO("x: %d, y: %d", lattice_points.back().x, lattice_points.back().y);
                    cv::circle(img_clip, p, 5, 255, -1, cv::LINE_AA);
                }
            }
        }
        cv::imwrite("/home/tenshi/catkin_ws/src/AliceNode/map/map_lattice.png", img_clip);

        map_width_ = w * kDistancePerCells;
        map_height_ = h * kDistancePerCells;

        robot_ox_ = 1.0;
        robot_oy_ = map_height_ / 2.0;

        return lattice_points;
    }

    std::queue<GoalPoint2DLib::GoalPoint2D> MapProcessing::ConvertCoordinate(const std::vector<cv::Point> points) {
        std::queue<GoalPoint2DLib::GoalPoint2D> converted_points;

        auto points_shuffle = points;

        std::random_shuffle(points_shuffle.begin(), points_shuffle.end());

        for (auto p : points_shuffle) {
            double x = static_cast<double>(p.x) * kDistancePerCells;
            double y = static_cast<double>(p.y) * kDistancePerCells;
            converted_points.push(GoalPoint2DLib::GoalPoint2D(map_width_ - robot_ox_ - x, y - robot_oy_, 0.0));

            converted_points.back().Show();
        }

        return converted_points;
    }

}

