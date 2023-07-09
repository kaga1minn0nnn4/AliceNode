#include "map_processing.hpp"

namespace MapProcessingLib {

    MapProcessing::MapProcessing(cv::Mat& img, uint16_t ln, uint16_t lm) {
        std::vector<cv::Point> target_points = CalcuLatticePoints(img, ln, lm);
        goal_points_ = ConvertCoordinate(target_points);
    }

    std::vector<cv::Point> MapProcessing::CalcuLatticePoints(cv::Mat& image, uint16_t lattice_n, uint16_t lattice_m) {
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

    std::queue<GoalPoint2DLib::GoalPoint2D> MapProcessing::ConvertCoordinate(const std::vector<cv::Point> points) {
        std::queue<GoalPoint2DLib::GoalPoint2D> converted_points;

        std::printf("%d, %d\n", points[3].x, points[3].y);

        for (int i = 0; i < points.size(); i++) {
            double x = static_cast<double>(points[i].x) * kDistancePerCells;
            double y = static_cast<double>(points[i].y) * kDistancePerCells;
            converted_points.push(GoalPoint2DLib::GoalPoint2D(1.0, 1.0, 1.0));

            converted_points.back().Show();
        }

        return converted_points;
    }
    
}

