/**
 * @file map_processing.hpp
 * @brief mapdata -> latticepoint
 */

#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "goal_point_2D.hpp"

namespace MapProcessingLib {

    class MapProcessing {
     public:
        MapProcessing(cv::Mat& img, uint16_t ln, uint16_t lm);

        const std::queue<GoalPoint2DLib::GoalPoint2D>& GetGoalPoints() const {
            return goal_points_;
        }

     private:
        static constexpr double kDistancePerCells = 0.05;

        std::queue<GoalPoint2DLib::GoalPoint2D> goal_points_;

        std::vector<cv::Point> CalcuLatticePoints(cv::Mat& image, uint16_t lattice_n, uint16_t lattice_m);

        std::queue<GoalPoint2DLib::GoalPoint2D> ConvertCoordinate(const std::vector<cv::Point> points);
    };

}

