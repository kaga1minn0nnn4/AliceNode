/**
 * @file map_processing.hpp
 * @brief mapdata -> latticepoint
 */

#pragma once

#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>

#include "goal_point_2D.hpp"
#include "tour_points.hpp"

namespace MapProcessingLib {

    class MapProcessing {
     public:
        MapProcessing(cv::Mat& img, uint16_t ln, uint16_t lm);

        void GetGoalPoints(TourPointsLib::TourPoints& points) const {
            for (auto p : goal_points_) {
                points.Register(p);
            }
        }

     private:
        static constexpr double kDistancePerCells = 0.03;

        double map_width_;
        double map_height_;

        double robot_ox_;
        double robot_oy_;

        std::vector<GoalPoint2DLib::GoalPoint2D> goal_points_;

        std::vector<cv::Point> CalcuLatticePoints(cv::Mat& image, uint16_t lattice_n, uint16_t lattice_m);

        std::vector<GoalPoint2DLib::GoalPoint2D> ConvertCoordinate(const std::vector<cv::Point> points);
    };

}

