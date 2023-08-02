/**
 * @file tour_points.hpp
 * @brief access tour points
 */

#pragma once

#include <iostream>
#include <vector>
#include <random>

#include "goal_point_2D.hpp"

namespace TourPointsLib {
    class TourPoints {
     public:
        TourPoints() : rd_{std::random_device{}()} {}

        void Register(GoalPoint2DLib::GoalPoint2D point);
        GoalPoint2DLib::GoalPoint2D& RandomAccess();

        void Show() const;

     private:
        std::vector<GoalPoint2DLib::GoalPoint2D> points_;

        uint8_t last_access_index_ = 0;

        std::mt19937 rd_;

    };

}
