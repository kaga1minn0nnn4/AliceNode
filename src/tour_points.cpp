#include "tour_points.hpp"

namespace TourPointsLib {

    void TourPoints::Register(GoalPoint2DLib::GoalPoint2D point) {
        points_.push_back(point);
    }

    GoalPoint2DLib::GoalPoint2D& TourPoints::RandomAccess() {
        std::uniform_int_distribution<int> dist(0, points_.size() - 1);
        uint8_t index = dist(rd_);
        while (index == last_access_index_) {
            index = dist(rd_);
        }

        //std::printf("size %d, index %d\n", points_.size(), index);

        last_access_index_ = index;
        return points_[index];
    }

}
