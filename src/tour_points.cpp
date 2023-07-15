#include "tour_points.hpp"

namespace TourPointsLib {

    void TourPoints::Register(GoalPoint2DLib::GoalPoint2D point) {
        points_.push_back(point);
    }

    const GoalPoint2DLib::GoalPoint2D& TourPoints::RandomAccess() const {
        std::uniform_int_distribution<int> dist(0, points_.size());
        uint8_t index = dist(rd_);
        while (index == last_access_index_) {
            index = dist(rd_);
        }
        return points_[index];
    }

}
