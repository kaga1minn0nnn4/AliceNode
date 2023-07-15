/**
 * @file goal_point_2D.hpp
 * @brief publish to move_base
 */

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace GoalPoint2DLib{
    class GoalPoint2D {
     public:
        GoalPoint2D(double x, double y, double angle);

        operator geometry_msgs::PoseStamped();

        void Show() const;

     private:
        geometry_msgs::PoseStamped goal_point_;

    };
}
