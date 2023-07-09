#include "goal_point_2D.hpp"

namespace GoalPoint2DLib {

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

}
