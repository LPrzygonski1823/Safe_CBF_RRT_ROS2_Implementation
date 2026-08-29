#include "cbf_rrt_planner/collision_checker.hpp"

#include <cmath>
#include <algorithm>

namespace cbf_rrt_planner
{

    CollisionChecker::CollisionChecker(const OccupancyGridData & map, int occupied_threshold) : map_(map), occupied_threshold_(occupied_threshold) {
    }

    bool CollisionChecker::isOccupiedValue(int8_t value) const {
        if (value < 0) {return true;}   // nunknown area treated as an obstacle
        return value >= occupied_threshold_;
    }

    bool CollisionChecker::isPointFree(double x, double y) const
    {
        int col = static_cast<int>(std::floor((x - map_.origin_x) / map_.resolution));
        int row = static_cast<int>(std::floor((y - map_.origin_y) / map_.resolution));

        if (row < 0 || row >= map_.height || col < 0 || col >= map_.width) {
            return false;   // out of map treated as collision
        }
        return !isOccupiedValue(map_.at(row, col));
    }

    bool CollisionChecker::isEdgeFree(double x1, double y1, double x2, double y2) const
    {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double length = std::sqrt(dx * dx + dy * dy);

        if (length < 1e-9) {
            return isPointFree(x1, y1);
        }

        // half-a-cell sampling - cannot "jump over" thin obstacle no matter the angle
        double step = map_.resolution * 0.5;
        int num_samples = std::max(1, static_cast<int>(std::ceil(length / step)));

        for (int i = 0; i <= num_samples; ++i) {
            double t = static_cast<double>(i) / num_samples;
            double x = x1 + t * dx;
            double y = y1 + t * dy;
            if (!isPointFree(x, y)) {
            return false;
            }
        }
        return true;
    }

}