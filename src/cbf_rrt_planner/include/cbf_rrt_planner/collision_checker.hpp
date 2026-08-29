#pragma once
#include "cbf_rrt_planner/occupancy_grid_data.hpp"

namespace cbf_rrt_planner
    {

    // geometric collision test independent from PSF/CBF
    // used directly by RrtStarPlanner and indiredctly by CBF (condition)
    class CollisionChecker
    {
        public:
            explicit CollisionChecker(const OccupancyGridData & map, int occupied_threshold = 65);

            bool isPointFree(double x, double y) const;
            bool isEdgeFree(double x1, double y1, double x2, double y2) const;

            // workspace frontiers [m] needed for RRT* sampling
            double minX() const {return map_.origin_x;}
            double maxX() const {return map_.origin_x + map_.width * map_.resolution;}
            double minY() const {return map_.origin_y;}
            double maxY() const {return map_.origin_y + map_.height * map_.resolution;}

        private:
            OccupancyGridData map_; // copy
            int occupied_threshold_;

            bool isOccupiedValue(int8_t value) const;
    };

}