#pragma once
#include "cbf_rrt_planner/occupancy_grid_data.hpp"

namespace cbf_rrt_planner
{

    // configuration-space map for a disc-shaped robot: every cell whose centre lies within
    // `radius` [m] of an occupied (or unknown) cell becomes occupied (100). unknown cells keep -1
    OccupancyGridData inflateObstacles(const OccupancyGridData & map, double radius, int occupied_threshold);

}
