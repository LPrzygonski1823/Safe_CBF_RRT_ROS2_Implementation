#pragma once
#include <limits>
#include <utility>
#include <vector>

#include "cbf_rrt_planner/occupancy_grid_data.hpp"

namespace cbf_rrt_planner
{

    struct ClearanceParams
    {
        double footprint_length = 0.33; // along the heading [m]
        double footprint_width = 0.28;  // across the heading [m]
        double search_radius = 2.0;     // obstacles farther than this are not looked for, clearances are capped at it [m]
        double sample_step = 0.05;      // spacing of the evaluated points along the path [m]
        int occupied_threshold = 65;
    };

    struct PathClearance
    {
        double min_clearance = std::numeric_limits<double>::quiet_NaN();  // robot centre to the nearest obstacle [m]
        double mean_clearance = std::numeric_limits<double>::quiet_NaN(); // length-weighted mean of the above [m]
        // rectangle footprint centred on the path and aligned with its tangent, < 0 = overlap [m]
        double min_footprint_clearance = std::numeric_limits<double>::quiet_NaN();
    };

    // geometric distance between a path and the obstacles of 'map', independent of h. pass the map
    // WITHOUT inflation: the result is then the real gap between the robot and the obstacles.
    // unknown cells count as obstacles, each occupied cell is its full square
    PathClearance computePathClearance(
        const OccupancyGridData & map,
        const std::vector<std::pair<double, double>> & path,
        const ClearanceParams & params);

}
