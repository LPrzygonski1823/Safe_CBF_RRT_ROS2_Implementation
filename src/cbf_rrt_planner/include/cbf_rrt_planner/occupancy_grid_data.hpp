#pragma once
#include <cstdint>
#include <vector>
#include <cstddef>


// ROS2 independent map representation - for testing
namespace cbf_rrt_planner
{
    struct OccupancyGridData
    {
        int width = 0;
        int height = 0;
        double resolution = 1.0; // (m/cell)
        double origin_x = 0.0; // world coorditate (0,0), map's edge
        double origin_y = 0.0;

        std::vector<int8_t> data; // -1, 0-100

        int8_t at(int row, int col) const
        {
            return data[static_cast<size_t>(row) * width + col];
        }
    };
}