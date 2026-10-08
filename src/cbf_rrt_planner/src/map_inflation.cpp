#include "cbf_rrt_planner/map_inflation.hpp"

#include <cmath>
#include <utility>
#include <vector>

namespace cbf_rrt_planner
{

    OccupancyGridData inflateObstacles(const OccupancyGridData & map, double radius, int occupied_threshold)
    {
        OccupancyGridData inflated = map;
        if (radius <= 0.0 || map.resolution <= 0.0) {return inflated;}

        // disc of cell offsets, measured between cell centres
        int r_cells = static_cast<int>(std::floor(radius / map.resolution));
        double r_sq = (radius / map.resolution) * (radius / map.resolution);
        std::vector<std::pair<int, int>> disc;
        for (int dr = -r_cells; dr <= r_cells; ++dr) {
            for (int dc = -r_cells; dc <= r_cells; ++dc) {
                if (dr * dr + dc * dc <= r_sq) {disc.emplace_back(dr, dc);}
            }
        }

        auto is_occupied = [&](int8_t v) {return v < 0 || v >= occupied_threshold;};

        for (int row = 0; row < map.height; ++row) {
            for (int col = 0; col < map.width; ++col) {
                if (!is_occupied(map.at(row, col))) {continue;}
                for (const auto & [dr, dc] : disc) {
                    int r = row + dr;
                    int c = col + dc;
                    if (r < 0 || r >= map.height || c < 0 || c >= map.width) {continue;}
                    int8_t & cell = inflated.data[static_cast<size_t>(r) * map.width + c];
                    if (!is_occupied(cell)) {cell = 100;}
                }
            }
        }
        return inflated;
    }

}
