#pragma once
#include <utility>
#include <vector>
 
#include "cbf_rrt_planner/grid2d.hpp"
#include "cbf_rrt_planner/occupancy_grid_data.hpp"

// classification of each grid cell (needed by PSF solver)
namespace cbf_rrt_planner
{
    enum class CellType : int
    {
        Free = 0,
        ObstacleInterior = 1,   // inside obstacle, not used by solver
        ObstacleBoundary = 2,   // edge of an obstacle (d0_i)
        DomainBoundary = 3      // map's outer edge (dOmega)
    };

    // single, found obstacle - its cells set and centroid (c_i)
    struct Obstacle
    {
        double centroid_x = 0.0;
        double centroid_y = 0.0;
        std::vector<std::pair<int, int>> cells; // all obstacle's cells
    };

    // segmentation output: full grid classification + obstacle list
    struct SegmentationResult
    {
        int width = 0;
        int height = 0;
        double resolution = 1.0;
        double origin_x = 0.0;
        double origin_y = 0.0;

        GridI cell_type; // CellType values, h*w
        GridI obstacle_id; // obstacle index, only for ObstacleBoundary cells, otherwise (-1)
        std::vector<Obstacle> obstacles;
        
        // grid index (row, col) to world coords (m) - center of a cell
        void gridToWorld(int row, int col, double & x, double & y) const
        {
            x = origin_x + (col + 0.5) * resolution;
            y = origin_y + (row + 0.5) * resolution;
        }
    };

    class ObstacleSegmentation
    {
        public:
            // occupied_treshold: for 0-100 occupancy grid value
            explicit ObstacleSegmentation(int occupied_threshold = 65, bool treat_unknown_as_occupied = true);
            
            // obstacles segmented (connected components) and centroids found -> classifies every cell for PDE
            SegmentationResult segment(const OccupancyGridData & grid) const;
            
        private:
            int occupied_threshold_;
            bool treat_unknown_as_occupied_;

            bool isOccupied(int8_t value) const;
    };

}