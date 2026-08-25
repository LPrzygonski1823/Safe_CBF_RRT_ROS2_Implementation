#include "cbf_rrt_planner/obstacle_segmentation.hpp"
 
#include <queue>
#include <utility>
 
namespace cbf_rrt_planner
{
    ObstacleSegmentation::ObstacleSegmentation(int occupied_threshold, bool treat_unknown_as_occupied) : occupied_threshold_(occupied_threshold), treat_unknown_as_occupied_(treat_unknown_as_occupied) {
    }

    bool ObstacleSegmentation::isOccupied(int8_t value) const
    {
        if (value < 0) { // (-1) - unknown area
            return treat_unknown_as_occupied_;
        }
        return value >= occupied_threshold_;
    }

    SegmentationResult ObstacleSegmentation::segment(const  OccupancyGridData & grid) const
    {
        SegmentationResult result;
        result.width = grid.width;
        result.height = grid.height;
        result.resolution = grid.resolution;
        result.origin_x = grid.origin_x;
        result.origin_y = grid.origin_y;
        result.cell_type = GridI(grid.height, grid.width, static_cast<int>(CellType::Free));
        result.obstacle_id = GridI(grid.height, grid.width, -1); // no obstacle id (obst. ids start from 0)

        // binary occupancy mask 0=free 1=occupied
        std::vector<char> occupied(static_cast<size_t>(grid.width) * grid.height, 0);
        for (int r = 0; r < grid.height; ++r) {
            for (int c = 0; c < grid.width; ++c) {
                occupied[static_cast<size_t>(r) * grid.width + c] = isOccupied(grid.at(r, c)) ? 1 : 0;
            }
        }

        // extraction of connected points (4 neighbours)
        std::vector<int> component_id(occupied.size(), -1); // not classified yet/not an obstacle
        const int dr[4] = {-1, 1, 0, 0};
        const int dc[4] = {0, 0, -1, 1};
        
        for (int r = 0; r < grid.height; ++r) {
            for (int c = 0; c < grid.width; ++c) {
                size_t idx = static_cast<size_t>(r) * grid.width + c;
                if (!occupied[idx] || component_id[idx] != -1) {continue;} // not free or already classified as obstacle -> skip
            
                // new obstacle (not assigned to any obstacle id)
                int comp_idx = static_cast<int>(result.obstacles.size()); // new id given
                Obstacle obstacle; // new, empty obstacle
            
                std::queue<std::pair<int, int>> q;
                q.push({r, c});
                component_id[idx] = comp_idx;
            
                // "flood fill" - checking neighbours
                while (!q.empty()) {
                    auto [cr, cc] = q.front();
                    q.pop();
                    obstacle.cells.push_back({cr, cc});
            
                    for (int k = 0; k < 4; ++k) {
                        int nr = cr + dr[k];
                        int nc = cc + dc[k];
                        if (nr < 0 || nr >= grid.height || nc < 0 || nc >= grid.width) {continue;}
                        size_t nidx = static_cast<size_t>(nr) * grid.width + nc;
                        // if neighbour is occupied but not assigned to any obstacle...
                        if (occupied[nidx] && component_id[nidx] == -1) {
                            component_id[nidx] = comp_idx; // add it to current obstacle
                            q.push({nr, nc}); // queue it to check his neighbours later
                        }
                    }
                }
        
        // centroid + adding ready obstacle to result.obstacles
        double sum_x = 0.0, sum_y = 0.0;
        for (auto & cell : obstacle.cells) {
            double wx, wy;
            result.gridToWorld(cell.first, cell.second, wx, wy);
            sum_x += wx;
            sum_y += wy;
        }
        obstacle.centroid_x = sum_x / static_cast<double>(obstacle.cells.size());
        obstacle.centroid_y = sum_y / static_cast<double>(obstacle.cells.size());
 
        result.obstacles.push_back(std::move(obstacle));
            }
        }

        // classify each cell as Free, ObstacleInterior, ObstacleBoundary or DomainBoundary
        // for obstacle boundary cells, also store the obstacle id
        for (int r = 0; r < grid.height; ++r) {
            for (int c = 0; c < grid.width; ++c) {
            size_t idx = static_cast<size_t>(r) * grid.width + c;
        
            if (occupied[idx]) {
                // checking if any of 4 neighbours is free -> it's obstacle's edge
                bool is_boundary = false;
                for (int k = 0; k < 4; ++k) {
                int nr = r + dr[k];
                int nc = c + dc[k];
                if (nr < 0 || nr >= grid.height || nc < 0 || nc >= grid.width) {continue;}
                size_t nidx = static_cast<size_t>(nr) * grid.width + nc;
                if (!occupied[nidx]) {
                    is_boundary = true;
                    break;
                }
                }
                if (is_boundary) {
                result.cell_type(r, c) = static_cast<int>(CellType::ObstacleBoundary);
                result.obstacle_id(r, c) = component_id[idx];
                } else {
                result.cell_type(r, c) = static_cast<int>(CellType::ObstacleInterior);
                }
            } else {
                // free cell - checking if it's on the map's edge (dOmega)
                bool on_domain_edge = (r == 0 || r == grid.height - 1 || c == 0 || c == grid.width - 1);
                result.cell_type(r, c) = on_domain_edge ?
                static_cast<int>(CellType::DomainBoundary) :
                static_cast<int>(CellType::Free);
            }
            }
        }
        return result;
    }
}