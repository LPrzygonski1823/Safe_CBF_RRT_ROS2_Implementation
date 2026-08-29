#include "cbf_rrt_planner/spatial_grid.hpp"

#include <cmath>
#include <limits>

namespace cbf_rrt_planner
{

    SpatialGrid::SpatialGrid(double cell_size) : cell_size_(cell_size > 1e-6 ? cell_size : 1.0){
    }

    void SpatialGrid::clear() {
        buckets_.clear();
    }

    int SpatialGrid::cellCoord(double value) const {
        return static_cast<int>(std::floor(value / cell_size_));
    }

    long long SpatialGrid::cellKey(int cx, int cy) const {
        // 2 32-bit cell coords to 1 64-bit
        return (static_cast<long long>(cx) << 32) ^ static_cast<unsigned int>(cy);
    }

    void SpatialGrid::insert(int index, double x, double y) {
        buckets_[cellKey(cellCoord(x), cellCoord(y))].push_back({index, x, y});
    }

    std::vector<int> SpatialGrid::queryRadius(double x, double y, double radius) const {
        std::vector<int> result;
        int cx0 = cellCoord(x - radius);
        int cx1 = cellCoord(x + radius);
        int cy0 = cellCoord(y - radius);
        int cy1 = cellCoord(y + radius);
        double radius_sq = radius * radius;

        for (int cx = cx0; cx <= cx1; ++cx) {
            for (int cy = cy0; cy <= cy1; ++cy) {
                auto it = buckets_.find(cellKey(cx, cy));
                if (it == buckets_.end()) {continue;}
                for (const auto & e : it->second) {
                    double dx = e.x - x;
                    double dy = e.y - y;
                    if (dx * dx + dy * dy <= radius_sq) {
                        result.push_back(e.index);
                    }
                }
            }
        }
        return result;
    }

    int SpatialGrid::queryNearest(double x, double y) const
    {
        int center_cx = cellCoord(x);
        int center_cy = cellCoord(y);

        int best_index = -1;
        double best_dist_sq = std::numeric_limits<double>::infinity();

        // search through rings of cells with increasing (Chebyshev) radius. Once
        // we find the first candidate, we search at least one additional
        // ring to rule out a closer point "around the corner" in an adjacent
        // cell, where the geometric distance might be smaller than the distance
        // to the candidate found during the earlier, "simpler" search.
        for (int ring = 0; ring < 1000000; ++ring) {
            for (int cx = center_cx - ring; cx <= center_cx + ring; ++cx) {
                for (int cy = center_cy - ring; cy <= center_cy + ring; ++cy) {
                    bool on_ring_edge = (cx == center_cx - ring) || (cx == center_cx + ring) ||
                    (cy == center_cy - ring) || (cy == center_cy + ring);
                    if (ring > 0 && !on_ring_edge) {continue;}

                    auto it = buckets_.find(cellKey(cx, cy));
                    if (it == buckets_.end()) {continue;}
                        for (const auto & e : it->second) {
                        double dx = e.x - x;
                        double dy = e.y - y;
                        double d_sq = dx * dx + dy * dy;
                        if (d_sq < best_dist_sq) {
                            best_dist_sq = d_sq;
                            best_index = e.index;
                        }
                    }
                }
            }

            if (best_index != -1) {
                double min_possible_dist_next_ring = static_cast<double>(ring) * cell_size_;
                if (min_possible_dist_next_ring * min_possible_dist_next_ring > best_dist_sq) {
                    break;
                }
            }
        }

        return best_index;
    }

}