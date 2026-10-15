#include "cbf_rrt_planner/path_clearance.hpp"

#include <algorithm>
#include <cmath>

namespace cbf_rrt_planner
{

    namespace
    {
        struct PoseClearance
        {
            double point;
            double footprint;
        };

        // signed distance from a point (in the box frame) to a centred box, < 0 inside
        inline double boxDistance(double px, double py, double half_x, double half_y)
        {
            double qx = std::abs(px) - half_x;
            double qy = std::abs(py) - half_y;
            if (qx > 0.0 || qy > 0.0) {return std::hypot(std::max(qx, 0.0), std::max(qy, 0.0));}
            return std::max(qx, qy);
        }

        // brute force over the cells of a square window - a path has a few hundred samples, so
        // this costs milliseconds and needs no distance transform of the whole map
        PoseClearance poseClearance(
            const OccupancyGridData & map, double x, double y, double yaw, const ClearanceParams & p)
        {
            const double res = map.resolution;
            const double half_cell = 0.5 * res;
            const double half_l = 0.5 * p.footprint_length;
            const double half_w = 0.5 * p.footprint_width;
            const double reach = p.search_radius + std::hypot(half_l, half_w);
            const double cos_yaw = std::cos(yaw);
            const double sin_yaw = std::sin(yaw);

            // footprint corners relative to (x, y), in map axes
            double corner_x[4], corner_y[4];
            for (int k = 0; k < 4; ++k) {
                double lx = (k & 1) ? half_l : -half_l;
                double ly = (k & 2) ? half_w : -half_w;
                corner_x[k] = cos_yaw * lx - sin_yaw * ly;
                corner_y[k] = sin_yaw * lx + cos_yaw * ly;
            }

            const int col0 = static_cast<int>(std::floor((x - map.origin_x) / res));
            const int row0 = static_cast<int>(std::floor((y - map.origin_y) / res));
            const int span = static_cast<int>(std::ceil(reach / res)) + 1;

            double best_point = p.search_radius;
            double best_footprint = p.search_radius;

            for (int r = std::max(0, row0 - span); r <= std::min(map.height - 1, row0 + span); ++r) {
                for (int c = std::max(0, col0 - span); c <= std::min(map.width - 1, col0 + span); ++c) {
                    int8_t v = map.at(r, c);
                    if (v >= 0 && v < p.occupied_threshold) {continue;}

                    // cell centre relative to (x, y)
                    double dx = map.origin_x + (c + 0.5) * res - x;
                    double dy = map.origin_y + (r + 0.5) * res - y;

                    best_point = std::min(best_point, boxDistance(dx, dy, half_cell, half_cell));

                    // for two convex polygons the gap is attained at a vertex of one of them, and an
                    // overlap always puts a vertex of one inside the other (the cell is smaller than
                    // the footprint in both directions)
                    double sd = std::numeric_limits<double>::infinity();
                    for (int k = 0; k < 4; ++k) {
                        double cx = dx + ((k & 1) ? half_cell : -half_cell);
                        double cy = dy + ((k & 2) ? half_cell : -half_cell);
                        sd = std::min(sd, boxDistance(
                            cos_yaw * cx + sin_yaw * cy, -sin_yaw * cx + cos_yaw * cy, half_l, half_w));
                        sd = std::min(sd, boxDistance(corner_x[k] - dx, corner_y[k] - dy, half_cell, half_cell));
                    }
                    best_footprint = std::min(best_footprint, sd);
                }
            }
            return {best_point, best_footprint};
        }
    }

    PathClearance computePathClearance(
        const OccupancyGridData & map,
        const std::vector<std::pair<double, double>> & path,
        const ClearanceParams & params)
    {
        PathClearance out;
        if (path.empty() || map.resolution <= 0.0) {return out;}

        if (path.size() == 1) {
            auto pc = poseClearance(map, path[0].first, path[0].second, 0.0, params);
            out.min_clearance = out.mean_clearance = pc.point;
            out.min_footprint_clearance = pc.footprint;
            return out;
        }

        const double step = std::max(params.sample_step, 1e-3);
        double min_point = std::numeric_limits<double>::infinity();
        double min_footprint = std::numeric_limits<double>::infinity();
        double weighted_sum = 0.0;
        double total_length = 0.0;
        double yaw = 0.0;

        for (size_t i = 0; i + 1 < path.size(); ++i) {
            const double x1 = path[i].first, y1 = path[i].second;
            const double x2 = path[i + 1].first, y2 = path[i + 1].second;
            const double len = std::hypot(x2 - x1, y2 - y1);
            // a degenerate segment carries no heading - keep the previous one
            if (len > 1e-9) {yaw = std::atan2(y2 - y1, x2 - x1);}

            // both ends are sampled with this segment's heading, so every vertex is checked with
            // the heading before and after the turn
            const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
            double seg_sum = 0.0;
            for (int s = 0; s <= n; ++s) {
                const double t = static_cast<double>(s) / n;
                auto pc = poseClearance(map, x1 + t * (x2 - x1), y1 + t * (y2 - y1), yaw, params);
                min_point = std::min(min_point, pc.point);
                min_footprint = std::min(min_footprint, pc.footprint);
                seg_sum += (s == 0 || s == n) ? 0.5 * pc.point : pc.point; // trapezoidal rule
            }
            weighted_sum += seg_sum / n * len;
            total_length += len;
        }

        out.min_clearance = min_point;
        out.min_footprint_clearance = min_footprint;
        out.mean_clearance = total_length > 1e-9 ? weighted_sum / total_length : min_point;
        return out;
    }

}
