#include "cbf_rrt_planner/path_smoother.hpp"
#include "cbf_rrt_planner/cbf_utils.hpp"
#include <cmath>

namespace cbf_rrt_planner
{

    PathSmoother::PathSmoother(
    const CollisionChecker & checker,
    const PSFGenerator * psf,
    const PlannerParams & params)
    : collision_checker_(checker), psf_generator_(psf), params_(params)
    {
    }

    bool PathSmoother::isEdgeAdmissible(double x1, double y1, double x2, double y2) const
    {
        if (!collision_checker_.isEdgeFree(x1, y1, x2, y2)) {
            return false;
        }
        if (!params_.enable_cbf || !psf_generator_) {
            return true;
        }
        return checkCbfCondition(
            *psf_generator_, x1, y1, x2, y2,
            params_.kappa, params_.nominal_velocity, params_.edge_sample_step);
    }

    std::vector<std::pair<double, double>> PathSmoother::smoothPath(
    const std::vector<std::pair<double, double>> & path) const
    {
        if (path.size() <= 2) { return path; }

        std::vector<std::pair<double, double>> smoothed_path;
        smoothed_path.push_back(path.front());

        size_t current_idx = 0;
        while (current_idx < path.size() - 1) {
            size_t furthest_visible = current_idx + 1;
            for (size_t i = path.size() - 1; i > current_idx + 1; --i) {
                if (isEdgeAdmissible(
                    path[current_idx].first, path[current_idx].second,
                    path[i].first, path[i].second)) {
                    furthest_visible = i;
                    break;
                }
            }
            smoothed_path.push_back(path[furthest_visible]);
            current_idx = furthest_visible;
        }
        return smoothed_path;
    }

}