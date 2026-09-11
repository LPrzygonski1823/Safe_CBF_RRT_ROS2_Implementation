#pragma once
#include <vector>
#include <utility>
#include "cbf_rrt_planner/collision_checker.hpp"
#include "cbf_rrt_planner/psf_generator.hpp"
#include "cbf_rrt_planner/rrt_star_types.hpp"

namespace cbf_rrt_planner
{

// greedy shortcutting smoother
// simplifies the jagged RRT* path by connecting distant nodes directly if admissible
    class PathSmoother
    {
    public:
        PathSmoother(
            const CollisionChecker & collision_checker,
            const PSFGenerator * psf_generator,
            const PlannerParams & params);

        std::vector<std::pair<double, double>> smoothPath(
            const std::vector<std::pair<double, double>> & path) const;

    private:
        const CollisionChecker & collision_checker_;
        const PSFGenerator * psf_generator_;
        PlannerParams params_;

        bool isEdgeAdmissible(double x1, double y1, double x2, double y2) const;
    };

}