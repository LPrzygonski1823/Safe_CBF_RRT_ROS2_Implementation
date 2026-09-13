#pragma once
#include <random>
#include <vector>

#include "cbf_rrt_planner/collision_checker.hpp"
#include "cbf_rrt_planner/psf_generator.hpp"
#include "cbf_rrt_planner/rrt_star_types.hpp"
#include "cbf_rrt_planner/spatial_grid.hpp"

namespace cbf_rrt_planner
{

    // switch: params.enable_cbf == false -> classic RRT*
    // enable_cbf == true -> full Safe-CBF-RRT*
    class RrtStarPlanner
    {
        public:
            RrtStarPlanner(
                const CollisionChecker & collision_checker,
                const PSFGenerator * psf_generator,
                const PlannerParams & params);

            PlanningResult plan(double start_x, double start_y, double goal_x, double goal_y);

        private:
            const CollisionChecker & collision_checker_;
            const PSFGenerator * psf_generator_;
            PlannerParams params_;
            std::mt19937 rng_;

            SpatialGrid spatial_grid_;

            // list of children for each tree node needed for recursive cost propagation after rewiring (see propagateCostUpdate)
            // indexed 1:1 with the 'tree' vector used within plan()
            std::vector<std::vector<int>> children_;

            int findNearest(double x, double y) const;
            std::vector<int> findNeighbors(double x, double y, double radius) const;

            // dynamic neighbourhood radius r_n = gamma*sqrt(log(n)/n) with min step_size and max params_.neighbor_radius
            double currentNeighborRadius(size_t tree_size) const;

            std::pair<double, double> steer(double from_x, double from_y, double to_x, double to_y) const;

            bool passesCbfCondition(double x1, double y1, double x2, double y2) const;
            double averageH(double x1, double y1, double x2, double y2) const;
            double averageInverseH(double x1, double y1, double x2, double y2) const;
            double minH(double x1, double y1, double x2, double y2) const;
            double edgeCost(double x1, double y1, double x2, double y2) const;

            bool isEdgeAdmissible(double x1, double y1, double x2, double y2, PlanningResult & metrics_out) const;

            std::pair<double, double> sampleRandomPoint(double goal_x, double goal_y);

            std::vector<std::pair<double, double>> reconstructPath(
                const std::vector<TreeNode> & tree, int node_index) const;

            void addChild(int parent_idx, int child_idx);
            void removeChild(int parent_idx, int child_idx);

            // recursive cost update (cost_to_come) of the whole subtree under 'idx' after 'idx' got lower cost during rewiring
            void propagateCostUpdate(std::vector<TreeNode> & tree, int idx);
    };

}