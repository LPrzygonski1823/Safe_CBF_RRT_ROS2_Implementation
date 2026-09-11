#include "cbf_rrt_planner/rrt_star_planner.hpp"
#include "cbf_rrt_planner/cbf_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace cbf_rrt_planner
{

    namespace
    {
    inline double distance(double x1, double y1, double x2, double y2) {
        return std::hypot(x2 - x1, y2 - y1);
        }
    }

    RrtStarPlanner::RrtStarPlanner(
    const CollisionChecker & collision_checker,
    const PSFGenerator * psf_generator,
    const PlannerParams & params)
    : collision_checker_(collision_checker),
    psf_generator_(psf_generator),
    params_(params),
    rng_(std::random_device{}()),
    spatial_grid_(params.neighbor_radius) {
    }

    int RrtStarPlanner::findNearest(double x, double y) const {
        return spatial_grid_.queryNearest(x, y);
    }

    std::vector<int> RrtStarPlanner::findNeighbors(double x, double y, double radius) const {
        return spatial_grid_.queryRadius(x, y, radius);
    }

    double RrtStarPlanner::currentNeighborRadius(size_t tree_size) const
    {
        double r;
        if (tree_size <= 1) {
            r = params_.step_size;
        }
        else {
            double n = static_cast<double>(tree_size);
            r = params_.gamma * std::sqrt(std::log(n) / n);
        }
        r = std::max(r, params_.step_size);
        r = std::min(r, params_.neighbor_radius);
        return r;
    }

    std::pair<double, double> RrtStarPlanner::steer(
    double from_x, double from_y, double to_x, double to_y) const
    {
        double dist = distance(from_x, from_y, to_x, to_y);
        if (dist <= params_.step_size || dist < 1e-9) {
            return {to_x, to_y};
        }
        double t = params_.step_size / dist;
        return {from_x + t * (to_x - from_x), from_y + t * (to_y - from_y)};
    }

    double RrtStarPlanner::averageH(double x1, double y1, double x2, double y2) const
    {
        return computeAverageH(*psf_generator_, x1, y1, x2, y2, params_.edge_sample_step);
    }

    bool RrtStarPlanner::passesCbfCondition(double x1, double y1, double x2, double y2) const
    {
        return checkCbfCondition(*psf_generator_, x1, y1, x2, y2, 
                                 params_.kappa, params_.nominal_velocity, params_.edge_sample_step);
    }

    double RrtStarPlanner::edgeCost(double x1, double y1, double x2, double y2) const
    {
        double L = distance(x1, y1, x2, y2);
        if (!params_.enable_cbf) {
            return L;
        }
        double h_bar = std::max(averageH(x1, y1, x2, y2), 1e-6);
        return params_.safety_weight_c * L + (1.0 - params_.safety_weight_c) / h_bar;
    }

    bool RrtStarPlanner::isEdgeAdmissible(double x1, double y1, double x2, double y2, PlanningResult & metrics_out) const
    {
        if (!collision_checker_.isEdgeFree(x1, y1, x2, y2)) {
            return false;
        }
        if (!params_.enable_cbf) {
            return true;
        }

        metrics_out.cbf_candidates++;
        bool ok = passesCbfCondition(x1, y1, x2, y2);
        if (!ok) { metrics_out.cbf_rejections++; }
        return ok;
    }

    std::pair<double, double> RrtStarPlanner::sampleRandomPoint(double goal_x, double goal_y)
    {
        std::uniform_real_distribution<double> goal_bias_dist(0.0, 1.0);
        if (goal_bias_dist(rng_) < params_.goal_bias) {
            return {goal_x, goal_y};
        }
        std::uniform_real_distribution<double> x_dist(collision_checker_.minX(), collision_checker_.maxX());
        std::uniform_real_distribution<double> y_dist(collision_checker_.minY(), collision_checker_.maxY());
        return {x_dist(rng_), y_dist(rng_)};
    }

    std::vector<std::pair<double, double>> RrtStarPlanner::reconstructPath(
    const std::vector<TreeNode> & tree, int node_index) const
    {
        std::vector<std::pair<double, double>> path;
        int idx = node_index;
        while (idx != -1) {
            path.push_back({tree[idx].x, tree[idx].y});
            idx = tree[idx].parent_index;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    void RrtStarPlanner::addChild(int parent_idx, int child_idx) {
        children_[parent_idx].push_back(child_idx);
    }

    void RrtStarPlanner::removeChild(int parent_idx, int child_idx) {
        auto & vec = children_[parent_idx];
        vec.erase(std::remove(vec.begin(), vec.end(), child_idx), vec.end());
    }

    void RrtStarPlanner::propagateCostUpdate(std::vector<TreeNode> & tree, int idx)
    {
        // recursive updating new, lower cost on other affected nodes
        // node positions (x,y) remain unaffected wso each child's cost is (new parent cost) + (same unaffected parent-child node cost)
        for (int child : children_[idx]) {
            tree[child].cost_to_come = tree[idx].cost_to_come +
            edgeCost(tree[idx].x, tree[idx].y, tree[child].x, tree[child].y);
            propagateCostUpdate(tree, child);
        }
    }

    PlanningResult RrtStarPlanner::plan(double start_x, double start_y, double goal_x, double goal_y)
    {
        PlanningResult result;

        std::vector<TreeNode> tree;
        tree.push_back({start_x, start_y, 0.0, -1});

        spatial_grid_.clear();
        spatial_grid_.insert(0, start_x, start_y);

        children_.clear();
        children_.push_back({});

        std::vector<int> goal_node_indices;

        for (int iter = 0; iter < params_.max_iterations; ++iter) {
            auto [sx, sy] = sampleRandomPoint(goal_x, goal_y);
            int nearest_idx = findNearest(sx, sy);
            auto [nx, ny] = steer(tree[nearest_idx].x, tree[nearest_idx].y, sx, sy);

            if (!collision_checker_.isPointFree(nx, ny)) {continue;}

            // neighborhood radius calculated based on the CURRENT tree size (before adding the new node) - decreases as the tree grows
            double neighbor_radius_now = currentNeighborRadius(tree.size());
            auto neighbor_indices = findNeighbors(nx, ny, neighbor_radius_now);
            if (std::find(neighbor_indices.begin(), neighbor_indices.end(), nearest_idx) == neighbor_indices.end()) {
                neighbor_indices.push_back(nearest_idx);
            }

            // 1. choice of cost-minimizing parent
            int best_parent = -1;
            double best_cost = std::numeric_limits<double>::infinity();
            for (int idx : neighbor_indices) {
                if (!isEdgeAdmissible(tree[idx].x, tree[idx].y, nx, ny, result)) {continue;}
                double c = tree[idx].cost_to_come + edgeCost(tree[idx].x, tree[idx].y, nx, ny);
                if (c < best_cost) {
                    best_cost = c;
                    best_parent = idx;
                }
            }
            if (best_parent == -1) {continue;}

            tree.push_back({nx, ny, best_cost, best_parent});
            int new_idx = static_cast<int>(tree.size()) - 1;
            children_.push_back({});
            addChild(best_parent, new_idx);
            spatial_grid_.insert(new_idx, nx, ny);

            // 2. rewiring
            for (int idx : neighbor_indices) {
                if (idx == best_parent) {continue;}
                if (!isEdgeAdmissible(nx, ny, tree[idx].x, tree[idx].y, result)) {continue;}
                double alt_cost = tree[new_idx].cost_to_come + edgeCost(nx, ny, tree[idx].x, tree[idx].y);
                if (alt_cost < tree[idx].cost_to_come) {
                    int old_parent = tree[idx].parent_index;
                    removeChild(old_parent, idx);
                    tree[idx].parent_index = new_idx;
                    tree[idx].cost_to_come = alt_cost;
                    addChild(new_idx, idx);
                    propagateCostUpdate(tree, idx);
                }
            }

            if (distance(nx, ny, goal_x, goal_y) <= params_.goal_tolerance) {
                if (isEdgeAdmissible(nx, ny, goal_x, goal_y, result)) {
                    double goal_cost = tree[new_idx].cost_to_come + edgeCost(nx, ny, goal_x, goal_y);
                    tree.push_back({goal_x, goal_y, goal_cost, new_idx});
                    int goal_idx = static_cast<int>(tree.size()) - 1;
                    children_.push_back({});
                    addChild(new_idx, goal_idx);
                    goal_node_indices.push_back(goal_idx);
                }
            }
        }

        result.iterations_used = params_.max_iterations;

        int best_goal_idx = -1;
        double min_final_cost = std::numeric_limits<double>::infinity();
        for (int idx : goal_node_indices)
        {
            if (tree[idx].cost_to_come < min_final_cost)
            {
                min_final_cost = tree[idx].cost_to_come;
                best_goal_idx = idx;
            }
        }

        if (best_goal_idx != -1)
        {
            result.success = true;
            result.path = reconstructPath(tree, best_goal_idx);

            double total_len = 0.0;
            double weighted_h_sum = 0.0;
            for (size_t i = 0; i + 1 < result.path.size(); ++i)
            {
                double x1 = result.path[i].first, y1 = result.path[i].second;
                double x2 = result.path[i + 1].first, y2 = result.path[i + 1].second;
                double seg_len = distance(x1, y1, x2, y2);
                total_len += seg_len;
                if (params_.enable_cbf && psf_generator_ != nullptr)
                {
                    weighted_h_sum += averageH(x1, y1, x2, y2) * seg_len;
                }
            }
            result.total_length = total_len;
            if (params_.enable_cbf && total_len > 1e-9)
            {
                result.mean_h = weighted_h_sum / total_len;
            }
        }

        return result;
    }

}