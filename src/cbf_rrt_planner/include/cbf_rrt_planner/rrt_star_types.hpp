#pragma once
#include <vector>
#include <utility>

namespace cbf_rrt_planner
{

    // single node of the RRT* tree
    // reconstructs the path once, at the end of planning, by following the parent pointers from the goal to the root
    struct TreeNode
    {
        double x = 0.0;
        double y = 0.0;
        double  cost_to_come = 0.0; // total cost from the root to this node
        int parent_index = -1; // -1 for the root
    };

    // parameters controlling the planner's behavior. enable_cbf=false exactly reproduces
    // the classic RRT*
    struct PlannerParams
    {
        // base RRT* parameters
        double  step_size = 1.5;
        int     max_iterations = 10000;
        double  goal_bias = 0.05; // probability of sampling the goal directly
        double  goal_tolerance = 1.0;

        // The neighborhood radius is not constant - according to RRT* theory (Karaman & Frazzoli)
        // it must decrease as the tree grows, using the formula r_n = gamma*(log(n)/n)^(1/d).
        // For d=2: r_n = gamma*sqrt(log(n)/n) - otherwise the algorithm loses the guarantee
        // of asymptotic convergence to the optimal path, and with a large tree
        // every neighbor query returns more and more candidates (slowdown).
        // 'neighbor_radius' now serves a dual role: (a) upper bound for r_n (we don't
        // let the radius grow beyond this value even for a small tree),
        // (b) spatial grid cell size (SpatialGrid) - a good performance compromise,
        // because this is the scale at which most queries actually happen.
        double  neighbor_radius = 5.0;
        double  gamma = 5.0;  // scaling factor in r_n = gamma*sqrt(log(n)/n)

        // safe-CBF-RRT* parameters (used only when enable_cbf == true)
        bool enable_cbf = true;
        double  kappa = 3.0; // formula (3)
        double  safety_weight_c = 0.05; // formula (6): c=1 length, c=0 safety
        double  nominal_velocity = 0.8; // assumed constant velocity along the edge
        double  edge_sample_step = 0.025; // fixed spatial distance [m] between samples for CBF (replaces cbf_samples_per_edge)

        // common geometric collision parameter
        int occupied_threshold = 65;
    };

    // planner execution result ready to be published/exported to metrics
    struct PlanningResult
    {
        bool success = false;
        std::vector<std::pair<double, double>>  path; // from start to goal, inclusive
        double total_length = 0.0;
        double mean_h = 0.0; // mean h along the entire path (only when enable_cbf is true)
        int iterations_used = 0;
        int cbf_rejections  = 0; // number of edges rejected BY CBF
        int cbf_candidates  = 0; // number of edges that passed the collision test (candidates for CBF)

        double rejectionRatio() const {
            return cbf_candidates > 0 ? static_cast<double>(cbf_rejections) / cbf_candidates : 0.0;
        }
    };

}