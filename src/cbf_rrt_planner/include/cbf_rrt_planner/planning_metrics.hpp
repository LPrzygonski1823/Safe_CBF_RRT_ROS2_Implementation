#pragma once
#include <string>

#include "cbf_rrt_planner/rrt_star_types.hpp"

namespace cbf_rrt_planner
{

    // export of single planning to CSV
    // (Length, h_bar, Iterations, Rejections)
    class PlanningMetricsLogger
    {
    public:
        static void appendResult( const std::string & csv_path, const std::string & run_label, const PlanningResult & result);
    };

}