#include "cbf_rrt_planner/planning_metrics.hpp"

#include <fstream>

namespace cbf_rrt_planner
{

    void PlanningMetricsLogger::appendResult(
    const std::string & csv_path,
    const std::string & run_label,
    const PlanningResult & result)
    {
        std::ifstream check(csv_path, std::ios::ate);
        bool has_header = check.good() && check.tellg() > 0;
        check.close();

        std::ofstream f(csv_path, std::ios::app);
        if (!has_header) {
            f << "run_label,success,length,mean_h,min_h,iterations_used,iterations_to_first_solution,"
                 "cbf_candidates,cbf_rejections,rejection_ratio\n";
        }

        f << run_label << ","
            << (result.success ? "1" : "0") << ","
            << result.total_length << ","
            << result.mean_h << ","
            << result.min_h << ","
            << result.iterations_used << ","
            << result.iterations_to_first_solution << ","
            << result.cbf_candidates << ","
            << result.cbf_rejections << ","
            << result.rejectionRatio() << "\n";
    }

}