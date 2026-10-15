#include "cbf_rrt_planner/planning_metrics.hpp"

#include <filesystem>

namespace cbf_rrt_planner
{

    std::ofstream openCsvForAppend(const std::string & csv_path, const std::string & header)
    {
        namespace fs = std::filesystem;
        fs::path path(csv_path);
        if (path.has_parent_path()) {fs::create_directories(path.parent_path());}

        std::string first_line;
        {
            std::ifstream check(csv_path);
            if (check.good()) {std::getline(check, first_line);}
        }

        if (!first_line.empty() && first_line != header) {
            fs::path old_path;
            for (int n = 1; ; ++n) {
                old_path = path.parent_path() /
                    (path.stem().string() + "_old" + std::to_string(n) + path.extension().string());
                if (!fs::exists(old_path)) {break;}
            }
            fs::rename(path, old_path);
            first_line.clear();
        }

        std::ofstream f(csv_path, std::ios::app);
        if (first_line.empty()) {f << header << "\n";}
        return f;
    }

    void PlanningMetricsLogger::appendResult(
    const std::string & csv_path,
    const std::string & run_label,
    const PlanningResult & result)
    {
        std::ofstream f = openCsvForAppend(
            csv_path,
            "run_label,success,length,mean_h,min_h,min_clearance,mean_clearance,min_footprint_clearance,"
            "iterations_used,iterations_to_first_solution,cbf_candidates,cbf_rejections,rejection_ratio");

        f << run_label << ","
            << (result.success ? "1" : "0") << ","
            << result.total_length << ","
            << result.mean_h << ","
            << result.min_h << ","
            << result.min_clearance << ","
            << result.mean_clearance << ","
            << result.min_footprint_clearance << ","
            << result.iterations_used << ","
            << result.iterations_to_first_solution << ","
            << result.cbf_candidates << ","
            << result.cbf_rejections << ","
            << result.rejectionRatio() << "\n";
    }

}
