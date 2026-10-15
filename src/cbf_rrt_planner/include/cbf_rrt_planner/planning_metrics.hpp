#pragma once
#include <fstream>
#include <string>

#include "cbf_rrt_planner/rrt_star_types.hpp"

namespace cbf_rrt_planner
{

    // opens csv_path for appending and writes 'header' (one line, no newline) if the file is new.
    // a file whose first line differs from 'header' was written by an older version: it is moved
    // to <stem>_old<N>.csv instead of getting rows with a different column layout
    std::ofstream openCsvForAppend(const std::string & csv_path, const std::string & header);

    // export of single planning to CSV
    // (Length, h_bar, Iterations, Rejections + geometric clearance)
    class PlanningMetricsLogger
    {
    public:
        static void appendResult( const std::string & csv_path, const std::string & run_label, const PlanningResult & result);
    };

}
