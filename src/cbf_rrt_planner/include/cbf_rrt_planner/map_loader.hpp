#pragma once
#include <map>
#include <string>

#include "cbf_rrt_planner/occupancy_grid_data.hpp"

namespace cbf_rrt_planner
{

    // 'key: value' lines of a YAML file, nesting ignored, comments and quotes stripped. enough for
    // map.yaml and for the flat ros__parameters block of a node params file
    std::map<std::string, std::string> readFlatYaml(const std::string & path);

    // ROS-free reader of a map_server map (map.yaml + PGM image), converted the way
    // nav2_map_server does it: occ = 1 - pixel/maxval (or pixel/maxval with negate: 1),
    // modes trinary / scale / raw, image rows flipped so that row 0 is the bottom (origin) row.
    // throws std::runtime_error on unreadable or unsupported input
    OccupancyGridData loadMapYaml(const std::string & yaml_path);

}
