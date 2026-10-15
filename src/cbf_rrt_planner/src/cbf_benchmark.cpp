// Offline benchmark of the planner core, without ROS: loads a map_server map, generates the PSF
// once, then plans from a fixed start to a set of goals for every safety_weight_c value (and
// optionally the plain RRT* baseline), several seeds each, and appends one CSV row per plan.
// The planner parameters are read from the same YAML as planner_node.

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <queue>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "cbf_rrt_planner/collision_checker.hpp"
#include "cbf_rrt_planner/map_inflation.hpp"
#include "cbf_rrt_planner/map_loader.hpp"
#include "cbf_rrt_planner/path_clearance.hpp"
#include "cbf_rrt_planner/path_smoother.hpp"
#include "cbf_rrt_planner/planning_metrics.hpp"
#include "cbf_rrt_planner/psf_generator.hpp"
#include "cbf_rrt_planner/rrt_star_planner.hpp"

using namespace cbf_rrt_planner;

namespace
{

const char * kUsage =
  "Usage: cbf_benchmark [options]\n"
  "  --map FILE          map_server yaml (default maps/map.yaml)\n"
  "  --params FILE       planner parameters (default src/cbf_rrt_planner/config/cbf_rrt_planner.yaml)\n"
  "  --set KEY=VALUE     override one planner parameter, repeatable (e.g. --set psf_source=centroid)\n"
  "  --c LIST            comma-separated safety_weight_c values (default: the value from --params)\n"
  "  --baseline          also run plain RRT* (enable_cbf=false) with the same goals and seeds\n"
  "  --goals FILE        goals, one 'x,y' per line ('#' comments allowed)\n"
  "  --random-goals N    draw N reachable goals instead (default 10 when --goals is not given)\n"
  "  --goal-seed S       seed of the goal draw (default 1)\n"
  "  --min-goal-dist D   minimum straight-line start-goal distance of drawn goals [m] (default 2.0)\n"
  "  --runs N            planner seeds per goal and configuration (default 10)\n"
  "  --seed S            first planner seed (default 1)\n"
  "  --start X,Y         start position in map coordinates (default 0,0)\n"
  "  --out FILE          results CSV, appended (default src/cbf_rrt_planner/benchmark/results.csv);\n"
  "                      paths go to <out>_paths.csv\n"
  "  --tag TEXT          free label stored in every row (e.g. the map name)\n";

struct Options
{
  std::string map_path = "maps/map.yaml";
  std::string params_path = "src/cbf_rrt_planner/config/cbf_rrt_planner.yaml";
  std::vector<std::pair<std::string, std::string>> overrides;
  std::vector<double> c_values;
  bool baseline = false;
  std::string goals_path;
  int random_goals = 10;
  unsigned goal_seed = 1;
  double min_goal_dist = 2.0;
  int runs = 10;
  long seed = 1;
  double start_x = 0.0;
  double start_y = 0.0;
  std::string out_path = "src/cbf_rrt_planner/benchmark/results.csv";
  std::string tag;
};

std::vector<double> parseList(const std::string & s)
{
  std::vector<double> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    if (!item.empty()) {out.push_back(std::stod(item));}
  }
  return out;
}

Options parseArgs(int argc, char ** argv)
{
  Options o;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() -> std::string {
        if (i + 1 >= argc) {throw std::runtime_error("missing value after " + a);}
        return argv[++i];
      };
    if (a == "--map") {o.map_path = next();}
    else if (a == "--params") {o.params_path = next();}
    else if (a == "--set") {
      std::string kv = next();
      size_t eq = kv.find('=');
      if (eq == std::string::npos) {throw std::runtime_error("--set expects KEY=VALUE, got '" + kv + "'");}
      o.overrides.emplace_back(kv.substr(0, eq), kv.substr(eq + 1));
    }
    else if (a == "--c") {o.c_values = parseList(next());}
    else if (a == "--baseline") {o.baseline = true;}
    else if (a == "--goals") {o.goals_path = next();}
    else if (a == "--random-goals") {o.random_goals = std::stoi(next());}
    else if (a == "--goal-seed") {o.goal_seed = static_cast<unsigned>(std::stoul(next()));}
    else if (a == "--min-goal-dist") {o.min_goal_dist = std::stod(next());}
    else if (a == "--runs") {o.runs = std::stoi(next());}
    else if (a == "--seed") {o.seed = std::stol(next());}
    else if (a == "--start") {
      auto xy = parseList(next());
      if (xy.size() != 2) {throw std::runtime_error("--start expects X,Y");}
      o.start_x = xy[0];
      o.start_y = xy[1];
    }
    else if (a == "--out") {o.out_path = next();}
    else if (a == "--tag") {o.tag = next();}
    else if (a == "-h" || a == "--help") {std::cout << kUsage; std::exit(0);}
    else {throw std::runtime_error("unknown option '" + a + "'");}
  }
  if (o.runs < 1) {throw std::runtime_error("--runs must be >= 1");}
  return o;
}

// planner parameters with the planner_node defaults, overridden by the YAML and by --set
class ParamTable
{
public:
  ParamTable()
  {
    values_ = {
      {"step_size", "1.0"}, {"max_iterations", "10000"}, {"goal_bias", "0.05"},
      {"goal_tolerance", "1.0"}, {"neighbor_radius", "5.0"}, {"gamma", "5.0"},
      {"enable_cbf", "true"}, {"kappa", "5.0"}, {"safety_weight_c", "0.2"},
      {"nominal_velocity", "1.0"}, {"occupied_threshold", "65"}, {"enable_smoothing", "false"},
      {"inflation", "true"}, {"inflation_robot_size", "0.22"},
      {"psf_source", "centroid"}, {"psf_boundary_flux", "1.0"},
      {"footprint_length", "0.33"}, {"footprint_width", "0.28"}, {"clearance_search_radius", "2.0"},
    };
  }

  // keys the benchmark does not use (ROS integration, seed, csv path) are ignored silently
  void merge(const std::map<std::string, std::string> & m)
  {
    for (const auto & [k, v] : m) {
      if (values_.count(k)) {values_[k] = v;}
    }
  }

  void set(const std::string & key, const std::string & value)
  {
    if (!values_.count(key)) {throw std::runtime_error("unknown planner parameter '" + key + "'");}
    values_[key] = value;
  }

  double d(const std::string & k) const {return std::stod(values_.at(k));}
  int i(const std::string & k) const {return std::stoi(values_.at(k));}
  bool b(const std::string & k) const
  {
    std::string v = values_.at(k);
    std::transform(v.begin(), v.end(), v.begin(), ::tolower);
    if (v == "true" || v == "1") {return true;}
    if (v == "false" || v == "0") {return false;}
    throw std::runtime_error("parameter '" + k + "' is not a bool: '" + v + "'");
  }
  const std::string & s(const std::string & k) const {return values_.at(k);}

private:
  std::map<std::string, std::string> values_;
};

bool isFreeValue(int8_t v, int threshold) {return v >= 0 && v < threshold;}

std::vector<std::pair<double, double>> readGoals(const std::string & path)
{
  std::ifstream f(path);
  if (!f) {throw std::runtime_error("cannot open goals file '" + path + "'");}
  std::vector<std::pair<double, double>> goals;
  std::string line;
  while (std::getline(f, line)) {
    size_t hash = line.find('#');
    if (hash != std::string::npos) {line = line.substr(0, hash);}
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream in(line);
    double x, y;
    if (in >> x >> y) {goals.emplace_back(x, y);}
  }
  if (goals.empty()) {throw std::runtime_error("no goals in '" + path + "'");}
  return goals;
}

// goals are drawn from the free cells of the inflated map that are 4-connected to the start, so
// a failure always means the planner did not find an existing path
std::vector<std::pair<double, double>> drawGoals(
  const OccupancyGridData & map, int threshold, double sx, double sy,
  int count, double min_dist, unsigned seed)
{
  const int W = map.width;
  const int H = map.height;
  const double res = map.resolution;
  const int c0 = static_cast<int>(std::floor((sx - map.origin_x) / res));
  const int r0 = static_cast<int>(std::floor((sy - map.origin_y) / res));

  std::vector<char> seen(static_cast<size_t>(W) * H, 0);
  std::vector<std::pair<double, double>> candidates;
  std::queue<std::pair<int, int>> q;
  q.push({r0, c0});
  seen[static_cast<size_t>(r0) * W + c0] = 1;
  const int dr[4] = {-1, 1, 0, 0};
  const int dc[4] = {0, 0, -1, 1};

  while (!q.empty()) {
    auto [r, c] = q.front();
    q.pop();
    double x = map.origin_x + (c + 0.5) * res;
    double y = map.origin_y + (r + 0.5) * res;
    if (std::hypot(x - sx, y - sy) >= min_dist) {candidates.emplace_back(x, y);}
    for (int k = 0; k < 4; ++k) {
      int nr = r + dr[k], nc = c + dc[k];
      if (nr < 0 || nr >= H || nc < 0 || nc >= W) {continue;}
      size_t idx = static_cast<size_t>(nr) * W + nc;
      if (seen[idx] || !isFreeValue(map.data[idx], threshold)) {continue;}
      seen[idx] = 1;
      q.push({nr, nc});
    }
  }

  std::mt19937 rng(seed);
  std::shuffle(candidates.begin(), candidates.end(), rng);
  if (static_cast<int>(candidates.size()) < count) {
    std::cerr << "warning: only " << candidates.size() << " reachable goal cells farther than "
              << min_dist << " m from the start\n";
    count = static_cast<int>(candidates.size());
  }
  candidates.resize(count);
  return candidates;
}

const char * failureName(PlanningFailure f)
{
  switch (f) {
    case PlanningFailure::None: return "none";
    case PlanningFailure::StartBlocked: return "start_blocked";
    case PlanningFailure::GoalBlocked: return "goal_blocked";
    default: return "no_path";
  }
}

std::string timestamp()
{
  std::time_t t = std::time(nullptr);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y%m%d-%H%M%S", std::localtime(&t));
  return buf;
}

struct Config
{
  std::string name;
  bool enable_cbf;
  double c;
};

double median(std::vector<double> v)
{
  if (v.empty()) {return std::nan("");}
  std::sort(v.begin(), v.end());
  size_t n = v.size();
  return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

int run(const Options & opt)
{
  ParamTable P;
  P.merge(readFlatYaml(opt.params_path));
  for (const auto & [k, v] : opt.overrides) {P.set(k, v);}

  const int threshold = P.i("occupied_threshold");
  const double inflation_radius = P.b("inflation") ? std::max(0.0, P.d("inflation_robot_size")) : 0.0;
  const std::string source_name = P.s("psf_source");
  if (source_name != "centroid" && source_name != "constant") {
    throw std::runtime_error("psf_source must be 'centroid' or 'constant', got '" + source_name + "'");
  }
  const bool smoothing = P.b("enable_smoothing");

  OccupancyGridData raw_map = loadMapYaml(opt.map_path);
  OccupancyGridData planning_map = inflateObstacles(raw_map, inflation_radius, threshold);
  std::cout << "Map " << opt.map_path << ": " << raw_map.width << " x " << raw_map.height
            << " cells, " << raw_map.resolution << " m/cell, inflation " << inflation_radius << " m\n";

  CollisionChecker checker(planning_map, threshold);
  if (!checker.isPointFree(opt.start_x, opt.start_y)) {
    CollisionChecker raw_checker(raw_map, threshold);
    std::cerr << "error: start (" << opt.start_x << ", " << opt.start_y << ") is "
              << (raw_checker.isPointFree(opt.start_x, opt.start_y) ?
        "within the inflation radius of an obstacle" : "in an occupied, unknown or off-map cell")
              << " - pick another one with --start X,Y\n";
    return 2;
  }

  PSFGenerator psf;
  psf.setOccupiedThreshold(threshold);
  psf.setSource(
    source_name == "constant" ? PsfSource::Constant : PsfSource::Centroid,
    P.d("psf_boundary_flux"));
  auto t_psf = std::chrono::steady_clock::now();
  psf.generate(planning_map);
  double psf_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_psf).count();
  std::cout << "PSF '" << source_name << "' ready in " << psf_time << " s ("
            << psf.obstacleCount() << " obstacles";
  if (psf.source() == PsfSource::Constant) {std::cout << ", f0 = " << psf.constantSource() << " 1/m";}
  std::cout << ")\n";

  auto goals = opt.goals_path.empty() ?
    drawGoals(planning_map, threshold, opt.start_x, opt.start_y,
              opt.random_goals, opt.min_goal_dist, opt.goal_seed) :
    readGoals(opt.goals_path);
  for (size_t g = 0; g < goals.size(); ++g) {
    std::printf("goal %zu: %.3f, %.3f\n", g, goals[g].first, goals[g].second);
  }

  std::vector<Config> configs;
  std::vector<double> c_values = opt.c_values.empty() ?
    std::vector<double>{P.d("safety_weight_c")} : opt.c_values;
  for (double c : c_values) {configs.push_back({"cbf", true, c});}
  if (opt.baseline) {configs.push_back({"baseline", false, std::nan("")});}

  PlannerParams base;
  base.step_size = P.d("step_size");
  base.max_iterations = P.i("max_iterations");
  base.goal_bias = P.d("goal_bias");
  base.goal_tolerance = P.d("goal_tolerance");
  base.neighbor_radius = P.d("neighbor_radius");
  base.gamma = P.d("gamma");
  base.edge_sample_step = raw_map.resolution * 0.5;
  base.kappa = P.d("kappa");
  base.nominal_velocity = P.d("nominal_velocity");
  base.occupied_threshold = threshold;

  ClearanceParams clearance_params;
  clearance_params.footprint_length = P.d("footprint_length");
  clearance_params.footprint_width = P.d("footprint_width");
  clearance_params.search_radius = P.d("clearance_search_radius");
  clearance_params.sample_step = raw_map.resolution;
  clearance_params.occupied_threshold = threshold;

  namespace fs = std::filesystem;
  fs::path out(opt.out_path);
  std::string paths_path =
    (out.parent_path() / (out.stem().string() + "_paths" + out.extension().string())).string();

  std::ofstream rows = openCsvForAppend(
    opt.out_path,
    "batch,tag,map,psf_source,psf_boundary_flux,inflation_radius,kappa,nominal_velocity,"
    "max_iterations,smoothing,config,enable_cbf,safety_weight_c,goal_id,start_x,start_y,goal_x,goal_y,"
    "straight_dist,run,seed,success,failure,length,length_ratio,mean_h,min_h,min_clearance,"
    "mean_clearance,min_footprint_clearance,iterations_used,iterations_to_first_solution,"
    "cbf_candidates,cbf_rejections,rejection_ratio,plan_time_s,psf_time_s");
  std::ofstream paths = openCsvForAppend(paths_path, "batch,run,seq,x,y");
  rows.precision(6);
  paths.precision(4);
  paths << std::fixed;

  const std::string batch = timestamp();
  const std::string map_name = fs::path(opt.map_path).stem().string();
  int run_id = 0;
  const size_t total = goals.size() * configs.size() * opt.runs;
  std::cout << "Batch " << batch << ": " << goals.size() << " goals x " << configs.size()
            << " configurations x " << opt.runs << " seeds = " << total << " plans\n";

  for (size_t g = 0; g < goals.size(); ++g) {
    const auto [gx, gy] = goals[g];
    const double straight = std::hypot(gx - opt.start_x, gy - opt.start_y);
    std::map<size_t, std::vector<double>> lengths, footprints;
    std::map<size_t, int> successes;

    // the seed loop is outside the configuration loop: every configuration gets exactly the same
    // seeds for a goal, so the runs can be compared in pairs
    for (int k = 0; k < opt.runs; ++k) {
      const long seed = opt.seed + static_cast<long>(g) * opt.runs + k;
      for (size_t ci = 0; ci < configs.size(); ++ci) {
        const Config & cfg = configs[ci];
        PlannerParams params = base;
        params.enable_cbf = cfg.enable_cbf;
        params.safety_weight_c = cfg.enable_cbf ? cfg.c : P.d("safety_weight_c");
        params.seed = seed;

        auto t0 = std::chrono::steady_clock::now();
        RrtStarPlanner planner(checker, &psf, params);
        PlanningResult result = planner.plan(opt.start_x, opt.start_y, gx, gy);
        if (result.success && smoothing) {smoothResult(result, checker, &psf, params);}
        double plan_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

        if (result.success) {
          auto cl = computePathClearance(raw_map, result.path, clearance_params);
          result.min_clearance = cl.min_clearance;
          result.mean_clearance = cl.mean_clearance;
          result.min_footprint_clearance = cl.min_footprint_clearance;
          lengths[ci].push_back(result.total_length);
          footprints[ci].push_back(result.min_footprint_clearance);
          successes[ci]++;
          for (size_t s = 0; s < result.path.size(); ++s) {
            paths << batch << "," << run_id << "," << s << ","
                  << result.path[s].first << "," << result.path[s].second << "\n";
          }
        }

        const double nan = std::nan("");
        rows << batch << "," << opt.tag << "," << map_name << "," << source_name << ","
             << P.d("psf_boundary_flux") << "," << inflation_radius << "," << params.kappa << ","
             << params.nominal_velocity << "," << params.max_iterations << "," << (smoothing ? 1 : 0) << ","
             << cfg.name << "," << (cfg.enable_cbf ? 1 : 0) << "," << cfg.c << ","
             << g << "," << opt.start_x << "," << opt.start_y << "," << gx << "," << gy << ","
             << straight << "," << run_id << "," << seed << ","
             << (result.success ? 1 : 0) << "," << failureName(result.failure) << ","
             << (result.success ? result.total_length : nan) << ","
             << (result.success && straight > 1e-9 ? result.total_length / straight : nan) << ","
             << (result.success ? result.mean_h : nan) << ","
             << (result.success ? result.min_h : nan) << ","
             << result.min_clearance << "," << result.mean_clearance << ","
             << result.min_footprint_clearance << ","
             << result.iterations_used << "," << result.iterations_to_first_solution << ","
             << result.cbf_candidates << "," << result.cbf_rejections << ","
             << result.rejectionRatio() << "," << plan_time << "," << psf_time << "\n";
        ++run_id;
      }
    }
    rows.flush();
    paths.flush();

    std::printf("goal %zu (%.2f, %.2f), straight %.2f m:\n", g, gx, gy, straight);
    for (size_t ci = 0; ci < configs.size(); ++ci) {
      std::string name = configs[ci].enable_cbf ?
        "c=" + std::to_string(configs[ci].c).substr(0, 4) : "baseline";
      std::printf("  %-9s success %d/%d, median length %.2f m, median footprint clearance %.3f m\n",
                  name.c_str(), successes[ci], opt.runs, median(lengths[ci]), median(footprints[ci]));
    }
  }

  std::cout << "Results appended to " << opt.out_path << " and " << paths_path
            << " (batch " << batch << ")\n";
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    return run(parseArgs(argc, argv));
  } catch (const std::exception & e) {
    std::cerr << "error: " << e.what() << "\n\n" << kUsage;
    return 1;
  }
}
