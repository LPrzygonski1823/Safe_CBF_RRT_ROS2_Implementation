#include "cbf_rrt_planner/psf_generator.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace cbf_rrt_planner
{

    PSFGenerator::PSFGenerator()
    : segmentation_(65, true), solver_(1.8, 1e-6, 20000) {
    }

    void PSFGenerator::generate(const OccupancyGridData & map)
    {
        // A1, A2
        seg_result_ = segmentation_.segment(map);

        // A3
        u_field_ = solver_.solveLaplaceVector(seg_result_, /*boundary_scale=*/ 1.0);

        // A4
        Grid2D jacobian_norm = solver_.computeJacobianFrobeniusNorm(u_field_, seg_result_.resolution);
        h_field_ = solver_.solvePoissonScalar(seg_result_, jacobian_norm);

        // A5
        solver_.computeGradient(h_field_, seg_result_.resolution, dh_dx_field_, dh_dy_field_);

        ready_ = true;
    }

    double PSFGenerator::bilinearSample(const Grid2D & field, double x, double y) const
    {
        // world coords to real grid coords (-0.5 -> gridToWorld() outputs center of a cell)
        double col_f = (x - seg_result_.origin_x) / seg_result_.resolution - 0.5;
        double row_f = (y - seg_result_.origin_y) / seg_result_.resolution - 0.5;

        int c0 = static_cast<int>(std::floor(col_f));
        int r0 = static_cast<int>(std::floor(row_f));
        int c1 = c0 + 1;
        int r1 = r0 + 1;

        double tx = col_f - std::floor(col_f);
        double ty = row_f - std::floor(row_f);

        // points out of map range get max legal map values
        c0 = std::clamp(c0, 0, field.cols() - 1);
        c1 = std::clamp(c1, 0, field.cols() - 1);
        r0 = std::clamp(r0, 0, field.rows() - 1);
        r1 = std::clamp(r1, 0, field.rows() - 1);

        double top = field(r0, c0) * (1 - tx) + field(r0, c1) * tx;
        double bottom = field(r1, c0) * (1 - tx) + field(r1, c1) * tx;
        return top * (1 - ty) + bottom * ty;
    }

    double PSFGenerator::getH(double x, double y) const
    {
        if (!ready_) {throw std::runtime_error("PSFGenerator::getH called before generate()");}
        return bilinearSample(h_field_, x, y);
    }

    void PSFGenerator::getGradientH(double x, double y, double & dh_dx, double & dh_dy) const
    {
        if (!ready_) {throw std::runtime_error("PSFGenerator::getGradientH called before generate()");}
        dh_dx = bilinearSample(dh_dx_field_, x, y);
        dh_dy = bilinearSample(dh_dy_field_, x, y);
    }

    void PSFGenerator::exportToCsv(const std::string & directory_path) const
    {
        if (!ready_) {throw std::runtime_error("PSFGenerator::exportToCsv called before generate()");}

        auto write_grid = [&](const Grid2D & g, const std::string & filename) {
            std::ofstream f(directory_path + "/" + filename);
            for (int r = 0; r < g.rows(); ++r) {
                for (int c = 0; c < g.cols(); ++c) {
                f << g(r, c);
                if (c + 1 < g.cols()) {f << ",";}
                }
                f << "\n";
            }
            };

        write_grid(h_field_, "h.csv");
        write_grid(u_field_.x, "u_x.csv");
        write_grid(u_field_.y, "u_y.csv");
    }

}