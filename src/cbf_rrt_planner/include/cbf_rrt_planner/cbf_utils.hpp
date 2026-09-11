#pragma once
#include <algorithm>
#include <cmath>
#include "cbf_rrt_planner/psf_generator.hpp"

namespace cbf_rrt_planner
{

    inline double computeAverageH(
    const PSFGenerator & psf,
    double x1, double y1, double x2, double y2,
    double step)
    {
    double dx = x2 - x1;
    double dy = y2 - y1;
    double length = std::hypot(dx, dy);
    if (length < 1e-9) { return psf.getH(x1, y1); }

    int num_samples = std::max(1, static_cast<int>(std::ceil(length / step)));
    double sum_h = 0.0;
    for (int i = 0; i <= num_samples; ++i) {
        double t = static_cast<double>(i) / num_samples;
        sum_h += psf.getH(x1 + t * dx, y1 + t * dy);
    }
    return sum_h / (num_samples + 1);
    }

    inline bool checkCbfCondition(
    const PSFGenerator & psf,
    double x1, double y1, double x2, double y2,
    double kappa, double nominal_velocity, double step)
    {
    double dx = x2 - x1;
    double dy = y2 - y1;
    double length = std::hypot(dx, dy);
    if (length < 1e-9) { return true; }

    double vx = (dx / length) * nominal_velocity;
    double vy = (dy / length) * nominal_velocity;

    int num_samples = std::max(1, static_cast<int>(std::ceil(length / step)));
    for (int i = 0; i <= num_samples; ++i) {
        double t = static_cast<double>(i) / num_samples;
        double x = x1 + t * dx;
        double y = y1 + t * dy;

        double h = psf.getH(x, y);
        double dhdx, dhdy;
        psf.getGradientH(x, y, dhdx, dhdy);

        double lhs = dhdx * vx + dhdy * vy;
        double rhs = -kappa * h;
        if (lhs < rhs) { return false; }
    }
    return true;
    }

}