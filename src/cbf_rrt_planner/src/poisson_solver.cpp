#include "cbf_rrt_planner/poisson_solver.hpp"

#include <algorithm>
#include <cmath>

namespace cbf_rrt_planner
{

    PoissonSolver::PoissonSolver(double omega, double tolerance, int max_iterations) : omega_(omega), tolerance_(tolerance), max_iterations_(max_iterations) {
    }

    Grid2D PoissonSolver::solveScalarSOR(const SegmentationResult & seg, const Grid2D & dirichlet_values, const Grid2D & source_term) const
    {
        const int H = seg.height;
        const int W = seg.width;
        const double dx2 = seg.resolution * seg.resolution;

        Grid2D field(H, W, 0.0);

        // edge cells immediately get their never-modified values
        for (int r = 0; r < H; ++r) {
            for (int c = 0; c < W; ++c) {
                auto type = static_cast<CellType>(seg.cell_type(r, c));
                if (type == CellType::ObstacleBoundary || type == CellType::DomainBoundary) {
                    field(r, c) = dirichlet_values(r, c);
                }
            }
        }

        // iterational Gauss-Seidel with overrelaxation (SOR)
        // discrete: (sasiedzi_sum - dx^2 * source) / 4 = wartosc rownowagi
        for (int iter = 0; iter < max_iterations_; ++iter) {
            double max_delta = 0.0;

            for (int r = 0; r < H; ++r) {
                for (int c = 0; c < W; ++c) {
                    auto type = static_cast<CellType>(seg.cell_type(r, c));
                    // Dirichlet (ObstacleBoundary/DomainBoundary) - don't modify
                    // ObstacleInterior - ignore
                    if (type != CellType::Free) {continue;}

                    double sum_neighbors = field(r - 1, c) + field(r + 1, c) + field(r, c - 1) + field(r, c + 1);

                    double new_val = (sum_neighbors - dx2 * source_term(r, c)) / 4.0;
                    double sor_val = (1.0 - omega_) * field(r, c) + omega_ * new_val;

                    max_delta = std::max(max_delta, std::abs(sor_val - field(r, c)));
                    field(r, c) = sor_val;
                }
            }
            if (max_delta < tolerance_) {break;}
        }

        return field;
    }

    VectorField PoissonSolver::solveLaplaceVector(const SegmentationResult & seg, double boundary_scale) const
    {
        const int H = seg.height;
        const int W = seg.width;

        Grid2D dirichlet_x(H, W, 0.0);
        Grid2D dirichlet_y(H, W, 0.0);
        Grid2D zero_source(H, W, 0.0);   // rownanie Laplace'a: prawa strona = 0

        // A3: warunek brzegowy u = c_i * (p - c_i) na dO_i, wzor (4) artykulu.
        // Na DomainBoundary (zewnetrzna krawedz mapy) przyjmujemy u=0 - artykul nie
        // precyzuje warunku brzegowego dla u na dOmega (tylko na dO_i) - zalozenie
        // DO POTWIERDZENIA z promotorem. dirichlet_x/y sa juz zainicjalizowane na 0.0.
        for (int r = 0; r < H; ++r) {
            for (int c = 0; c < W; ++c) {
                if (static_cast<CellType>(seg.cell_type(r, c)) != CellType::ObstacleBoundary) {continue;}

                int obs_id = seg.obstacle_id(r, c);
                const Obstacle & obs = seg.obstacles[obs_id];

                double wx, wy;
                seg.gridToWorld(r, c, wx, wy);

                dirichlet_x(r, c) = boundary_scale * (wx - obs.centroid_x);
                dirichlet_y(r, c) = boundary_scale * (wy - obs.centroid_y);
            }
        }

        VectorField u;
        u.x = solveScalarSOR(seg, dirichlet_x, zero_source);
        u.y = solveScalarSOR(seg, dirichlet_y, zero_source);
        return u;
    }

    void PoissonSolver::computeGradient(
    const Grid2D & field, double resolution,
    Grid2D & dfdx, Grid2D & dfdy) const
    {
    const int H = field.rows();
    const int W = field.cols();
    dfdx = Grid2D(H, W, 0.0);
    dfdy = Grid2D(H, W, 0.0);

    // central finite differences for interior points, one-sided on the boundaries of the entire grid to avoid going out of range
    for (int r = 0; r < H; ++r) {
        for (int c = 0; c < W; ++c) {
        int c_left = std::max(c - 1, 0);
        int c_right = std::min(c + 1, W - 1);
        int r_up = std::max(r - 1, 0);
        int r_down = std::min(r + 1, H - 1);

        double left = field(r, c_left);
        double right = field(r, c_right);
        double up = field(r_up, c);
        double down = field(r_down, c);

        double dx_step = (c_right - c_left) * resolution;
        double dy_step = (r_down - r_up) * resolution;

        dfdx(r, c) = dx_step > 0.0 ? (right - left) / dx_step : 0.0;
        dfdy(r, c) = dy_step > 0.0 ? (down - up) / dy_step : 0.0;
        }
    }
    }

    Grid2D PoissonSolver::computeJacobianFrobeniusNorm(const VectorField & u, double resolution) const
    {
        Grid2D dux_dx, dux_dy, duy_dx, duy_dy;
        computeGradient(u.x, resolution, dux_dx, dux_dy);
        computeGradient(u.y, resolution, duy_dx, duy_dy);

        const int H = u.x.rows();
        const int W = u.x.cols();
        Grid2D norm(H, W, 0.0);

        // Frobenius norm of Jacobi matrix of u vector field
        // ||grad u||_F = sqrt( (dux/dx)^2 + (dux/dy)^2 + (duy/dx)^2 + (duy/dy)^2 )
        for (int r = 0; r < H; ++r) {
            for (int c = 0; c < W; ++c) {
            double a = dux_dx(r, c);
            double b = dux_dy(r, c);
            double d = duy_dx(r, c);
            double e = duy_dy(r, c);
            norm(r, c) = std::sqrt(a * a + b * b + d * d + e * e);
            }
        }
        return norm;
    }

    Grid2D PoissonSolver::solvePoissonScalar(const SegmentationResult & seg, const Grid2D & source_term) const
    {
        const int H = seg.height;
        const int W = seg.width;

        // h=0 on obstacle edges (dO_i) and outer map edges (dOmega)
        Grid2D dirichlet_zero(H, W, 0.0);

        // Poisson equation
        Grid2D negated_source(H, W, 0.0);
        for (int r = 0; r < H; ++r) {
            for (int c = 0; c < W; ++c) {
                negated_source(r, c) = -source_term(r, c);
            }
        }

        return solveScalarSOR(seg, dirichlet_zero, negated_source);
    }

}