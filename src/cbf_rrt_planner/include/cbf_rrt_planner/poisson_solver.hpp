#pragma once
#include "cbf_rrt_planner/grid2d.hpp"
#include "cbf_rrt_planner/obstacle_segmentation.hpp"

namespace cbf_rrt_planner
{

    // vector field u = [u_x, u_y] - section III.C ("Poisson Safety Function Generation"), equation (4)
    struct VectorField
    {
    Grid2D x;
    Grid2D y;
    };

    class PoissonSolver
    {
    public:
        // iterational Gauss-Seidel solver with successive over-relaxation (SOR):
        // omega - relaxation parameter (1.0 = Gauss-Seidel, 1<omega<2 = acceleration)
        // tolerance
        // max_iterations
        explicit PoissonSolver(double omega = 1.8, double tolerance = 1e-6, int max_iterations = 20000);

        // Laplace equation solved 2x - for x and y in u vector field
        // edge case for dO_i: u = boundary_scale * (p - centroid)
        // 'c_i' (4) as scalar (boundary_scale) is an assumption - not literally from article
        VectorField solveLaplaceVector(const SegmentationResult & seg, double boundary_scale = 1.0) const;

        // Frobenius norm of a Jacobi matrix of a vector field
        // right side (source term) of Poisson equation, equation (5)
        // Frobenius norm usage is an assumption
        Grid2D computeJacobianFrobeniusNorm(const VectorField & u, double resolution) const;

        // solving Poisson equation having source_term defined (output of computeJacobianFrobeniusNorm)
        // edge case: h=0 on ObstacleBoundary and DomainBoundary
        Grid2D solvePoissonScalar(const SegmentationResult & seg, const Grid2D & source_term) const;

        // gradient of any scalar field
        void computeGradient(
            const Grid2D & field, double resolution,
            Grid2D & dfdx, Grid2D & dfdy) const;

    private:
        double omega_;
        double tolerance_;
        int max_iterations_;

        // common SOR usage - used 3 times for u_x, u_y and h
        Grid2D solveScalarSOR(
            const SegmentationResult & seg,
            const Grid2D & dirichlet_values,
            const Grid2D & source_term) const;
    };

}