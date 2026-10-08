#pragma once
#include <string>

#include "cbf_rrt_planner/obstacle_segmentation.hpp"
#include "cbf_rrt_planner/poisson_solver.hpp"

namespace cbf_rrt_planner
{

    // source term of the Poisson equation for h
    enum class PsfSource
    {
        Centroid,   // -||grad u||_F, u = p - c_i on obstacle boundaries (equations 4-5 of the paper)
        Constant    // -f0 everywhere, f0 = |b| * perimeter / area (average flux method, Bahati et al. 2025)
    };

    // public interface of PSF
    // PlannerNode and RRT*
    class PSFGenerator
    {
    public:
        PSFGenerator();

        // runs the full pipeline segmentation -> Laplace(u) -> Jacobian -> Poisson(h) -> gradient(h)
        // ! currently a synchronous call - it blocks the calling thread for the duration of the computation
        // to be moved to std::async when integrating with planner_node.cpp
        void generate(const OccupancyGridData & map);

        bool isReady() const {return ready_;}

        // number of detected obstacles
        size_t obstacleCount() const {return seg_result_.obstacles.size();}

        // h(x,y) in any point [m], interpolated between 4 points for RRT*
        double getH(double x, double y) const;

        // gradient h(x,y) in any point [m]
        void getGradientH(double x, double y, double & dh_dx, double & dh_dy) const;

        // diagnostic tool, export the h, u_x, u_y fields to CSV files for visual verification in visualize_psf.py
        void exportToCsv(const std::string & directory_path) const;

        void setOccupiedThreshold(int threshold) { 
            segmentation_ = ObstacleSegmentation(threshold, true); 
        }

        // boundary_flux: desired mean |dh/dn| on the boundary, only used by PsfSource::Constant
        void setSource(PsfSource source, double boundary_flux) {
            source_ = source;
            boundary_flux_ = boundary_flux;
        }

        PsfSource source() const {return source_;}

        // f0 of the last Constant run, 0 for Centroid
        double constantSource() const {return constant_source_;}

    private:
        PsfSource source_ = PsfSource::Centroid;
        double boundary_flux_ = 1.0;
        double constant_source_ = 0.0;

        ObstacleSegmentation segmentation_;
        PoissonSolver solver_;

        SegmentationResult seg_result_;
        VectorField u_field_;
        Grid2D h_field_;
        Grid2D dh_dx_field_;
        Grid2D dh_dy_field_;

        bool ready_ = false;

        double bilinearSample(const Grid2D & field, double x, double y) const;
    };

}