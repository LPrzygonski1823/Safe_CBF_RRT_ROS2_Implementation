#pragma once
#include <vector>

//representation of grids (used in psf etc.)
namespace cbf_rrt_planner
{
    // double grid
    class Grid2D
    {
        public:
            Grid2D() = default;
            Grid2D(int rows, int cols, double fill = 0.0) : rows_(rows), cols_(cols), data_(static_cast<size_t>(rows)*cols, fill) {
            }

            int rows() const {return rows_;}
            int cols() const {return cols_;}

            double operator()(int r, int c) const {
                return data_[idx(r, c)];
            }
            double & operator()(int r, int c) {
                return data_[idx(r, c)];
            }

        private:
            // 2D to vector
            size_t idx(int r, int c) const {
                return static_cast<size_t>(r) * cols_ + c;
            }

            int rows_ = 0;
            int cols_ = 0;
            std::vector<double> data_;
    };

    // int grid
    class GridI
    {
        public:
            GridI() = default;
            GridI(int rows, int cols, int fill = 0) : rows_(rows), cols_(cols), data_(static_cast<size_t>(rows) * cols, fill){
            }

            int rows() const {return rows_;}
            int cols() const {return cols_;}

            int operator()(int r, int c) const {
                return data_[idx(r, c)];
            }
            int & operator()(int r, int c) {
                return data_[idx(r, c)];
            }

        private:
            size_t idx(int r, int c) const {
                return static_cast<size_t>(r) * cols_ + c;
            }

            int rows_ = 0;
            int cols_ = 0;
            std::vector<int> data_;
    };

}