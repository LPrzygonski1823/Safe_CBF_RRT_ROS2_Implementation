#pragma once
#include <unordered_map>
#include <vector>

namespace cbf_rrt_planner
{

    // spatial index based on a uniform grid (or spatial hashing)
    // accelerates nearest-neighbor and fixed-radius neighbor searches from O(n) (linear search of the entire tree) to approximately O(1) per query,
    // assuming a reasonably uniform spatial distribution of samples—typical of random sampling in RRT*.
    // ? alternative ? : KD-tree

    class SpatialGrid
    {
        public:
            explicit SpatialGrid(double cell_size);

            void clear();
            void insert(int index, double x, double y);

            // returns all put points' indexes in 'radius' from (x,y)
            std::vector<int> queryRadius(double x, double y, double radius) const;

            // returns index of the nearest put point or (-1) if grid is empty
            int queryNearest(double x, double y) const;

        private:
            struct Entry
            {
                int index;
                double x;
                double y;
            };

            double cell_size_;
            std::unordered_map<long long, std::vector<Entry>> buckets_;

            int cellCoord(double value) const;
            long long cellKey(int cx, int cy) const;
    };

}