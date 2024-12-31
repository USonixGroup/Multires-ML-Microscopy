//
//  labelRangeDataGroundRemovalUtils.hpp
//  libmwvision
//
//  This file contains utilities used by visionLabelRangeDataGroundRemoval.
//
//
//  Copyright 2019-2020 The MathWorks, Inc.

#include <cmath> // std::abs
#include <algorithm>
#include <functional>
#define _USE_MATH_DEFINES
#include <math.h>
#include <limits>
#include <queue>
#include <cstring>

#ifdef COMPILE_FOR_VISION_BUILTINS
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#endif

struct depthPixel {
    int r, c;
    depthPixel(int r0, int c0) : r(r0), c(c0)
    {}
};

//==============================================================================
// Label one connected component from the seed position. The neighbors are points
// from the top, bottom, left, and right.
//==============================================================================
template <typename T>
        void floodFill(T* range, const int nRows, const int nCols, bool* labels, depthPixel& seed, const T angle_threshold)
{
    static int shifts_r[4] = { -1, 1, 0, 0 };
    static int shifts_c[4] = { 0, 0, -1, 1 };
    
    T* angleMatrix = range + 2 * nRows * nCols;
    
    // use queue for breadth first search
    std::queue<depthPixel> labelPixelQueue;
    labelPixelQueue.push(seed);
    
    while (!labelPixelQueue.empty()) {
        // copy the current coordinate
        const depthPixel current = labelPixelQueue.front();
        labelPixelQueue.pop();
        
        int index = current.c * nRows + current.r;
        if (labels[index]) {
            // skip if we have already labeled this point
            continue;
        }
        
        if (!std::isfinite(range[index]) || !std::isfinite(angleMatrix[index])) {
            // range is invalid
            continue;
        }
        
        // set the label of this point to current label
        labels[index] = true;
        
        for (int k = 0; k < 4; ++k) {
            int r = current.r + shifts_r[k];
            if (r < 0 || r >= nRows) {
                continue;
            }
            // wrap around if crossing the column border
            int c = current.c + shifts_c[k];
            if (c < 0) {
                c = c + nCols;
            }
            else if (c >= nCols) {
                c = c - nCols;
            }
            
            int index2 = c * nRows + r;
            if (labels[index2]) {
                // skip we have already labeled this point
                continue;
            }
            
            if (!std::isfinite(range[index2]) || !std::isfinite(angleMatrix[index2])) {
                // range is invalid
                continue;
            }
            
            T a = angleMatrix[current.c * nRows + current.r];
            T b = angleMatrix[c * nRows + r];
            T diff = static_cast<T>(fabs(a - b));
            if (diff < angle_threshold) {
                depthPixel p(r, c);
                labelPixelQueue.push(p);
            }
        }
    }
}

template <typename T>
        void computeAngleMatrix(T* range, const int nRows, const int nCols)
{
    T* pitch = range + nRows * nCols;
    T* angleMatrix = pitch + nRows * nCols;
    for (int i = 0; i < nRows - 1; ++i) {
        for (int j = 0; j < nCols; ++j) {
            int k = j * nRows + i;
            if (!std::isfinite(range[k + 1]) || !std::isfinite(range[k]) || !std::isfinite(pitch[k]) || !std::isfinite(pitch[k + 1]))
                angleMatrix[k + 1] = std::numeric_limits<T>::quiet_NaN();
            else {
                T deltaZ = fabs(range[k + 1] * sin(pitch[k + 1]) - range[k] * sin(pitch[k]));
                T deltaX = fabs(range[k + 1] * cos(pitch[k + 1]) - range[k] * cos(pitch[k]));
                angleMatrix[k + 1] = atan2(deltaZ, deltaX);
            }
        }
    }
    // The first row is unknown. We initialize it with the value from the second row
    
    for (int j = 0; j < nCols; ++j) {
        angleMatrix[j * nRows] = angleMatrix[j * nRows + 1];
    }
    
}
template <typename T>
        inline void repairRangeColumnwise(const int begin, const int end, const T* range, const int nRows, T* repairedRange,
        const T repairDepthThreshold )
{
    const int stepSize = 5;
    for (int c = begin; c < end; ++c) {
        for (int r = 0; r < nRows; ++r) {
            // check if it is a hole and need to interpolate
            if (!std::isfinite(range[c * nRows + r])) {
                int	counter = 0;
                T sum = 0;
                for (int i = 1; i <= stepSize; ++i) {
                    if (r - i < 0)
                        continue;
                    
                    T prev = repairedRange[c * nRows + r - i];
                    if (!std::isfinite(prev))
                        continue;
                    
                    for (int j = 1; j <= stepSize; ++j) {
                        if (r + j >= nRows)
                            continue;
                        
                        T next = repairedRange[c * nRows + r + j];
                        if (!std::isfinite(next))
                            continue;
                        
                        // needs to be small enough to interpolate, otherwise too much gap
                        if (fabs(prev - next) < repairDepthThreshold) {
                            sum = sum + prev + next;
                            counter = counter + 2;
                        }
                    }
                }
                
                if (counter > 0) {
                    repairedRange[c * nRows + r] = sum / counter;
                }
            }
        }
    }
}

template <typename T>
        inline void repairPitchRowwise(const int begin, const int end, T* repairPitch,
        const int nRows, const int nCols)
{
    for (int r = begin; r < end; ++r) {
        
        for (int c = 0; c < nCols; ++c) {
            // fix pitch if needed
            if (!std::isfinite(repairPitch[c * nRows + r])) {
                int k1 = -1;
                int k2 = -1;
                for (int m = 1; m < nCols; m++) {
                    if (c - m < 0)
                        break;
                    if (std::isfinite(repairPitch[(c - m) * nRows + r])) {
                        k1 = c - m;
                        break;
                    }
                }
                for (int m = 1; m < nCols; m++) {
                    if (c + m >= nCols)
                        break;
                    if (std::isfinite(repairPitch[(c + m) * nRows + r])) {
                        k2 = c + m;
                        break;
                    }
                }
                
                if (k1 >= 0 && k2 >= 0)
                    repairPitch[c * nRows + r] = repairPitch[k1 * nRows + r] * (k2 - c) / (k2 - k1)
                    + repairPitch[k2 * nRows + r] * (c - k1) / (k2 - k1);
                else if (k1 >= 0 && k2 < 0)
                    repairPitch[c * nRows + r] = repairPitch[k1 * nRows + r];
                else if (k2 >= 0 && k1 < 0)
                    repairPitch[c * nRows + r] = repairPitch[k2 * nRows + r];
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// This function repairs holes in the range/pitch data, compute and store
// angle matrix. range is MxNx3 matrix that stores range and pitch in its first
// two planes. The angle matrix is returned in the third plane.
////////////////////////////////////////////////////////////////////////////////
template <typename T>
        void preprocessRangeData(const T* range, const int nRows, const int nCols, T* repairedRange,
        const T repairDepthThreshold)
{
       T* repairPitch = repairedRange + nRows * nCols;
    
    // The operation is column - wise indpendently from adjacent columns     
#ifdef COMPILE_FOR_VISION_BUILTINS
         // Repair range column at a time
    tbb::parallel_for( tbb::blocked_range<int>(0,nCols), // range
            [&](tbb::blocked_range<int> r) // lambda begin
    {
        repairRangeColumnwise(r.begin(), r.end(), range, nRows, repairedRange,
                repairDepthThreshold);
    } // end lambda
    );
    // Repair pitch row at a time
    tbb::parallel_for( tbb::blocked_range<int>(0,nRows),// range
            [&](tbb::blocked_range<int> rr)  // lambda begin
    {
        repairPitchRowwise(rr.begin(), rr.end(), repairPitch,
                nRows, nCols);
    }// end lambda
    );
    
#else
    repairRangeColumnwise(0, nCols, range, nRows, repairedRange,
            repairDepthThreshold);
    repairPitchRowwise(0, nRows, repairPitch, nRows, nCols);
    
#endif
    
// store the angle matrix in the last plane
    computeAngleMatrix(repairedRange, nRows, nCols);
}
template <typename T>
        inline void segmentGroundColumnwise(int begin, int end, T* angleMatrix, T* range,
        const int nRows, const int nCols, bool* labels, const T seed_threshold, const T angle_threshold)
{
    for (int col = begin; col < end; ++col) {
        // start at bottom, need to initialize for every column
        // because the bottom row is different for each column
        int row = nRows - 1;
        // check if the range is valid, aka not nan
        while (row >= 0 && !std::isfinite(range[col * nRows + row]) ) {
            --row;
        }
        // g1873714 fix, to constrain using index < 0
        int index = col* nRows + row;
        if(index < 0){
            // skip, if we have surpassed bottom of the image
            continue;
        }
        //check if the seed angle value is bigger than the seed_threshold
        if (angleMatrix[index] > seed_threshold) {
            continue;
        }
        // search one component for the current label
        depthPixel p(row, col);
        floodFill(range, nRows, nCols, labels, p, angle_threshold);
    }
}

template <typename T>
        void segmentGround(T* range, const int nRows, const int nCols, bool* labels, const T seed_threshold, const T angle_threshold)
{
    T* angleMatrix = range + 2 * nRows * nCols;
#ifdef COMPILE_FOR_VISION_BUILTINS
    tbb::parallel_for( tbb::blocked_range<int>(0,nCols), //range
            [=](tbb::blocked_range<int> r) // lambda begin
    {
        segmentGroundColumnwise(r.begin(), r.end(), angleMatrix, range,
                nRows, nCols, labels, seed_threshold, angle_threshold);
    } // lambda end
    );
#else
    segmentGroundColumnwise(0, nCols, angleMatrix, range, nRows, nCols,
            labels,  seed_threshold, angle_threshold);
#endif
}
