//
//  scanContextDescriptorUtils.hpp
//
//  This file contains utilities used by scanContextDescriptor.
//
//  Copyright 2020 The MathWorks, Inc.

#include <cmath>      // isfinite hypot
#define _USE_MATH_DEFINES
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <vector>
#include <algorithm>  // fill_n

#ifdef COMPILE_FOR_VISION_BUILTINS
#include "mfl_scalar/constants.hpp" // getNaN

#include "tbb/concurrent_vector.h"
#include "tbb/parallel_for.h"
#endif

namespace vision {
    namespace pc {
///////////////////////////////////////////////////////////////////////////
// Compute maximum radius using spatial extent of points
///////////////////////////////////////////////////////////////////////////
        template <typename T>
                T computeMaxRadius(const T* xyzPoints, const size_t numPoints, const T* sensorOrigin) {
            
            T maxRadius(0.0);
            const T xOrigin(sensorOrigin[0]), yOrigin(sensorOrigin[1]);
            
            const size_t xOffset(0), yOffset(numPoints);
            for (size_t idx = 0; idx < numPoints; idx++) {
                T x = xyzPoints[idx + xOffset];
                T y = xyzPoints[idx + yOffset];
                
                if (std::isfinite(x) && std::isfinite(y)) {
                    T radius = hypot(x-xOrigin, y-yOrigin);
                    maxRadius = std::max(radius, maxRadius);
                }
            }
            
            return maxRadius;
        }
        
///////////////////////////////////////////////////////////////////////////
// Compute scan context descriptor
///////////////////////////////////////////////////////////////////////////
        template <typename T>
                void scanContextImageDescriptor(
                const T* xyzPoints,
                const size_t numPoints,
                const size_t numRadialBins,
                const size_t numAzimuthalBins,
                const size_t minPointsPerBin,
                const T* sensorOrigin,
                T* radialRange,
                T* descriptor) {
            
            const auto totalNumBins = numRadialBins * numAzimuthalBins;
            
            if (numPoints==0) {
                std::fill_n(descriptor, totalNumBins, NAN);
                return;
            }
            
            // If a max radius was not provided, compute it from data
            if (!std::isfinite(radialRange[1])) {
                radialRange[1] = computeMaxRadius(xyzPoints, numPoints, sensorOrigin);
            }
            
            // Determine spatial extent of a bin
            const T ringSize = static_cast<T>((double(radialRange[1])-double(radialRange[0])) / double(numRadialBins));
            const T sectorSize = static_cast<T>(2.0 * (T)M_PI / double(numAzimuthalBins));
            
            // Initialize a vector of vectors.
            // Each element indexed by the linear bin index, holds a
            // vector of point heights.
#ifdef COMPILE_FOR_VISION_BUILTINS
            tbb::concurrent_vector<tbb::concurrent_vector<T>> binnedHeights(totalNumBins);
            binnedHeights.reserve(totalNumBins);
            
            // Bin point heights (Z) to their respective bins
            tbb::parallel_for(tbb::blocked_range<size_t>(0, numPoints),
                    [&xyzPoints, &binnedHeights, sensorOrigin, radialRange, ringSize, numRadialBins, sectorSize, numAzimuthalBins, numPoints]
                    (const tbb::blocked_range<size_t>& range) {
#else
                std::vector<std::vector<T>> binnedHeights(totalNumBins);
                binnedHeights.reserve(totalNumBins);
#endif
                // Bin point heights (Z) to their respective bins
                
                const size_t xOffset(0), yOffset(numPoints), zOffset(numPoints+numPoints);
                const T xOrigin(sensorOrigin[0]), yOrigin(sensorOrigin[1]);
                const T minRadius(radialRange[0]), maxRadius(radialRange[1]);
                const T twoPi = static_cast<T>(2.0 * (T)M_PI);
#ifdef COMPILE_FOR_VISION_BUILTINS
                for (size_t idx = range.begin(); idx < range.end(); idx++) {
#else
                    for (size_t idx = 0; idx < numPoints; idx++) {
#endif
                        T x = xyzPoints[idx + xOffset];
                        T y = xyzPoints[idx + yOffset];
                        T z = xyzPoints[idx + zOffset];
                        
                        // Check if point is valid
                        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                            continue;
                        }
                        
                        // Offset sensor origin
                        x = x - xOrigin;
                        y = y - yOrigin;
                        
                        // Compute radius
                        T radius = hypot(x, y);
                        
                        // Check if point is within radialRange
                        if ((radius < minRadius) || (radius > maxRadius)) {
                            continue;
                        }
                        
                        // Compute azimuth
                        T theta = atan2(y, x);
                        
                        // Wrap theta in [0, 2pi]
                        theta = theta < T(0) ? theta + twoPi : theta;
                        
                        // Compute radial and azimuthal bin indices
                        size_t ridx = std::min(size_t((radius - minRadius)/ringSize), numRadialBins-1);
                        size_t tidx = std::min(size_t(theta/sectorSize), numAzimuthalBins-1);
                        
                        // Convert to linear index
                        auto linearIndex = tidx * numRadialBins + ridx;
                        
                        // Append point height to bin
                        binnedHeights[linearIndex].push_back( z );
                    }
#ifdef COMPILE_FOR_VISION_BUILTINS
                });
#endif
                // Compute the maximum point height in each bin if there are sufficient points
#ifdef COMPILE_FOR_VISION_BUILTINS
                const auto NaN = mfl_scalar::getNaN<T>();
                tbb::parallel_for(tbb::blocked_range<size_t>(0, totalNumBins),
                        [&binnedHeights, &descriptor, minPointsPerBin, NaN]
                        (const tbb::blocked_range<size_t>& range) {
                    for (size_t idx = range.begin(); idx < range.end(); idx++) {
#else
                        for (size_t idx = 0; idx < totalNumBins; idx++) {
#endif
                            const auto& binHeights = binnedHeights[idx];
                            
                            if (binHeights.size() >= minPointsPerBin) {
                                descriptor[idx] = *std::max_element(binHeights.begin(), binHeights.end());
                            } else {
                                descriptor[idx] = NAN;
                            }
                        }
#ifdef COMPILE_FOR_VISION_BUILTINS
                    });
#endif
                }
            } // namespace pc
        } // namespace vision
