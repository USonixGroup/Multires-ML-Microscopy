#ifndef _VISIONLABELRANGEDATA_H_
#define _VISIONLABELRANGEDATA_H_

#define _USE_MATH_DEFINES
#include <cmath>
#include <queue>
#include <vector>
#include <algorithm> // for std::copy

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct depthPixel {
    int r, c;
    depthPixel(int r0, int c0)
        : r(r0)
        , c(c0) {
    }
};
//==============================================================================
// Compute beta angle to determine if two points are from the same object
//==============================================================================
template <typename T>
T compAngleDiff(const int r1, const int c1, const int r2, const int c2, const T* range, const int nRows, const int nCols) {
    T current_depth = range[c1 * nRows + r1];
    T neighbor_depth = range[c2 * nRows + r2];
    const T* pitch = range + nRows * nCols;
    const T* yaw = pitch + nRows * nCols;

    //  rowAlpha   : v_angle(i+1)-v_angle(i). The last row is zero.
    //  colAlpha   : h_angle(i+1)-h_angle(i). The last column is h_angle(1)-h_angle(end) + 2*pi
    T alpha = 0;
    if (r1 < r2) {
        alpha = pitch[c2 * nRows + r2] - pitch[c1 * nRows + r1];
    } else if (r1 > r2 && r1 < nRows - 1) {
        alpha = pitch[c1 * nRows + r1] - pitch[c2 * nRows + r2];
    } else if (r1 > r2 && r1 == nRows - 1) {
        alpha = 0;
    } else if (c1 == 0 && c2 == nCols - 1) {
        alpha = yaw[c2 * nRows + r2] - yaw[r1];
        if (alpha > (T)M_PI) {
            alpha -= 2 * (T)M_PI;
        } else if (alpha < -(T)M_PI) {
            alpha += 2 * (T)M_PI;
        } else {
            // keep alpha as is
        }
    } else if (c2 == 0 && c1 == nCols - 1) {
        alpha = yaw[r2] - yaw[c1 * nRows + r1];
        if (alpha > (T)M_PI) {
            alpha -= 2 * (T)M_PI;
        } else if (alpha < -(T)M_PI) {
            alpha += 2 * (T)M_PI;
        } else {
            // keep alpha as is
        }
    } else if (c1 < c2) {
        alpha = yaw[c2 * nRows + r2] - yaw[c1 * nRows + r1];
    } else if (c1 > c2) {
        alpha = yaw[c1 * nRows + r1] - yaw[c2 * nRows + r2];
    } else {
        alpha = 0;
    }

    T d1 = std::max(current_depth, neighbor_depth);
    T d2 = std::min(current_depth, neighbor_depth);
    T beta = std::atan2((d2 * sin(alpha)), (d1 - d2 * cos(alpha)));
    T diff = static_cast<T>(fabs(beta));

    return diff;
}
//==============================================================================
// Label one connected component from the seed position. The neighbors are points
// from the top, bottom, left, and right.
//==============================================================================
template <typename T>
bool floodFill(const T* xyz,
               const T* range,
               const int nRows,
               const int nCols,
               uint32_T* labels,
               const uint32_T label,
               const depthPixel& seed,
               const T threshold,
               const T angle_threshold,
               const uint32_T minClusterPoints,
               const uint32_T maxClusterPoints,
               std::vector<int>* invalidLabelsIdx) {
    static const int shifts_r[4] = {-1, 1, 0, 0};
    static const int shifts_c[4] = {0, 0, -1, 1};
    
    int const nSize = nRows * nCols;
    const T* x = xyz;
    const T* y = x + nSize;
    const T* z = y + nSize;
    T diff;

    // use queue for breadth first search
    std::queue<depthPixel> labelPixelQueue;
    labelPixelQueue.push(seed);
    
    // indices that were labeled
    std::vector<int> labeledPixel;


    while (!labelPixelQueue.empty()) {
        // copy the current coordinate
        const depthPixel current = labelPixelQueue.front();
        labelPixelQueue.pop();

        int index = current.c * nRows + current.r;
        if (labels[index] > 0 || !std::isfinite(xyz[index])) {
            // skip if we have already labeled this point or depth is invalid
            continue;
        }

        // set the label of this point to current label
        labels[index] = label;
        
        labeledPixel.push_back(index);

        for (int k = 0; k < 4; ++k) {
            int r = current.r + shifts_r[k];
            if (r < 0 || r >= nRows) {
                continue;
            }
            // wrap around if crossing the column border
            int c = current.c + shifts_c[k];
            if (c < 0) {
                c = c + nCols;
            } else if (c >= nCols) {
                c = c - nCols;
            }

            int index2 = c * nRows + r;
            if (labels[index2] > 0) {
                // skip we have already labeled this point
                continue;
            }

            if (!std::isfinite(xyz[index2])) {
                // depth is invalid
                continue;
            }

            diff = (x[index] - x[index2]) * (x[index] - x[index2]);
            diff += (y[index] - y[index2]) * (y[index] - y[index2]);
            diff += (z[index] - z[index2]) * (z[index] - z[index2]);
            diff = sqrt(diff);
            if (diff < threshold) {
                labelPixelQueue.push(depthPixel(r, c));
            } else {
                diff = compAngleDiff<T>(current.r, current.c, r, c, range, nRows, nCols);
                if (diff > angle_threshold) {
                    depthPixel p(r, c);
                    labelPixelQueue.push(p);
                }
            }
        }
    }
    
    // check if the number of points in the cluster is valid    
    uint32_T numPoints = (uint32_T)labeledPixel.size();
    bool isValidCluster = !((numPoints < minClusterPoints) || (numPoints > maxClusterPoints));
    
    if (!isValidCluster) {
        // keep track of invalid label indices to remove
        size_t initialSize = invalidLabelsIdx->size();
        invalidLabelsIdx->resize(initialSize + numPoints);
        std::copy(labeledPixel.begin(), labeledPixel.end(), invalidLabelsIdx->begin() + initialSize);
    }
    return isValidCluster;
}

template <typename T>
void labelClusters(const T* xyz,
                   const T* range,
                   const int nRows,
                   const int nCols,
                   uint32_T* labels,
                   T* numClusters,
                   const T threshold,
                   const T angle_threshold,
                   const uint32_T minClusterPoints,
                   const uint32_T maxClusterPoints) {
    uint32_T label = 1;
    std::vector<int> invalidLabelsIdx;

    for (int row = 0; row < nRows; ++row) {
        for (int col = 0; col < nCols; ++col) {
            int index = col * nRows + row;
            if (labels[index] > 0 || !std::isfinite(xyz[index])) {
                // skip if we have already labeled this pixel or depth is invalid
                continue;
            }
            
            // search one component for the current label
            depthPixel p(row, col);
            if (floodFill<T>(xyz, range, nRows, nCols, labels, label, p, threshold, angle_threshold, minClusterPoints, maxClusterPoints, &invalidLabelsIdx)) {
                // get new label only if the previous label was used
                label++;
            }
        }
    }
    
    *numClusters = static_cast<T>(label - 1);
    
    // remove labels for invalid cluster size
    for (size_t i = 0; i < invalidLabelsIdx.size(); ++i) {
        labels[invalidLabelsIdx[i]] = 0;
    }
}

#endif /* _VISIONLABELRANGEDATA_H_ */
