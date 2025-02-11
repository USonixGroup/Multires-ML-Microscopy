///////////////////////////////////////////////////////////////////////////////
// HybridSearchLoopImpl implements a functor for use with tbb::parallel_for. It
// calls hybridSearch from Kdtree
// Copyright 2023-2024 The MathWorks, Inc.
///////////////////////////////////////////////////////////////////////////////

#ifndef _vision_hybridsearchloopimpl_hpp_
#define _vision_hybridsearchloopimpl_hpp_
#include "KdtreeAdaptor.hpp" // includes "nanoflann.hpp"
#include <vector>
#ifdef COMPILE_FOR_VISION_BUILTINS
#include <tbb/blocked_range.h>
#endif

namespace vision {
/**
 * \class HybridSearchLoopImpl
 *
 * \tparam  DistanceMetric
 * \tparam  DataType
 * \tparam  RowMajorAccess
 */
template <typename DistanceMetric, typename DataType, bool RowMajorAccess = false>
class HybridSearchLoopImpl {
  public:
    ///////////////////////////////////////////////////////////////////////////////
    // Constructor - setup input and output buffers
    ///////////////////////////////////////////////////////////////////////////////

    HybridSearchLoopImpl(
        std::shared_ptr<nanoflann::KdtreeAdaptor<DataType, uint32_t, -1, DistanceMetric, RowMajorAccess>> index,
        DataType* queryData,
        uint32_t numQueries,
        uint32_t numQueryDims,
        uint32_t K,
        const double radius,
        void* indices,
        void* metric,
        void* valid,
        nanoflann::SearchParameters& params) {

        mIndex = index;
        mKnn = (size_t)K;
        mRadius = (DataType)(radius * radius);
        mNumNeighbors = static_cast<uint32_t>(K);
        mNumQueries = static_cast<uint32_t>(numQueries);
        mFeatureLength = static_cast<uint32_t>(numQueryDims);

        // Input data
        mQueryData = static_cast<DataType*>(queryData);

        // Output data
        mIndices = static_cast<uint32_t*>(indices);
        mMetric = static_cast<DataType*>(metric);
        mValid = static_cast<uint32_t*>(valid);

        // Search params
        mParams = params;
    }

    /////////////////////////////////////////////////////////////////
    // Loop body for tbb::parallel_for. Performs radius search for
    // a subset of query points and updates the output buffers with
    // nearest neighbors index and their distances from query points.
    /////////////////////////////////////////////////////////////////
#ifdef COMPILE_FOR_VISION_BUILTINS
    void operator()(const tbb::blocked_range<int>& range) const {
        const int startIdx = range.begin();
        const int numFeatures = static_cast<int>(range.size());
#else
    void operator()(const uint32_t startIdx, const uint32_t numFeatures) const {
#endif
        // Get offset into output buffers
        uint32_t* indices = mIndices + startIdx * mNumNeighbors;
        DataType* metric = mMetric + startIdx * mNumNeighbors;
        uint32_t* valid = mValid + startIdx;

        for (uint32_t n = 0; n < (uint32_t)numFeatures; n++) {
            // If data is not indexed at all, do nothing
            if (mIndex->index_->root_node_ == nullptr) {
                valid[n] = static_cast<uint32_t>(0);
            } else {
                std::vector<uint32_t> indices_eigen(mNumNeighbors);
                std::vector<DataType> resultSqrDist(mNumNeighbors);

                bool isQueryPointValid = true;
                std::vector<DataType> queryFeature(mFeatureLength);
                for (uint32_t m = 0; m < mFeatureLength; m++) {
                    queryFeature[m] = mQueryData[m * mNumQueries + n + startIdx];
                    if (std::isinf(queryFeature[m]) || std::isnan(queryFeature[m])) {
                        isQueryPointValid = false;
                        break;
                    }
                }

                if (isQueryPointValid) {
                    std::size_t numReturn = (std::size_t)mIndex->index_->knnSearch(
                        queryFeature.data(), mNumNeighbors, indices_eigen.data(),
                        resultSqrDist.data());

                    std::size_t numValid = (std::size_t)(std::distance(
                        resultSqrDist.begin(),
                        std::lower_bound(resultSqrDist.begin(), resultSqrDist.begin() + numReturn,
                                         mRadius)));

                    std::size_t offset = n * mNumNeighbors;
                    for (std::size_t k = 0; k < numValid; k++) {
                        indices[offset + k] =
                            static_cast<uint32_t>(indices_eigen[k] + 1); // shift to one-based index
                        metric[offset + k] = resultSqrDist[k];
                    }

                    valid[n] = static_cast<uint32_t>(numValid);
                } else {
                    valid[n] = static_cast<uint32_t>(0);
                }
            }
        }
    }

  private:
    std::shared_ptr<nanoflann::KdtreeAdaptor<DataType, uint32_t, -1, DistanceMetric, RowMajorAccess>>
        mIndex;

    DataType* mQueryData;
    std::uint32_t mFeatureLength;
    std::uint32_t mNumNeighbors;
    std::uint32_t mNumQueries;

    std::size_t mKnn;
    DataType mRadius;

    // Output data
    DataType* mMetric;
    std::uint32_t* mIndices;
    std::uint32_t* mValid;

    nanoflann::SearchParameters mParams{};
};
} // namespace vision

#endif