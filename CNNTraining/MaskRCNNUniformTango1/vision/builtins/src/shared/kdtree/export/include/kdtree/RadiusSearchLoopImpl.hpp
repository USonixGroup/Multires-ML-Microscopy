///////////////////////////////////////////////////////////////////////////////
// RadiusSearchLoopImpl implements a functor for use with tbb::parallel_for. It
// calls radiusSearch from Kdtree
// Copyright 2019-2024 The MathWorks, Inc.
///////////////////////////////////////////////////////////////////////////////

#ifndef _vision_radiussearchloopimpl_hpp__
#define _vision_radiussearchloopimpl_hpp__
#include "KdtreeAdaptor.hpp" // includes "nanoflann.hpp"
#ifdef COMPILE_FOR_VISION_BUILTINS
#include <tbb/blocked_range.h>
#endif

namespace vision {
/**
 * \class RadiusSearchLoopImpl
 *
 * \tparam  DistanceMetric
 * \tparam  DataType
 * \tparam  RowMajorAccess
 */
template <typename DistanceMetric, typename DataType, bool RowMajorAccess = false>
class RadiusSearchLoopImpl {
  public:
    ///////////////////////////////////////////////////////////////////////////////
    // Constructor - setup input and output buffers
    ///////////////////////////////////////////////////////////////////////////////
    RadiusSearchLoopImpl(
        std::shared_ptr<nanoflann::KdtreeAdaptor<DataType, uint32_t, -1, DistanceMetric, RowMajorAccess>> index,
        std::vector<std::vector<uint32_t>>& neighInd,
        std::vector<std::vector<DataType>>& neighDis,
        uint32_t* validInds,
        DataType* queryData,
        const size_t& numQueries,
        const size_t& numQueryDims,
        const double& radius,
        nanoflann::SearchParameters& params) {
        mFeatureLength = numQueryDims;
        mValid = validInds;
        mNumQueries = numQueries;
        mQueryData = queryData;

        mRadius = (DataType)radius * radius;
        mIndices = &neighInd;
        mMetric = &neighDis;
        mParams = params;
        mIndex = index;
    }

/////////////////////////////////////////////////////////////////
// Loop body for tbb::parallel_for. Performs radius search for
// a subset of query points and updates the output buffers with
// nearest neighbors index and their distances from query points.
/////////////////////////////////////////////////////////////////
#ifdef COMPILE_FOR_VISION_BUILTINS
    void operator()(const tbb::blocked_range<size_t>& range) const {
        const size_t startIdx = range.begin();
        const size_t endIdx = range.end();
#else
    void operator()(const uint32_t startIdx, const uint32_t endIdx) const {
#endif


        std::vector<nanoflann::ResultItem<uint32_t, DataType>> indices_dists;

        for (size_t n = startIdx; n < endIdx; n++) {
            std::vector<uint32_t> resultIndices;
            std::vector<DataType>& resultSqrDist = (*mMetric)[n];

            // If data is not indexed at all, do nothing
            if (mIndex->index_->root_node_ == nullptr) {
                mValid[n] = static_cast<uint32_t>(0);
            } else {
                bool isQueryPointValid = true;
                std::vector<DataType> queryFeature(mFeatureLength);
                for (size_t m = 0; m < mFeatureLength; m++) {
                    queryFeature[m] = mQueryData[m * mNumQueries + n];
                    if(std::isinf(queryFeature[m]) || std::isnan(queryFeature[m])){
                        isQueryPointValid = false;
                        break;
                    }
                }

                if(isQueryPointValid){
                    size_t retValidIndices = (size_t)mIndex->index_->radiusSearch(
                        queryFeature.data(), mRadius, indices_dists, mParams);

                    size_t retValidIndicesTemp = indices_dists.size();

                    mValid[n] = static_cast<uint32_t>(retValidIndicesTemp);
                    (*mIndices)[n].resize(retValidIndicesTemp);
                    (*mMetric)[n].resize(retValidIndicesTemp);

                    for (std::size_t k = 0; k < retValidIndicesTemp; k++) {
                        (*mIndices)[n][k] = static_cast<uint32_t>(indices_dists[k].first + 1);
                        (*mMetric)[n][k] = static_cast<DataType>(indices_dists[k].second);
                    }
                }
                else{
                    mValid[n] = static_cast<uint32_t>(0);
                }
            }
        }
    }

  private:
    std::shared_ptr<nanoflann::KdtreeAdaptor<DataType, uint32_t, -1, DistanceMetric, RowMajorAccess>>
        mIndex;
    nanoflann::SearchParameters mParams{};
    std::vector<std::vector<uint32_t>>* mIndices;
    std::vector<std::vector<DataType>>* mMetric;

    size_t mFeatureLength;
    size_t mNumQueries;
    DataType* mQueryData;
    DataType mRadius;
    uint32_t* mValid;
};
} // namespace vision

#endif
