////////////////////////////////////////////////////////////////////////////////
//    KdtreeImpl class
//    Copyright 2018-2024 The MathWorks, Inc.
//////////////////////////////////////////////////////////////////////////////
 
#ifndef KdtreeImpl_hpp
#define KdtreeImpl_hpp
 
// known issue with Eigen and C++17
#ifndef _SILENCE_CXX17_NEGATORS_DEPRECATION_WARNING
#define _SILENCE_CXX17_NEGATORS_DEPRECATION_WARNING
#endif
#ifndef _SILENCE_CXX17_ADAPTOR_TYPEDEFS_DEPRECATION_WARNING
#define _SILENCE_CXX17_ADAPTOR_TYPEDEFS_DEPRECATION_WARNING
#endif
 
#ifdef _WIN64
#pragma warning(push)
 
// '=' : conversion from 'size_t' to 'float', possible loss of data
#pragma warning(disable : 4244)
#pragma warning(disable : 4996)
// 'initializing': conversion from 'size_t' to 'int', possible loss of data
#pragma warning(disable : 4267) 
#pragma warning(disable : 4189)
#pragma warning(disable : 4100)
#pragma warning(disable : 4456)
#pragma warning(disable : 4127)
//  warning C4068: unknown pragma 'diag_suppress' from Eigen
#pragma warning(disable : 4068)
#endif

#include "KdtreeAdaptor.hpp" // includes "nanoflann.hpp"
#include "KnnSearchLoopImpl.hpp"
#include "RadiusSearchLoopImpl.hpp"
#include "HybridSearchLoopImpl.hpp"
#include "KdtreeUtil.hpp"

#ifdef COMPILE_FOR_VISION_BUILTINS
// tbb includes for simulation and sharedLib codegen
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#endif

namespace vision {
/**
 * \class KdtreeImpl
 */
template <typename DistanceMetric, typename DataType, bool RowMajorAccess = false>
class KdtreeImpl {
  public:
    ///////////////////////////////////////////////////////////////////////////////
    // Constructor
    ///////////////////////////////////////////////////////////////////////////////
    KdtreeImpl() {}

    ///////////////////////////////////////////////////////////////////////
    // Destructor
    ///////////////////////////////////////////////////////////////////////
    ~KdtreeImpl() {
        if (nanoflann_index_) {
            nanoflann_index_ = NULL;
        }
    }

    ///////////////////////////////////////////////////////////////////////////////
    // Index a set of data
    ///////////////////////////////////////////////////////////////////////////////
    void index(DataType* pData = NULL,
               uint32_t dataSize = 0,
               uint32_t dims = 0,
               std::size_t bucketSize = 10) {
        buildIndex(pData, dataSize, dims, bucketSize);
    }

    ///////////////////////////////////////////////////////////////////////////////
    // Index data using a KD-Tree for single/double data
    ///////////////////////////////////////////////////////////////////////////////
    void buildIndex(DataType* pData = NULL,
                    uint32_t dataSize = 0,
                    uint32_t dims = 0,
                    std::size_t bucketSize = 10) {
        mNumData = dataSize;
        mDims = dims;

        dataset_size_ = (size_t)dataSize;
        dimension_ = (size_t)dims;

        saveAddressOfIndexedData(pData);

        if (dimension_ == 0 || dataset_size_ == 0) {
            // TO FIX: Error out
            // return false;
        }

        nanoflann_index_.reset(
            new nanoflann::KdtreeAdaptor<DataType, uint32_t, -1, DistanceMetric, RowMajorAccess>(
                dimension_, (DataType*)(pData), dataset_size_, dimension_, (int)(bucketSize)));

        nanoflann_index_->index_->buildIndex();

        mIsIndexed = true;
        if (nanoflann_index_->index_->root_node_ == nullptr) {
            mIsIndexed = false;
        }   
    }

    ///////////////////////////////////////////////////////////////////////////////
    // needsReIndex a set of data
    ///////////////////////////////////////////////////////////////////////////////
    bool needsReindex(DataType* pData = NULL) {
        return !mIsIndexed || (mIndexedData != pData);
    }

    //////////////////////////////////////////////////////////////////////////////
    //  Checks indexed data and reindexes if needed. This is to avoid
    //   the indexed data changing from underneath the index
    ///////////////////////////////////////////////////////////////////////////////
    void checkIndex(DataType* pData = NULL,
                    uint32_t dataSize = 0,
                    uint32_t dims = 0,
                    std::size_t bucketSize = 10) {
        if (!mIsIndexed || needsReindex(pData)) {
            index(pData, dataSize, dims, bucketSize);
        }
    }

    ///////////////////////////////////////////////////////////////////////////////
    // boxSearch a set of data
    ///////////////////////////////////////////////////////////////////////////////
    size_t boxSearch(void* roi, void** indices) {
        VISION_ASSERT_MSG(mIsIndexed, "Internal index tree error, tree is not Indexed");
        return invokeBoxSearch(roi, indices);
    }

    ///////////////////////////////////////////////////////////////////////////////
    // knnSearch a set of data
    ///////////////////////////////////////////////////////////////////////////////
    int32_t knnSearch(DataType* queryData,
                      uint32_t numQueries,
                      uint32_t numQueryDims,
                      uint32_t knn,
                      DataType paramEps,
                      void* indices,
                      void* dists,
                      void* valid,
                      int32_t grainSize = 2000,
                      uint32_t tbbQueryThreshold = 500) {
        return invokeKNNSearch(queryData, numQueries, numQueryDims, knn, paramEps,
                               indices, dists, valid, grainSize, tbbQueryThreshold);
    }

    ///////////////////////////////////////////////////////////////////////////////
    // radiusSearch a set of data
    ///////////////////////////////////////////////////////////////////////////////
    int32_t radiusSearch(DataType* queryData,
                         uint32_t numQueries,
                         uint32_t numQueryDims,
                         double radius,
                         DataType paramEps,
                         void** indices,
                         void** dists,
                         void* valid,
                         int32_t grainSize = 1000,
                         uint32_t tbbQueryThreshold = 500) {
        return invokeRadiusSearch(queryData, numQueries, numQueryDims, radius, paramEps,
                                  indices, dists, valid, grainSize, tbbQueryThreshold);
    }

    ///////////////////////////////////////////////////////////////////////////////
    // hybridSearch a set of data
    ///////////////////////////////////////////////////////////////////////////////
    int32_t hybridSearch(DataType* queryData,
                      uint32_t numQueries,
                      uint32_t numQueryDims,
                      uint32_t knn,
                      double radius,
                      DataType paramEps,
                      void* indices,
                      void* dists,
                      void* valid,
                      int32_t grainSize = 2000,
                      uint32_t tbbQueryThreshold = 500) {
        return invokeHybridSearch(queryData, numQueries, numQueryDims, knn, radius, paramEps,
                               indices, dists, valid, grainSize, tbbQueryThreshold);
    }    

    ///////////////////////////////////////////////////////////////////////////////
    // get value of number of dimensions
    ///////////////////////////////////////////////////////////////////////////////
    uint32_t getDims() {
        return mDims;
    }


    ///////////////////////////////////////////////////////////////////////////////
    // get value size of the data
    ///////////////////////////////////////////////////////////////////////////////
    uint32_t getNumData() {
        return mNumData;
    }
    template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
    }
    ///////////////////////////////////////////////////////////////////////////////
    // get value of Index Flag
    ///////////////////////////////////////////////////////////////////////////////
    bool isIndexed() {
        return mIsIndexed;
    }

    ///////////////////////////////////////////////////////////////////////////////
    // set Index Flag to true/false
    ///////////////////////////////////////////////////////////////////////////////
    void setIndexFlag(bool flag) {
        mIsIndexed = flag;
    }

    ///////////////////////////////////////////////////////////////////////////////
    // To check whether data pointer is null or not
    ///////////////////////////////////////////////////////////////////////////////
    bool isIndexedDataExists() {
        return (mIndexedData != NULL) ? true : false;
    }

  private:
    ///////////////////////////////////////////////////////////////////////////////
    //  Save the address of the data we indexed. This is used to check whether we
    //  need to reindex the data in situations where it has somehow been moved.
    ///////////////////////////////////////////////////////////////////////////////
    void saveAddressOfIndexedData(DataType* pData) {
        mIndexedData = pData;
    }

    ///////////////////////////////////////////////////////////////////////////////
    // Find the points within a box using an exact search.
    ///////////////////////////////////////////////////////////////////////////////
    size_t invokeBoxSearch(void* roi, void** indices) {
        const std::size_t boxDims = 3;
        std::vector<DataType> minPoint(boxDims);
        std::vector<DataType> maxPoint(boxDims);        
        double* box = static_cast<double*>(roi);

        minPoint[0] = static_cast<DataType>(box[0]);
        minPoint[1] = static_cast<DataType>(box[1]);
        minPoint[2] = static_cast<DataType>(box[2]);
        maxPoint[0] = static_cast<DataType>(box[3]);
        maxPoint[1] = static_cast<DataType>(box[4]);
        maxPoint[2] = static_cast<DataType>(box[5]);

        for (size_t n = 0; n < boxDims; n++) {    
            if (!std::isfinite(minPoint[n]))
            {
                if (sgn<DataType>(minPoint[n]) > 0)
                {
                    minPoint[n] = std::numeric_limits<DataType>::max();
                } else {
                    minPoint[n] = -std::numeric_limits<DataType>::max();
                }
            }

            if (!std::isfinite(maxPoint[n])) 
            {
                if (sgn<DataType>(maxPoint[n]) > 0)                  

                {
                    maxPoint[n] = std::numeric_limits<DataType>::max();
                } else {
                    maxPoint[n] = -std::numeric_limits<DataType>::max();
                }
            }
        }
 #ifdef COMPILE_FOR_VISION_BUILTINS
        VISION_ASSERT_MSG(
            (maxPoint[0] >= minPoint[0] && maxPoint[1] >= minPoint[1] && maxPoint[2] >= minPoint[2]),
            "Invalid ROI");   
             #endif

        // create Indices container
        std::vector<std::size_t>* ptrIndices = new std::vector<std::size_t>();
        *indices = (void*)ptrIndices;

        // box Search
        std::vector<std::size_t>& refIndices = *ptrIndices;
        nanoflann_index_->index_->boxSearch(&minPoint[0],&maxPoint[0], boxDims, refIndices);
        
        return static_cast<size_t>(refIndices.size());
    }

    ///////////////////////////////////////////////////////////////////////////////
    // Find the K nearest neighbors using an approximate search.
    ///////////////////////////////////////////////////////////////////////////////
    int32_t invokeKNNSearch(DataType* queryData,
                            uint32_t numQueries,
                            uint32_t numQueryDims,
                            uint32_t knn,
                            DataType paramEps,
                            void* indices,
                            void* dists,
                            void* valid,
                            int32_t grainSize = 2000,
                            uint32_t tbbQueryThreshold = 500) {
        VISION_ASSERT_MSG(knn > 0, "Invalid Input, knn should be greater than zero");

        nanoflann::SearchParameters params{(float)(paramEps),true};

        if (queryData != NULL) {
            KNNSearchLoopImpl<DistanceMetric, DataType, RowMajorAccess> knnSearcher(
                nanoflann_index_, queryData, numQueries, numQueryDims, knn, indices, dists, valid, params);
            #ifdef COMPILE_FOR_VISION_BUILTINS
            // TBB is triggered when the number of queries
            // exceeds tbbThreshold (default = 500)

            // Construct knnSearch functor for use with parallel_for
            if (grainSize > 0 && numQueries >= tbbQueryThreshold) {
                tbb::blocked_range<int32_t> range(0, numQueries, grainSize);
                tbb::parallel_for(range, knnSearcher);
            } else {
                tbb::blocked_range<int32_t> range(0, numQueries);
                knnSearcher(range);
            }
            #else
            knnSearcher(0, numQueries);
            #endif
            
        }
        return 0;
    }

    ///////////////////////////////////////////////////////////////////////////////
    // Find the points within radius search.
    ///////////////////////////////////////////////////////////////////////////////
    int32_t invokeRadiusSearch(DataType* queryData,
                               uint32_t numQueries,
                               uint32_t numQueryDims,
                               double radius,
                               DataType paramEps,
                               void** indices,
                               void** dists,
                               void* valid,
                               int32_t grainSize = 1000,
                               uint32_t tbbQueryThreshold = 500) {
        // create Indices container
        std::vector<std::vector<uint32_t>>* ptrIndices = new std::vector<std::vector<uint32_t>>();
        *indices = (void*)ptrIndices;

        // create dists container
        std::vector<std::vector<DataType>>* ptrDists = new std::vector<std::vector<DataType>>();
        *dists = (void*)ptrDists;

        // radius Search
        std::vector<std::vector<uint32_t>>& refIndices = *ptrIndices;
        std::vector<std::vector<DataType>>& refDists = *ptrDists;

        // resize indices & distances vector with size of numQueries
        refIndices.resize(numQueries);
        refDists.resize(numQueries);

        nanoflann::SearchParameters params{(float)(paramEps),true};

        if (queryData != NULL) {
            RadiusSearchLoopImpl<DistanceMetric, DataType, RowMajorAccess> radiusSearcher(
                nanoflann_index_, refIndices, refDists, (uint32_t*)valid, queryData, numQueries, numQueryDims, radius, params);           
#ifdef COMPILE_FOR_VISION_BUILTINS
            // TBB is triggered when the number of queries
            // exceeds tbbThreshold (default = 500)

            // Construct radiusSearch functor for use with parallel_for
            if (grainSize > 0 && numQueries >= tbbQueryThreshold) {
                tbb::blocked_range<size_t> range(0, numQueries, grainSize);
                tbb::parallel_for(range, radiusSearcher);
            } else {
                tbb::blocked_range<size_t> range(0, numQueries);
                radiusSearcher(range);
            }
            #else
            radiusSearcher(0, numQueries);
            #endif
        }
            return 0;
        }

    ///////////////////////////////////////////////////////////////////////////////
    // Find the points within radius search.
    ///////////////////////////////////////////////////////////////////////////////
    int32_t invokeHybridSearch(DataType* queryData,
                               uint32_t numQueries,
                               uint32_t numQueryDims,
                               uint32_t knn,
                               double radius,
                               DataType paramEps,
                               void* indices,
                               void* dists,
                               void* valid,
                               int32_t grainSize = 2000,
                               uint32_t tbbQueryThreshold = 500) {
        VISION_ASSERT_MSG(knn > 0, "Invalid Input, knn should be greater than zero");
        VISION_ASSERT_MSG(radius > 0, "Invalid Input, radius should be greater than zero");                                

        nanoflann::SearchParameters params{(float)(paramEps),true};

        if (queryData != NULL) {
            HybridSearchLoopImpl<DistanceMetric, DataType, RowMajorAccess> hybridSearcher(
                nanoflann_index_, queryData, numQueries, numQueryDims, knn, radius, indices, dists, valid, params);           
#ifdef COMPILE_FOR_VISION_BUILTINS
            // TBB is triggered when the number of queries
            // exceeds tbbThreshold (default = 500)

            // Construct hybridSearch functor for use with parallel_for
            if (grainSize > 0 && numQueries >= tbbQueryThreshold) {
                tbb::blocked_range<int32_t> range(0, numQueries, grainSize);
                tbb::parallel_for(range, hybridSearcher);
            } else {
                tbb::blocked_range<int32_t> range(0, numQueries);
                hybridSearcher(range);
            }
            #else
            hybridSearcher(0, numQueries);
            #endif
        }
            return 0;
        }    

  private: // data members
    std::shared_ptr<
        nanoflann::KdtreeAdaptor<DataType, uint32_t, -1, DistanceMetric, RowMajorAccess>>
        nanoflann_index_;

    size_t dimension_ = 0;
    size_t dataset_size_ = 0;
   
    DataType* mIndexedData = NULL;
    uint32_t mDims = 0;
    uint32_t mNumData = 0;
    bool mIsIndexed = false;
};

} // namespace vision
#endif