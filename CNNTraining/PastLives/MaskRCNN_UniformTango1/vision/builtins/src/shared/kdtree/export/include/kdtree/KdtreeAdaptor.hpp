///////////////////////////////////////////////////////////////////
// KdtreeAdaptor implements an adaptor to work directly with data
// pointers and different distance metrics. 
// Copyright 2024 The MathWorks, Inc.
///////////////////////////////////////////////////////////////////

#pragma once

#ifndef DISABLE_MULTITHREADED_INDEXING
#define DISABLE_MULTITHREADED_INDEXING 1
#endif

#include <memory>
#include "nanoflann.hpp"
// For dynamic memory management functions
#include <cstdlib>

namespace nanoflann{
template <typename DataType = double,
          typename IndexType = size_t,
          int32_t DIM = -1,
          class Distance = nanoflann::metric_L2,
          bool RowMajorAccess = false>
struct KdtreeAdaptor {
    using self_t = KdtreeAdaptor<DataType, IndexType, DIM, Distance, RowMajorAccess>;
    using metric_t  = typename Distance::template traits<
        DataType, self_t, IndexType>::distance_t;
    using index_t = nanoflann::KDTreeSingleIndexAdaptor<metric_t , self_t, DIM, IndexType>;

    index_t* index_;  //! The kd-tree index for the user to call its methods as
                      //! usual with any other FLANN index.	
    
    const DataType* matrixDataPtr;
	size_t datasetSize;
	size_t datasetDimension;                      

    using Offset    = typename index_t::Offset;
    using Size      = typename index_t::Size;
    using Dimension = typename index_t::Dimension;

    /// Constructor: takes a const ref pointer of the data points
    explicit KdtreeAdaptor(Dimension dimensionality,
                           const DataType* matPtr,
                           size_t datSize,
                           size_t datDimension,
                           const int leaf_max_size = 10){
        matrixDataPtr = matPtr;
        datasetSize = datSize;
        datasetDimension = datDimension;
        const auto dims = datasetDimension;
        if (static_cast<Dimension>(dims) != dimensionality) {
            throw std::runtime_error(
                "Error: 'dimensionality' must match column count in data");
        }
        if (DIM > 0 && static_cast<int32_t>(dims) != DIM) {
            throw std::runtime_error(
                "Data set dimensionality does not match the 'DIM' template argument");
        }
        index_ = new index_t(dims, *this /* adaptor */,
                             nanoflann::KDTreeSingleIndexAdaptorParams(leaf_max_size));
    }

   public:
    /** Deleted copy constructor */
    KdtreeAdaptor(const self_t&) = delete;

    ~KdtreeAdaptor() { delete index_; }
	
    /* @name Interface expected by KDTreeSingleIndexAdaptor */
	 
	typedef KdtreeAdaptor Derived; //!< In this case the dataset class is myself.

	/// CRTP helper method
	inline const Derived& derived() const { return *static_cast<const Derived*>(this); }
	/// CRTP helper method
	inline       Derived& derived()       { return *static_cast<Derived*>(this); }

	// Must return the number of data points
    inline size_t kdtree_get_point_count() const { return datasetSize;}

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
    inline DataType kdtree_get_pt(const size_t idx, const size_t dim) const {
        return RowMajorAccess ? matrixDataPtr[datasetDimension * idx + (dim)] : matrixDataPtr[datasetSize * dim + (idx)];
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX &bb) const { return false; }
};
} // end namespace "nanoflann"