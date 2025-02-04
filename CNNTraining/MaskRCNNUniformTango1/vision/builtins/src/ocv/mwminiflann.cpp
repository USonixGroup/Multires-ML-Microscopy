#include "precomp_flann.hpp"

#define MINIFLANN_SUPPORT_EXOTIC_DISTANCE_TYPES 0

static cvflann::IndexParams& get_params(const cv::flann::IndexParams& p)
{
    return *(cvflann::IndexParams*)(p.params);
}

cv::flann::IndexParams::~IndexParams()
{
    delete &get_params(*this);
}

namespace cv
{

namespace flann
{

 using namespace cvflann;

IndexParams::IndexParams()
{
    params = new ::cvflann::IndexParams();
}
 
template<typename T>
T getParam(const IndexParams& _p, const std::string& key, const T& defaultVal=T())
{
    ::cvflann::IndexParams& p = get_params(_p);
    ::cvflann::IndexParams::const_iterator it = p.find(key);
    if( it == p.end() )
        return defaultVal;
    return it->second.cast<T>();
}

KDTreeIndexParams::KDTreeIndexParams(int trees)
{
    ::cvflann::IndexParams& p = get_params(*this);
    p["algorithm"] = FLANN_INDEX_KDTREE;
    p["trees"] = trees;
}


HierarchicalClusteringIndexParams::HierarchicalClusteringIndexParams(int branching ,
                                      flann_centers_init_t centers_init,
                                      int trees, int leaf_size)
{
    ::cvflann::IndexParams& p = get_params(*this);
    p["algorithm"] = FLANN_INDEX_HIERARCHICAL;
    // The branching factor used in the hierarchical clustering
    p["branching"] = branching;
    // Algorithm used for picking the initial cluster centers
    p["centers_init"] = centers_init;
    // number of parallel trees to build
    p["trees"] = trees;
    // maximum leaf size
    p["leaf_size"] = leaf_size;
}

SearchParams::SearchParams( int checks, float eps, bool sorted )
{
    ::cvflann::IndexParams& p = get_params(*this);

    // how many leafs to visit when searching for neighbours (-1 for unlimited)
    p["checks"] = checks;
    // search for eps-approximate neighbours (default: 0)
    p["eps"] = eps;
    // only for radius search, require neighbours sorted by distance (default: true)
    p["sorted"] = sorted;
}
 
template<typename Distance, typename IndexType> void
buildIndex_(void*& index, const Mat& data, const IndexParams& params, const Distance& dist = Distance())
{
    typedef typename Distance::ElementType ElementType;
    if(DataType<ElementType>::type != data.type())
        CV_Error_(cv::Error::StsUnsupportedFormat, ("type=%d\n", data.type()));
    if(!data.isContinuous())
        CV_Error(cv::Error::StsBadArg, "Only continuous arrays are supported");

    ::cvflann::Matrix<ElementType> dataset((ElementType*)data.data, data.rows, data.cols);
    IndexType* _index = new IndexType(dataset, get_params(params), dist);
    _index->buildIndex();
    index = _index;
}

template<typename Distance> void
buildIndex(void*& index, const Mat& data, const IndexParams& params, const Distance& dist = Distance())
{
    buildIndex_<Distance, ::cvflann::Index<Distance> >(index, data, params, dist);
}

#if CV_NEON
typedef ::cvflann::Hamming<uchar> HammingDistance;
#else
typedef ::cvflann::HammingLUT HammingDistance;
#endif

Index::Index(InputArray _data, const IndexParams& params, flann_distance_t _distType)
{
    index = 0;
    featureType = CV_32F;
    algo = FLANN_INDEX_LINEAR;
    distType = FLANN_DIST_L2;
    build(_data, params, _distType);
}

void Index::build(InputArray _data, const IndexParams& params, flann_distance_t _distType)
{
    release();
    algo = getParam<flann_algorithm_t>(params, "algorithm", FLANN_INDEX_LINEAR);
    if( algo == FLANN_INDEX_SAVED )
    {
        load(_data, getParam<std::string>(params, "filename", std::string()));
        return;
    }

    Mat data = _data.getMat();
    index = 0;
    featureType = data.type();
    distType = _distType;

    if ( algo == FLANN_INDEX_LSH)
    {
        distType = FLANN_DIST_HAMMING;
    }

    switch( distType )
    {
    case FLANN_DIST_HAMMING:
        buildIndex< HammingDistance >(index, data, params);
        break;
    case FLANN_DIST_L2:
        buildIndex< ::cvflann::L2<float> >(index, data, params);
        break;
    case FLANN_DIST_L1:
        buildIndex< ::cvflann::L1<float> >(index, data, params);
        break;
#if MINIFLANN_SUPPORT_EXOTIC_DISTANCE_TYPES
    case FLANN_DIST_MAX:
        buildIndex< ::cvflann::MaxDistance<float> >(index, data, params);
        break;
    case FLANN_DIST_HIST_INTERSECT:
        buildIndex< ::cvflann::HistIntersectionDistance<float> >(index, data, params);
        break;
    case FLANN_DIST_HELLINGER:
        buildIndex< ::cvflann::HellingerDistance<float> >(index, data, params);
        break;
    case FLANN_DIST_CHI_SQUARE:
        buildIndex< ::cvflann::ChiSquareDistance<float> >(index, data, params);
        break;
    case FLANN_DIST_KL:
        buildIndex< ::cvflann::KL_Divergence<float> >(index, data, params);
        break;
#endif
    default:
        CV_Error(cv::Error::StsBadArg, "Unknown/unsupported distance type");
    }
}

template<typename IndexType> void deleteIndex_(void* index)
{
    delete (IndexType*)index;
}

template<typename Distance> void deleteIndex(void* index)
{
    deleteIndex_< ::cvflann::Index<Distance> >(index);
}

Index::~Index()
{
    release();
}

void Index::release()
{
    if( !index )
        return;

    switch( distType )
    {
        case FLANN_DIST_HAMMING:
            deleteIndex< HammingDistance >(index);
            break;
        case FLANN_DIST_L2:
            deleteIndex< ::cvflann::L2<float> >(index);
            break;
        case FLANN_DIST_L1:
            deleteIndex< ::cvflann::L1<float> >(index);
            break;
#if MINIFLANN_SUPPORT_EXOTIC_DISTANCE_TYPES
        case FLANN_DIST_MAX:
            deleteIndex< ::cvflann::MaxDistance<float> >(index);
            break;
        case FLANN_DIST_HIST_INTERSECT:
            deleteIndex< ::cvflann::HistIntersectionDistance<float> >(index);
            break;
        case FLANN_DIST_HELLINGER:
            deleteIndex< ::cvflann::HellingerDistance<float> >(index);
            break;
        case FLANN_DIST_CHI_SQUARE:
            deleteIndex< ::cvflann::ChiSquareDistance<float> >(index);
            break;
        case FLANN_DIST_KL:
            deleteIndex< ::cvflann::KL_Divergence<float> >(index);
            break;
#endif
        default:
            CV_Error(cv::Error::StsBadArg, "Unknown/unsupported distance type");
    }
    index = 0;
}

template<typename Distance, typename IndexType>
void runKnnSearch_(void* index, const Mat& query, Mat& indices, Mat& dists,
                  int knn, const SearchParams& params)
{
    typedef typename Distance::ElementType ElementType;
    typedef typename Distance::ResultType DistanceType;
    int type = DataType<ElementType>::type;
    int dtype = DataType<DistanceType>::type;
    CV_Assert(query.type() == type && indices.type() == CV_32S && dists.type() == dtype);
    CV_Assert(query.isContinuous() && indices.isContinuous() && dists.isContinuous());

    ::cvflann::Matrix<ElementType> _query((ElementType*)query.data, query.rows, query.cols);
    ::cvflann::Matrix<int> _indices((int*)indices.data, indices.rows, indices.cols);
    ::cvflann::Matrix<DistanceType> _dists((DistanceType*)dists.data, dists.rows, dists.cols);

    ((IndexType*)index)->knnSearch(_query, _indices, _dists, knn,
                                   (const ::cvflann::SearchParams&)get_params(params));
}

template<typename Distance>
void runKnnSearch(void* index, const Mat& query, Mat& indices, Mat& dists,
                  int knn, const SearchParams& params)
{
    runKnnSearch_<Distance, ::cvflann::Index<Distance> >(index, query, indices, dists, knn, params);
}


static void createIndicesDists(OutputArray _indices, OutputArray _dists,
                               Mat& indices, Mat& dists, int rows,
                               int minCols, int maxCols, int dtype)
{
    if( _indices.needed() )
    {
        indices = _indices.getMat();
        if( !indices.isContinuous() || indices.type() != CV_32S ||
            indices.rows != rows || indices.cols < minCols || indices.cols > maxCols )
        {
            if( !indices.isContinuous() )
               _indices.release();
            _indices.create( rows, minCols, CV_32S );
            indices = _indices.getMat();
        }
    }
    else
        indices.create( rows, minCols, CV_32S );

    if( _dists.needed() )
    {
        dists = _dists.getMat();
        if( !dists.isContinuous() || dists.type() != dtype ||
           dists.rows != rows || dists.cols < minCols || dists.cols > maxCols )
        {
            if( !indices.isContinuous() )
                _dists.release();
            _dists.create( rows, minCols, dtype );
            dists = _dists.getMat();
        }
    }
    else
        dists.create( rows, minCols, dtype );
}


void Index::knnSearch(InputArray _query, OutputArray _indices,
               OutputArray _dists, int knn, const SearchParams& params)
{
    Mat query = _query.getMat(), indices, dists;
    int dtype = distType == FLANN_DIST_HAMMING ? CV_32S : CV_32F;

    createIndicesDists( _indices, _dists, indices, dists, query.rows, knn, knn, dtype );

    switch( distType )
    {
    case FLANN_DIST_HAMMING:
        runKnnSearch<HammingDistance>(index, query, indices, dists, knn, params);
        break;
    case FLANN_DIST_L2:
        runKnnSearch< ::cvflann::L2<float> >(index, query, indices, dists, knn, params);
        break;
    case FLANN_DIST_L1:
        runKnnSearch< ::cvflann::L1<float> >(index, query, indices, dists, knn, params);
        break;
#if MINIFLANN_SUPPORT_EXOTIC_DISTANCE_TYPES
    case FLANN_DIST_MAX:
        runKnnSearch< ::cvflann::MaxDistance<float> >(index, query, indices, dists, knn, params);
        break;
    case FLANN_DIST_HIST_INTERSECT:
        runKnnSearch< ::cvflann::HistIntersectionDistance<float> >(index, query, indices, dists, knn, params);
        break;
    case FLANN_DIST_HELLINGER:
        runKnnSearch< ::cvflann::HellingerDistance<float> >(index, query, indices, dists, knn, params);
        break;
    case FLANN_DIST_CHI_SQUARE:
        runKnnSearch< ::cvflann::ChiSquareDistance<float> >(index, query, indices, dists, knn, params);
        break;
    case FLANN_DIST_KL:
        runKnnSearch< ::cvflann::KL_Divergence<float> >(index, query, indices, dists, knn, params);
        break;
#endif
    default:
        CV_Error(cv::Error::StsBadArg, "Unknown/unsupported distance type");
    }
}

}

}
