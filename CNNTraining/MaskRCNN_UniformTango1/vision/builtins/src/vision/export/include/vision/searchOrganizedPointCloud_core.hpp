//////////////////////////////////////////////////////////////////////////////
//
//      searchOrganizedPointcloud
//      Copyright 2018-2023 The MathWorks, Inc.
//
//////////////////////////////////////////////////////////////////////////////
#ifndef SEARCHORGANIZEDPOINTCLOUD_CORE_HPP
#define SEARCHORGANIZEDPOINTCLOUD_CORE_HPP


#ifndef  __portable__
//export includes
#include "mfl_scalar/query.hpp"
#include "mfl_scalar/constants.hpp"
#endif

//system includes
#include <limits>
#include <cstring>
#include <queue>
#include <vector>
#include <algorithm>

// include cmath for std::isnan and std::isinf
#include <cmath>

using namespace std;

namespace vision
{
    // Class which is used to hold the index-distance pair
    template <typename DataType>
    class IndexDistancePair
    {
    public:
        IndexDistancePair(uint32_T idx, DataType dist): index(idx), distance(dist){}
        
        bool operator < (const IndexDistancePair &idxDistPair) const
        {
            return distance < idxDistPair.distance;
        }
        
        uint32_T index;
        DataType distance;
    };
    
    template <typename DataType>
    class SearchOrganizedPointCloudImpl
    {
    public:
        SearchOrganizedPointCloudImpl(){}
        // Constructor
        SearchOrganizedPointCloudImpl(DataType *location, uint32_T height_, uint32_T width_ )
        {
            locationPoints = static_cast<DataType*>(location);
                        
            height = height_;
            width = width_;
        }
        
        ~SearchOrganizedPointCloudImpl(){}
        
               // K-Nearest Neighbor search algorithm
        void knnSearch(DataType *point, real64_T kValue, DataType *pointProjection, DataType *KRKRT, vector<uint32_T> &indices, vector<DataType> &distances)
        {
            
            queryPoint = static_cast<DataType *>(point);
            uint32_T k = static_cast<uint32_T>(kValue);
            projectionOfQueryPoint = static_cast<DataType *>(pointProjection);
            kRKRTMatrix = static_cast<DataType *>(KRKRT);
            
            // Limits of the inner window.
            int32_T innerXBegin = static_cast<int32_T>(( projectionOfQueryPoint[0] / projectionOfQueryPoint[2] ) + 0.5f);
            int32_T innerXEnd = innerXBegin + 1;
            int32_T innerYBegin = static_cast<int32_T>(( projectionOfQueryPoint[1] / projectionOfQueryPoint[2] ) + 0.5f);
            int32_T innerYEnd = innerYBegin + 1;
            
            // Limits of the outer window
            int32_T outerXBegin = 0;
            int32_T outerXEnd = static_cast<int32_T>(width) - 1;
            int32_T outerYBegin = 0;
            int32_T outerYEnd = static_cast<int32_T>(height) - 1;
            
            // Priority queue holding the search results
            std::priority_queue<IndexDistancePair<DataType> > searchResults;

            
            if ( (innerXBegin >= 0) && (innerXBegin < static_cast<int32_T>(width)) && (innerYBegin >= 0) && (innerYBegin < static_cast<int32_T>(height)) )
            {
                // Add the query point projected to the image
                uint32_T idxVal = static_cast<uint32_T>(innerXBegin * (int32_T)height + innerYBegin);
                checkIfNNAndAddToQueue(idxVal, k, searchResults);
            }
            else
            {
				int32_T dist = std::numeric_limits<int32_T>::max();

                // Make sure the inner bounding box touches the image
                if (innerXBegin < 0)
                {
					dist = -innerXBegin;
                }
				else if (innerXBegin >= static_cast<int32_T>(width))
                {
					dist = innerXBegin - static_cast<int32_T>(width);
                }
                
                if (innerYBegin < 0)
                {
					dist = std::min(dist, -innerYBegin);
                }
				else if (innerYBegin >= static_cast<int32_T>(height))
                {
					dist = std::min(dist, innerYBegin- static_cast<int32_T>(height));
                }

				innerXBegin = innerXBegin - dist;
				innerXEnd = innerXEnd + dist;
				innerYBegin = innerYBegin - dist;
				innerYEnd = innerYEnd + dist;
            }
           
            // This is the stopping criterion for the search
            bool outerBoxSmallerThanInner = false;
            
            // If we find a nearest point with distance less than the distances currently in the queue
            // we reduce the outer bounding box and search  just in that space.
            bool searchSpaceRecalculate = false;
            
            while(!outerBoxSmallerThanInner)
            {
                innerXBegin--;
                innerXEnd++;
                innerYBegin--;
                innerYEnd++;
                
                // Indexes for searching the box
                uint32_T xStartIdx, xEndIdx, yStartIdx, yEndIdx;
                
                xStartIdx = min(max(innerXBegin, 0), static_cast<int32_T>(width));
                xEndIdx = max(min(innerXEnd,static_cast<int32_T>(width)), 0);
                yStartIdx = min(max(innerYBegin, 0), static_cast<int32_T>(height));
                yEndIdx = min(max(innerYEnd, 0), static_cast<int32_T>(height));
                
                //   Outer Box
                //  -------------------
                // |   Inner Box       |
                // |   ----            |
                // |  |    |           |
                // |  |    |           |
                // |   ----            |
                // |                   |
                //  -------------------
                
                // Since the search always starts with the 1x1 box and then expands by 1 always, it is sufficient to
                // search in the top row, bottom row, left col and right col of the expanded box since all the other inner
                // elements would have been searched in the previous pass.
                
				if (xEndIdx > xStartIdx)
				{
					// Top row of the inner bounding box
					if (innerYBegin >= 0 && innerYBegin < static_cast<int32_T>(height))
					{
						for (uint32_T col = xStartIdx; col < xEndIdx; col++)
						{
							uint32_T idxValue = col * height + yStartIdx;
							searchSpaceRecalculate = checkIfNNAndAddToQueue(idxValue, k, searchResults) || searchSpaceRecalculate;
						}
					}

					// Bottom row of the inner bounding box, innerYEnd does not belong to the box
					if (innerYEnd > 0 && innerYEnd < static_cast<int32_T>(height))
					{
						for (uint32_T col = xStartIdx; col < xEndIdx; col++)
						{
							uint32_T idxValue = col * height + (yEndIdx - 1);
							searchSpaceRecalculate = checkIfNNAndAddToQueue(idxValue, k, searchResults) || searchSpaceRecalculate;
						}
					}

					if (yEndIdx > yStartIdx)
					{
						// Left column of the inner bounding box - We do not go over the elements in the left column which is
						// covered by the top and bottom search
						if (innerXBegin >= 0 && innerXBegin < static_cast<int32_T>(width))
						{
							for (uint32_T row = yStartIdx + 1; row < yEndIdx - 1; row++)
							{
								uint32_T idxValue = xStartIdx * height + row;
								searchSpaceRecalculate = checkIfNNAndAddToQueue(idxValue, k, searchResults) || searchSpaceRecalculate;
							}
						}

						// Right column of the inner bounding box - We do not go over the elements in the right column which is
						// covered by the top and bottom search
						if (innerXEnd > 0 && innerXEnd < static_cast<int32_T>(width))
						{
							for (uint32_T row = yStartIdx + 1; row < yEndIdx - 1; row++)
							{
								uint32_T idxValue = (xEndIdx - 1) * height + row;
								searchSpaceRecalculate = checkIfNNAndAddToQueue(idxValue, k, searchResults) || searchSpaceRecalculate;
							}
						}
					}
				}
                     
				// if there are K neighbors already, the one with the largest distance defines a new search space
				// The new search space covers the existing search space, and it gurarentees points outside the new space won't be better
                if (searchSpaceRecalculate)
                {
                    // Calculate the outer box of the ellipse to modify the search space
                    real64_T largestSqrDistanceInQueue = searchResults.top().distance;
                    uint32_T boundingBox[4];
                    readjustOuterBoxOfEllipse(largestSqrDistanceInQueue, boundingBox);
                    
                    outerXBegin = boundingBox[0];
                    outerXEnd   = boundingBox[1];
                    outerYBegin = boundingBox[2];
                    outerYEnd   = boundingBox[3];
                    
                    searchSpaceRecalculate = false;
                }
                
                // Check if the inner box is whithin the outer box
                outerBoxSmallerThanInner = (outerXBegin >= innerXBegin) && (outerXBegin < innerXEnd) &&
                                           (outerXEnd   >= innerXBegin) && (outerXEnd   < innerXEnd) &&
                                           (outerYBegin >= innerYBegin) && (outerYBegin < innerYEnd) &&
                                           (outerYEnd   >= innerYBegin) && (outerYEnd   < innerYEnd);
                
                if ( (k==1) && searchResults.size() != 0 )
                {
                    outerBoxSmallerThanInner = (searchResults.top().distance == 0) || outerBoxSmallerThanInner;
                }
            }
            
            size_t searchResultsSize = searchResults.size();
            
            indices.resize(searchResultsSize);
            distances.resize(searchResultsSize);
            
            for (int32_T i = (int32_T)searchResultsSize-1; i >= 0; --i)
            {
                // Add 1 to match MATLAB indexing scheme
                indices[i] = searchResults.top().index + 1;
                distances[i] = searchResults.top().distance;
				searchResults.pop();
            }
        }
        
        // Search all the points within the given radius
        void radiusSearch(DataType *point, real64_T radius, DataType *pointProjection, DataType *KRKRT, vector<uint32_T> &indices, vector<DataType> &distances)
        {
            queryPoint = static_cast<DataType *>(point);
            projectionOfQueryPoint = static_cast<DataType *>(pointProjection);
            kRKRTMatrix = static_cast<DataType *>(KRKRT);
            
            uint32_T boundingBox[4];
            
            // Find the boundingg box to search in
            readjustOuterBoxOfEllipse(radius*radius, boundingBox);
            
            // Caluclate the values to iterate over
            
            uint32_T xStartIdx = boundingBox[0];
            uint32_T xEndIdx   = boundingBox[1] + 1;
            uint32_T yStartIdx = boundingBox[2];
            uint32_T yEndIdx   = boundingBox[3] + 1;
           
            // Priority queue holding the search results
            std::vector<IndexDistancePair<DataType> > searchResults;
			searchResults.reserve((yEndIdx-yStartIdx)*(xEndIdx-xStartIdx));
            
            for( uint32_T row = yStartIdx; row < yEndIdx; row++ )
            {
                for( uint32_T col = xStartIdx; col < xEndIdx; col++ )
                {
                    uint32_T idx = col * height + row;
#ifndef  __portable__
                    if ( ! ( mfl_scalar::IsNaN(locationPoints[idx]) || mfl_scalar::IsInf(locationPoints[idx]) ) )
#else
                    if ( ! ( std::isnan(locationPoints[idx]) || std::isinf(locationPoints[idx]) ) )
#endif
                            
                    {
                        DataType xDistance = locationPoints[idx] - queryPoint[0];
                        DataType yDistance = locationPoints[idx+ width*height] - queryPoint[1];
                        DataType zDistance = locationPoints[idx+ width*height*2] - queryPoint[2];
                        
                        DataType euclideanDistance = xDistance * xDistance + yDistance * yDistance + zDistance * zDistance;
                        
                        if (euclideanDistance <= radius*radius)
                        {
                            IndexDistancePair<DataType> newIdxDistPair(idx, euclideanDistance);
                            searchResults.push_back(newIdxDistPair);
                        }
                    }
                }
            }
            
            size_t searchResultsSize = searchResults.size();
            
            indices.resize(searchResultsSize);
            distances.resize(searchResultsSize);
            
            for (size_t i = 0; i < searchResultsSize; ++i)
            {
                // Add 1 to match MATLAB indexing scheme
                indices[i] = searchResults[i].index + 1;
                distances[i] = searchResults[i].distance;
            }
        }
        private:
        DataType *locationPoints;
        DataType *queryPoint;
        DataType *projectionOfQueryPoint; // unnormalized
        
        // Multiplication of the KR (camera * rotation matices) with its transpose
		DataType *kRKRTMatrix;
        
        // Height and Width of the point cloud
		uint32_T height;
		uint32_T width;
        
        // Return the bounding box for the search based on the square radius
		/*
			Let KR = [K1; K2; K3], P0 = [X0, Y0, Z0]', dP is a unit vector
			The ball centered at P0 is P0+r*dP
			Let	q = K[R, t]*[P;1], so the projection [u, v] of a point from the ball is:
			u = (K1*P0+K1*r*dP+K1*t)/(K3*P0+K3*r*dP+K3*t)) => (K3*P0+K3*r*dP+K3*t)) * u = K1*P0+K1*r*dP+K1*t => (Pz+r*K3*dP)*u = Px+r*K1*dP => (r*K3*u-r*K1)*dP = Px-u*Pz
			This suggests that for a given u coordinate, all points that can project to the same u are lying on the plane. This plane needs to intersect with the unit ball
			so that at least one point locates on both the ball and the plane. This implies the distance from [0,0,0] to the plane has to be at most r.
			Let D = KR*(KR)', from the distance requirement above, we get
			(Px-u*Pz)^2 <= (r*K3*u-r*K1)*(r*K3'*u-r*K1') => 0 <= (r^2 * D33 - Pz^2)*u^2 + (2*Px*Pz-2*r^2*D13)*u+(r^2*D11-Px^2)
		*/
        void readjustOuterBoxOfEllipse( real64_T largestSqrDistanceInQueue, uint32_T *boundingBox )
        {
            // Coefficients of the quadratic equation
            
            real64_T a = largestSqrDistanceInQueue * kRKRTMatrix[8] - projectionOfQueryPoint[2] * projectionOfQueryPoint[2];
            real64_T b = largestSqrDistanceInQueue * kRKRTMatrix[5] - projectionOfQueryPoint[1] * projectionOfQueryPoint[2];
            real64_T c = largestSqrDistanceInQueue * kRKRTMatrix[4] - projectionOfQueryPoint[1] * projectionOfQueryPoint[1];
            
            real64_T determinant = b * b - a * c;
            
            // Adjust y
            
            if (determinant < 0 || a >= 0)
            {
                boundingBox[2] = 0;
                boundingBox[3] = height - 1;
            }
            else
            {
                real64_T root1 = (b - sqrt(determinant)) / a;
                real64_T root2 = (b + sqrt(determinant)) / a;
                
                int32_T minimumRoot = min( static_cast<int32_T> (floor(root1)), static_cast<int32_T> (floor(root2)) );
                int32_T maximumRoot = max( static_cast<int32_T> (ceil(root1)), static_cast<int32_T> (ceil(root2)) );
                
                boundingBox[2] = min ((height - 1), static_cast<uint32_T> (max (0, minimumRoot)));
                boundingBox[3] = static_cast<uint32_T> (max (min ((static_cast<int32_T> (height) - 1), maximumRoot), 0));

            }
            
            // Adjust x
            
            b = largestSqrDistanceInQueue * kRKRTMatrix[2] - projectionOfQueryPoint[0] * projectionOfQueryPoint[2];
            c = largestSqrDistanceInQueue * kRKRTMatrix[0] - projectionOfQueryPoint[0] * projectionOfQueryPoint[0];
            
            determinant = b * b - a * c;
            
            if (determinant < 0 || a >= 0)
            {
                boundingBox[0] = 0;
                boundingBox[1] = width - 1;
            }
            else
            {
                real64_T root1 = (b - sqrt(determinant)) / a;
                real64_T root2 = (b + sqrt(determinant)) / a;
                
                int32_T minimumRoot = min( static_cast<int32_T> (floor(root1)), static_cast<int32_T> (floor(root2)) );
                int32_T maximumRoot = max( static_cast<int32_T> (ceil(root1)), static_cast<int32_T> (ceil(root2)) );
                
                boundingBox[0] = min ((width - 1), static_cast<uint32_T>(max (0, minimumRoot)));
                boundingBox[1] = static_cast<uint32_T>(max (min (static_cast<int32_T>(width - 1), maximumRoot), 0));
            }
        }
        
        // Check if the point (idx) is a NN and add it to the queue, return true only if the new point replaces the old one
        bool checkIfNNAndAddToQueue(uint32_T idx, uint32_T k, priority_queue<IndexDistancePair<DataType> > &searchResults)
        {
#ifndef  __portable__
            if ( ! ( mfl_scalar::IsNaN(locationPoints[idx]) || mfl_scalar::IsInf(locationPoints[idx]) ) )
#else
            if ( ! ( std::isnan(locationPoints[idx]) || std::isinf(locationPoints[idx]) ) )
#endif
            {
                DataType xDistance = locationPoints[idx]                 - queryPoint[0];
                DataType yDistance = locationPoints[idx+ width*height]   - queryPoint[1];
                DataType zDistance = locationPoints[idx+ width*height*2] - queryPoint[2];
                
                DataType euclideanDistance = xDistance * xDistance + yDistance * yDistance + zDistance * zDistance;
                
                if( searchResults.size() < k )
                {
                    // Add the element to the queue
                    IndexDistancePair<DataType> newIdxDistPair( idx, euclideanDistance);
                    searchResults.push(newIdxDistPair);
					return (searchResults.size() == k);
                }
                else
                {
                    IndexDistancePair<DataType> itemWithLargestDist = searchResults.top();
                    
                    if( (euclideanDistance <  itemWithLargestDist.distance) )
                    {
                        //Remove the element with largest distance
                        searchResults.pop();
                        IndexDistancePair<DataType> newIdxDistPair( idx, euclideanDistance);
                        searchResults.push(newIdxDistPair);
                        // Notify that the queue has been changed and search space needs to be recalculated
                        return true;
                    }
                }
            }

            // Continue with the search
            return false;
        }
    };
}
#endif