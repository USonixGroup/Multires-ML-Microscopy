/*
* Copyright 2024 The MathWorks, Inc.
*/

#ifndef ROIALIGNFORWARD_HPP
#define ROIALIGNFORWARD_HPP

#include <vector>    // vector
#include <cmath>     // std::max 

struct BilinearCache
{
    size_t pos1;
    size_t pos2;
    size_t pos3;
    size_t pos4;
    float w1;
    float w2;
    float w3;
    float w4;
};
///////////////////////////////////////////////////////////////////////////
// Precalculate & cache interpolation for each ROI point grid
void cacheBilinearInterpolation(
    const size_t xHeight,
    const size_t xWidth,
    const size_t pooledHeight,
    const size_t pooledWidth,
    const size_t samplingRatioX,
    const size_t samplingRatioY,
    const float roiStartX,
    const float roiStartY,
    const float binSizeX,
    const float binSizeY,
    std::vector<BilinearCache>* interpCache)
{

    // Loop over all points in the roi grid (including bin samples) and
    // calculate the weights and 4 neighbors for each point and cache them.
    size_t interpCacheIndex = 0;

    for(size_t poolXIndex = 0; poolXIndex < pooledWidth; ++poolXIndex)
    {

        for(size_t poolYIndex = 0; poolYIndex < pooledHeight; ++poolYIndex)
        {

            for(size_t binXIndex = 0; binXIndex < samplingRatioX; ++binXIndex)
            {

                for(size_t binYIndex = 0; binYIndex < samplingRatioY; ++binYIndex)
                {
                    // Calculate the remapped point in featureMap coordinates

                    float xs = roiStartX + poolXIndex * binSizeX + (binXIndex+float(0.5)) * (binSizeX/samplingRatioX);
                    float ys = roiStartY + poolYIndex * binSizeY + (binYIndex+float(0.5)) * (binSizeY/samplingRatioY);

                    // Check out of bounds
                    if(xs<-1 || xs>xWidth || ys<-1 || ys>xHeight)
                    {
                        BilinearCache cache;

                        cache.pos1 = 0;
                        cache.pos2 = 0;
                        cache.pos3 = 0;
                        cache.pos4 = 0;
                        cache.w1 = 0;
                        cache.w2 = 0;
                        cache.w3 = 0;
                        cache.w4 = 0;

                        interpCache->at(interpCacheIndex) = cache;
                        interpCacheIndex+=1;
                        continue;
                    }


                    size_t xLow = size_t(xs);
                    size_t xHigh = size_t(xs) + 1;
                    size_t yLow = size_t(ys);
                    size_t yHigh = size_t(ys)+1;


                    if(xLow >= xWidth-1)
                    {
                        xLow = xWidth-1;
                        xHigh = xWidth-1;
                        xs = float(xLow);
                    }

                    if(yLow >= xHeight -1)
                    {
                        yLow = xHeight-1;
                        yHigh = xHeight-1;
                        ys = float(yLow);
                    }

                    /*
                    *          (1)---------(2)
                    *           |           |
                    *           |  x(xs,ys) |
                    *           |           |
                    *          (3)---------(4)
                    *
                    */
                    // Contribution from each of the 4 neighbors

                    float yRatioLow = ys - yLow;
                    float xRatioLow = xs - xLow;
                    float yRatioHigh = 1 - yRatioLow;
                    float xRatioHigh = 1 - xRatioLow;
                    float w1 = yRatioHigh * xRatioHigh;
                    float w2 = yRatioHigh * xRatioLow;
                    float w3 = yRatioLow * xRatioHigh;
                    float w4 = yRatioLow * xRatioLow;

                    size_t pos1 = xLow * xHeight + yLow;
                    size_t pos2 = xHigh * xHeight + yLow;
                    size_t pos3 = xLow * xHeight + yHigh;
                    size_t pos4 = xHigh * xHeight + yHigh;

                    BilinearCache bc;

                    bc.pos1 = pos1;
                    bc.pos2 = pos2;
                    bc.pos3 = pos3;
                    bc.pos4 = pos4;
                    bc.w1 = w1;
                    bc.w2 = w2;
                    bc.w3 = w3;
                    bc.w4 = w4;

                    interpCache->at(interpCacheIndex) = bc;
                    interpCacheIndex += 1;

                }
            }
        }
    }
}
///////////////////////////////////////////////////////////////////////////
// Calculate pooled dl data
void pooledDlDataImpl(float const* roi, float* output,  const float* src,
                      const int32_T initSamplingRatioX, const int32_T initSamplingRatioY, const uint32_T numChannels,
                      const uint32_T numROI, const uint32_T pooledHeight, const uint32_T pooledWidth,
                      const uint32_T xDim, const uint32_T yDim)
{

    float * dst;
    for(size_t i = 0; i < numROI; ++i)
    {
        // Reset sampling ratio for each ROI to handle the auto case of (-1,-1)
        auto samplingRatioX = initSamplingRatioX;
        auto samplingRatioY = initSamplingRatioY;
        
        dst = output + i*numChannels*pooledHeight*pooledWidth;

        const float roiStartX = (roi[0 + i*5])-1;
        const float roiStartY = (roi[1 + i*5])-1;
        const float roiEndX   = (roi[2 + i*5])-1;
        const float roiEndY   = (roi[3 + i*5])-1;

        const size_t batchIdx = static_cast<size_t>(roi[4 + i*5]) - 1;

        // Size of feature map region to process.
        const float roiWidth = std::max(static_cast<float>(roiEndX - roiStartX), float(1));
        const float roiHeight= std::max(static_cast<float>(roiEndY - roiStartY), float(1));

        // Size of pooling region
        const float binSizeX  = roiWidth / pooledWidth;
        const float binSizeY = roiHeight/ pooledHeight;

        // Handle default value of samplingRatio = -1
        if((samplingRatioX<0)||(samplingRatioY<0))
        {
            samplingRatioX = int(ceil(roiWidth/pooledWidth));
            samplingRatioY = int(ceil(roiHeight/pooledHeight));
        }

        // Position at upper-left corner of ROI in input feature map.
        auto srcBatchOffset = batchIdx*numChannels*xDim*yDim;

        // Count for performing average pooling in each bin
        size_t binSampleCount = size_t(samplingRatioX * samplingRatioY);

        float binSampleCountInv = float(1)/float(binSampleCount);

        std::vector<BilinearCache> interpCache(binSampleCount*pooledWidth*pooledHeight);

        cacheBilinearInterpolation(
            xDim,
            yDim,
            pooledHeight,
            pooledWidth,
            samplingRatioX,
            samplingRatioY,
            roiStartX,
            roiStartY,
            binSizeX,
            binSizeY,
            &interpCache) ;

        for(size_t c = 0; c < numChannels; c++)
        {
            size_t srcChannelOffset =  c * xDim * yDim;
            size_t dstChannelOffset =  c * pooledWidth * pooledHeight;
            size_t bilinearCacheIndex = 0;

            for(size_t x = 0; x < pooledWidth; x++ )
            {
                for(size_t y = 0; y < pooledHeight; y++)
                {
                    // For each(x,y,c) look up neighbors and weights from the cache
                    size_t dstXYOffset = x * pooledHeight + y;

                    const float* srcOffsetData = src+srcBatchOffset+srcChannelOffset;

                    size_t dstOffset = dstChannelOffset + dstXYOffset;

                    float outputVal = 0;

                    for(int binXIndex = 0; binXIndex < samplingRatioX; binXIndex++)
                    {
                        for(int binYIndex = 0; binYIndex < samplingRatioY; binYIndex++)
                        {
                            BilinearCache cache = interpCache[bilinearCacheIndex];

                            outputVal += cache.w1*srcOffsetData[cache.pos1] +
                                cache.w2*srcOffsetData[cache.pos2] +
                                cache.w3*srcOffsetData[cache.pos3] +
                                cache.w4*srcOffsetData[cache.pos4];

                            bilinearCacheIndex += 1;

                        }
                    }

                    outputVal*=binSampleCountInv;
                    dst[dstOffset] = outputVal;
                }
            }
        }
    }
}

#endif