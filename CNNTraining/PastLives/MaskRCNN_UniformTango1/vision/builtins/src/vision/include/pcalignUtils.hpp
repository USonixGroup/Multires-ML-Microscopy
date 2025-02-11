///////////////////////////////////////////////////////////////////////////
// This file contains utility functions for pcalign
//
// Copyright 2021-2022 The MathWorks, Inc.
///////////////////////////////////////////////////////////////////////////
#ifndef COMPILE_FOR_VISION_BUILTINS
#include <utility>

///////////////////////////////////////////////////////////////////////////
// Voxel grid filter on an array of point clouds
///////////////////////////////////////////////////////////////////////////
template<typename PointsType, typename ColorType, typename IntensityValueType>
        void voxelGridAlgImpl(const PointsType* pData,
        const ColorType* pColor,
        const PointsType* pNormal,
        const IntensityValueType* pIntensity,
        PointsType* pFilteredData,
        ColorType* pFilteredColorData,
        PointsType* pFilteredNormalData,
        IntensityValueType* pFilteredIntensityData,
        const uint64_T* indexVector,
        const uint32_T numPoints,
        const boolean_T needColor,
        const boolean_T needNormal,
        const boolean_T needIntensity,
        const uint32_T indexVecSize,
        const uint32_T* sizeArray,
        const uint32_T total) {
    
    typedef typename std::conditional<std::is_same<IntensityValueType, double>::value, double, float>::type IntensityType;
    
    double cx(0), cy(0), cz(0);
    float cr(0), cg(0), cb(0);
    PointsType nx(0), ny(0), nz(0);
    IntensityType ix(0);
    uint32_T index(0);
    
    for (uint64_T n(0); n < indexVecSize;) {
        
        uint64_T pcIndex     = indexVector[n + indexVecSize];
        uint64_T matrixIndex = indexVector[n + indexVecSize*2];
        
        int offset = 0;
        if (pcIndex != 0){
            for(uint64_T idx = 0; idx < pcIndex; idx++)
                offset += sizeArray[idx];
        }
        else
            offset = 0;
        
        cx = (double)pData[matrixIndex + offset];
        cy = (double)pData[matrixIndex + offset + numPoints];
        cz = (double)pData[matrixIndex + offset + numPoints * 2];
        
        if (needColor) {
            cr = (float)pColor[matrixIndex + offset];
            cg = (float)pColor[matrixIndex + offset + numPoints];
            cb = (float)pColor[matrixIndex + offset + numPoints * 2];
        }
        if (needNormal) {
            nx = pNormal[matrixIndex + offset];
            ny = pNormal[matrixIndex + offset + numPoints];
            nz = pNormal[matrixIndex + offset + numPoints * 2];
        }
        if (needIntensity) {
            ix = static_cast<IntensityType>(pIntensity[matrixIndex + offset]);
        }
        
        uint64_T i(n+1);
        while (i < indexVecSize && indexVector[i] == indexVector[n]) {
            
            pcIndex     = indexVector[i + indexVecSize];
            matrixIndex = indexVector[i + indexVecSize*2];
            
            offset = 0;
            if (pcIndex != 0){
                for(uint64_T idx = 0; idx < pcIndex ;idx++)
                    offset += sizeArray[idx];
            }
            else
                offset = 0;
            
            cx += (double)pData[matrixIndex + offset];
            cy += (double)pData[matrixIndex + offset + numPoints];
            cz += (double)pData[matrixIndex + offset + numPoints * 2];
            
            if (needColor) {
                cr += (float)pColor[matrixIndex + offset];
                cg += (float)pColor[matrixIndex + offset + numPoints];
                cb += (float)pColor[matrixIndex + offset + numPoints * 2];
            }
            if (needNormal) {
                nx += pNormal[matrixIndex + offset];
                ny += pNormal[matrixIndex + offset + numPoints];
                nz += pNormal[matrixIndex + offset + numPoints * 2];
            }
            if (needIntensity) {
                ix += static_cast<IntensityType>(pIntensity[matrixIndex + offset]);             }
            
            i++;
        }
        uint64_T numPtsIdx = (i - n);
        cx /= numPtsIdx;
        cy /= numPtsIdx;
        cz /= numPtsIdx;
        
        // Store averages
        pFilteredData[index]             = static_cast<PointsType>(cx);
        pFilteredData[index + total]     = static_cast<PointsType>(cy);
        pFilteredData[index + 2 * total] = static_cast<PointsType>(cz);
        
        if (needColor) {
            cr /= numPtsIdx;
            cg /= numPtsIdx;
            cb /= numPtsIdx;
            pFilteredColorData[index]               = static_cast<ColorType>(cr);
            pFilteredColorData[index + total]       = static_cast<ColorType>(cg);
            pFilteredColorData[index + 2 * total]   = static_cast<ColorType>(cb);
        }
        if (needNormal) {
            nx /= numPtsIdx;
            ny /= numPtsIdx;
            nz /= numPtsIdx;
            pFilteredNormalData[index]              = nx;
            pFilteredNormalData[index + total]      = ny;
            pFilteredNormalData[index + 2 * total]  = nz;
        }
        if (needIntensity) {
            ix /= numPtsIdx;
            pFilteredIntensityData[index] = static_cast<IntensityValueType>(ix);
        }
        n = i;
        index++;
    }
}
#endif
