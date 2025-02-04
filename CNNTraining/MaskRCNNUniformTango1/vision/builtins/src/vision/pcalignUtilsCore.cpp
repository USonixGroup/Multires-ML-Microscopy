//////////////////////////////////////////////////////////////////////////////
//
// APIs for interfaces for pcalignUtils that are used in codegeneration
// Copyright 2021-2022 The MathWorks, Inc.
//
//////////////////////////////////////////////////////////////////////////////
// vision_builtins does not need this source file

#ifndef COMPILE_FOR_VISION_BUILTINS

#include "pcalignUtilsCore_api.hpp"
#include "pcalignUtils.hpp"

///////////////////////////////////////////////////////////////////////////
//    voxelGridFilter Wrapper for single dataType and uint8 Color
///////////////////////////////////////////////////////////////////////////
void voxelGridAlgImpl_single_uint8Color(const float* pData,
        const unsigned char* pColor,
        const float* pNormal,
        const float* pIntensity,
        float* pFilteredData,
        unsigned char* pFilteredColorData,
        float* pFilteredNormalData,
        float* pFilteredIntensityData,
        const uint64_T* indexVector,
        const uint32_T numClouds,
        const boolean_T needColor,
        const boolean_T needNormal,
        const boolean_T needIntensity,
        const uint32_T indexVecSize,
        const uint32_T* sizeArray,
        const uint32_T total){
    
    voxelGridAlgImpl<float, unsigned char, float>(pData, pColor, pNormal, pIntensity, pFilteredData,
            pFilteredColorData, pFilteredNormalData, pFilteredIntensityData,
            indexVector, numClouds, needColor, needNormal, needIntensity,
            indexVecSize, sizeArray, total);
}
///////////////////////////////////////////////////////////////////////////
//    voxelGridFilter Wrapper for double dataType and uint8 Color 
///////////////////////////////////////////////////////////////////////////
void voxelGridAlgImpl_double_uint8Color(const double* pData,
        const unsigned char* pColor,
        const double* pNormal,
        const double* pIntensity,
        double* pFilteredData,
        unsigned char* pFilteredColorData,
        double* pFilteredNormalData,
        double* pFilteredIntensityData,
        const uint64_T* indexVector,
        const uint32_T numClouds,
        const boolean_T needColor,
        const boolean_T needNormal,
        const boolean_T needIntensity,
        const uint32_T indexVecSize,
        const uint32_T* sizeArray,
        const uint32_T total){
    
    voxelGridAlgImpl<double, unsigned char, double>(pData, pColor, pNormal, pIntensity, pFilteredData,
            pFilteredColorData, pFilteredNormalData, pFilteredIntensityData,
            indexVector, numClouds, needColor, needNormal, needIntensity,
            indexVecSize, sizeArray, total);
}
///////////////////////////////////////////////////////////////////////////
// voxelGridFilter Wrapper for single dataType, uint8 Color and uint8 Intensity
///////////////////////////////////////////////////////////////////////////
void voxelGridAlgImpl_single_uint8Color_uint8Intensity(const float* pData,
        const unsigned char* pColor,
        const float* pNormal,
        const unsigned char* pIntensity,
        float* pFilteredData,
        unsigned char* pFilteredColorData,
        float* pFilteredNormalData,
        unsigned char* pFilteredIntensityData,
        const uint64_T* indexVector,
        const uint32_T numClouds,
        const boolean_T needColor,
        const boolean_T needNormal,
        const boolean_T needIntensity,
        const uint32_T indexVecSize,
        const uint32_T* sizeArray,
        const uint32_T total){
    
    voxelGridAlgImpl<float, unsigned char, unsigned char>(pData, pColor, pNormal, pIntensity,
            pFilteredData, pFilteredColorData, pFilteredNormalData,
            pFilteredIntensityData, indexVector, numClouds, needColor, needNormal,
            needIntensity, indexVecSize, sizeArray, total);
}
///////////////////////////////////////////////////////////////////////////
// voxelGridFilter Wrapper for double dataType, uint8 Color and uint8 Intensity
///////////////////////////////////////////////////////////////////////////
void voxelGridAlgImpl_double_uint8Color_uint8Intensity(const double* pData,
        const unsigned char* pColor,
        const double* pNormal,
        const unsigned char* pIntensity,
        double* pFilteredData,
        unsigned char* pFilteredColorData,
        double* pFilteredNormalData,
        unsigned char* pFilteredIntensityData,
        const uint64_T* indexVector,
        const uint32_T numClouds,
        const boolean_T needColor,
        const boolean_T needNormal,
        const boolean_T needIntensity,
        const uint32_T indexVecSize,
        const uint32_T* sizeArray,
        const uint32_T total){
    
    voxelGridAlgImpl<double, unsigned char, unsigned char>(pData, pColor, pNormal, pIntensity,
            pFilteredData, pFilteredColorData, pFilteredNormalData,
            pFilteredIntensityData, indexVector, numClouds, needColor, needNormal,
            needIntensity, indexVecSize, sizeArray, total);
}
///////////////////////////////////////////////////////////////////////////
// voxelGridFilter Wrapper for single dataType, uint8 Color and uint16 Intensity
///////////////////////////////////////////////////////////////////////////
void voxelGridAlgImpl_single_uint8Color_uint16Intensity(const float* pData,
        const unsigned char* pColor,
        const float* pNormal,
        const uint16_T* pIntensity,
        float* pFilteredData,
        unsigned char* pFilteredColorData,
        float* pFilteredNormalData,
        uint16_T* pFilteredIntensityData,
        const uint64_T* indexVector,
        const uint32_T numClouds,
        const boolean_T needColor,
        const boolean_T needNormal,
        const boolean_T needIntensity,
        const uint32_T indexVecSize,
        const uint32_T* sizeArray,
        const uint32_T total){
    
    voxelGridAlgImpl<float, unsigned char, uint16_T>(pData, pColor, pNormal, pIntensity,
            pFilteredData, pFilteredColorData, pFilteredNormalData,
            pFilteredIntensityData, indexVector, numClouds, needColor, needNormal,
            needIntensity, indexVecSize, sizeArray, total);
}
///////////////////////////////////////////////////////////////////////////
// voxelGridFilter Wrapper for double dataType, uint8 Color and uint16 Intensity
///////////////////////////////////////////////////////////////////////////
void voxelGridAlgImpl_double_uint8Color_uint16Intensity(const double* pData,
        const unsigned char* pColor,
        const double* pNormal,
        const uint16_T* pIntensity,
        double* pFilteredData,
        unsigned char* pFilteredColorData,
        double* pFilteredNormalData,
        uint16_T* pFilteredIntensityData,
        const uint64_T* indexVector,
        const uint32_T numClouds,
        const boolean_T needColor,
        const boolean_T needNormal,
        const boolean_T needIntensity,
        const uint32_T indexVecSize,
        const uint32_T* sizeArray,
        const uint32_T total){
    
    voxelGridAlgImpl<double, unsigned char, uint16_T>(pData, pColor, pNormal, pIntensity,
            pFilteredData, pFilteredColorData, pFilteredNormalData,
            pFilteredIntensityData, indexVector, numClouds, needColor, needNormal,
            needIntensity, indexVecSize, sizeArray, total);
}
///////////////////////////////////////////////////////////////////////////
//    voxelGridFilter Wrapper for single dataType and uint16 Color
///////////////////////////////////////////////////////////////////////////
void voxelGridAlgImpl_single_uint16Color(const float* pData,
        const uint16_T* pColor,
        const float* pNormal,
        const float* pIntensity,
        float* pFilteredData,
        uint16_T* pFilteredColorData,
        float* pFilteredNormalData,
        float* pFilteredIntensityData,
        const uint64_T* indexVector,
        const uint32_T numClouds,
        const boolean_T needColor,
        const boolean_T needNormal,
        const boolean_T needIntensity,
        const uint32_T indexVecSize,
        const uint32_T* sizeArray,
        const uint32_T total){
    
    voxelGridAlgImpl<float, uint16_T, float>(pData, pColor, pNormal, pIntensity, pFilteredData,
            pFilteredColorData, pFilteredNormalData, pFilteredIntensityData,
            indexVector, numClouds, needColor, needNormal, needIntensity,
            indexVecSize, sizeArray, total);
}
///////////////////////////////////////////////////////////////////////////
//    voxelGridFilter Wrapper for double dataType and uint16 Color 
///////////////////////////////////////////////////////////////////////////
void voxelGridAlgImpl_double_uint16Color(const double* pData,
        const uint16_T* pColor,
        const double* pNormal,
        const double* pIntensity,
        double* pFilteredData,
        uint16_T* pFilteredColorData,
        double* pFilteredNormalData,
        double* pFilteredIntensityData,
        const uint64_T* indexVector,
        const uint32_T numClouds,
        const boolean_T needColor,
        const boolean_T needNormal,
        const boolean_T needIntensity,
        const uint32_T indexVecSize,
        const uint32_T* sizeArray,
        const uint32_T total){
    
    voxelGridAlgImpl<double, uint16_T, double>(pData, pColor, pNormal, pIntensity, pFilteredData,
            pFilteredColorData, pFilteredNormalData, pFilteredIntensityData,
            indexVector, numClouds, needColor, needNormal, needIntensity,
            indexVecSize, sizeArray, total);
}
///////////////////////////////////////////////////////////////////////////
// voxelGridFilter Wrapper for single dataType, uint16 Color and uint8 Intensity
///////////////////////////////////////////////////////////////////////////
void voxelGridAlgImpl_single_uint16Color_uint8Intensity(const float* pData,
        const uint16_T* pColor,
        const float* pNormal,
        const unsigned char* pIntensity,
        float* pFilteredData,
        uint16_T* pFilteredColorData,
        float* pFilteredNormalData,
        unsigned char* pFilteredIntensityData,
        const uint64_T* indexVector,
        const uint32_T numClouds,
        const boolean_T needColor,
        const boolean_T needNormal,
        const boolean_T needIntensity,
        const uint32_T indexVecSize,
        const uint32_T* sizeArray,
        const uint32_T total){
    
    voxelGridAlgImpl<float, uint16_T, unsigned char>(pData, pColor, pNormal, pIntensity,
            pFilteredData, pFilteredColorData, pFilteredNormalData,
            pFilteredIntensityData, indexVector, numClouds, needColor, needNormal,
            needIntensity, indexVecSize, sizeArray, total);
}
///////////////////////////////////////////////////////////////////////////
// voxelGridFilter Wrapper for double dataType, uint16 Color and uint8 Intensity
///////////////////////////////////////////////////////////////////////////
void voxelGridAlgImpl_double_uint16Color_uint8Intensity(const double* pData,
        const uint16_T* pColor,
        const double* pNormal,
        const unsigned char* pIntensity,
        double* pFilteredData,
        uint16_T* pFilteredColorData,
        double* pFilteredNormalData,
        unsigned char* pFilteredIntensityData,
        const uint64_T* indexVector,
        const uint32_T numClouds,
        const boolean_T needColor,
        const boolean_T needNormal,
        const boolean_T needIntensity,
        const uint32_T indexVecSize,
        const uint32_T* sizeArray,
        const uint32_T total){
    
    voxelGridAlgImpl<double, uint16_T, unsigned char>(pData, pColor, pNormal, pIntensity,
            pFilteredData, pFilteredColorData, pFilteredNormalData,
            pFilteredIntensityData, indexVector, numClouds, needColor, needNormal,
            needIntensity, indexVecSize, sizeArray, total);
}
///////////////////////////////////////////////////////////////////////////
// voxelGridFilter Wrapper for single dataType, uint16 Color and uint16 Intensity
///////////////////////////////////////////////////////////////////////////
void voxelGridAlgImpl_single_uint16Color_uint16Intensity(const float* pData,
        const uint16_T* pColor,
        const float* pNormal,
        const uint16_T* pIntensity,
        float* pFilteredData,
        uint16_T* pFilteredColorData,
        float* pFilteredNormalData,
        uint16_T* pFilteredIntensityData,
        const uint64_T* indexVector,
        const uint32_T numClouds,
        const boolean_T needColor,
        const boolean_T needNormal,
        const boolean_T needIntensity,
        const uint32_T indexVecSize,
        const uint32_T* sizeArray,
        const uint32_T total){
    
    voxelGridAlgImpl<float, uint16_T, uint16_T>(pData, pColor, pNormal, pIntensity,
            pFilteredData, pFilteredColorData, pFilteredNormalData,
            pFilteredIntensityData, indexVector, numClouds, needColor, needNormal,
            needIntensity, indexVecSize, sizeArray, total);
}
///////////////////////////////////////////////////////////////////////////
// voxelGridFilter Wrapper for double dataType, uint16 Color and uint16 Intensity
///////////////////////////////////////////////////////////////////////////
void voxelGridAlgImpl_double_uint16Color_uint16Intensity(const double* pData,
        const uint16_T* pColor,
        const double* pNormal,
        const uint16_T* pIntensity,
        double* pFilteredData,
        uint16_T* pFilteredColorData,
        double* pFilteredNormalData,
        uint16_T* pFilteredIntensityData,
        const uint64_T* indexVector,
        const uint32_T numClouds,
        const boolean_T needColor,
        const boolean_T needNormal,
        const boolean_T needIntensity,
        const uint32_T indexVecSize,
        const uint32_T* sizeArray,
        const uint32_T total){
    
    voxelGridAlgImpl<double, uint16_T, uint16_T>(pData, pColor, pNormal, pIntensity,
            pFilteredData, pFilteredColorData, pFilteredNormalData,
            pFilteredIntensityData, indexVector, numClouds, needColor, needNormal,
            needIntensity, indexVecSize, sizeArray, total);
}

#endif

