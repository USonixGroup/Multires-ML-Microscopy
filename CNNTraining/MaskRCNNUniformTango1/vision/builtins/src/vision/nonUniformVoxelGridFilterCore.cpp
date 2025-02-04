///////////////////////////////////////////////////////////////////////////
//
//       APIs for nonUniformVoxelGridFilter
//      Copyright 2018 The MathWorks, Inc.
//
///////////////////////////////////////////////////////////////////////////
#ifndef COMPILE_FOR_VISION_BUILTINS

#include "nonUniformVoxelGridFilterCore_api.hpp"
#include "nonUniformVoxelGridFilter.hpp"

///////////////////////////////////////////////////////////////////////////
//     nonUniformVoxelGridFilter wrapper for single datatype
///////////////////////////////////////////////////////////////////////////
uint32_T nonUniformVoxelGridFilterImpl_single(void** filterObj,
        const float* pData,
        uint32_T numPoints,
        double maxVoxelPoints,
        bool needNormals)
{
    NonUniformVoxelGridAlgImpl<float>* algorithm = new NonUniformVoxelGridAlgImpl<float>();
    *filterObj = (void*)algorithm;
    return algorithm->execute(pData, numPoints, maxVoxelPoints, needNormals);
}

///////////////////////////////////////////////////////////////////////////
//     nonUniformVoxelGridFilter wrapper for double datatype
///////////////////////////////////////////////////////////////////////////
uint32_T nonUniformVoxelGridFilterImpl_double(void** filterObj,
        double* pData,
        uint32_T numPoints,
        double maxVoxelPoints,
        bool needNormals)
{
    NonUniformVoxelGridAlgImpl<double>* algorithm = new NonUniformVoxelGridAlgImpl<double>();
    *filterObj = (void*)algorithm;
    return algorithm->execute(pData, numPoints, maxVoxelPoints, needNormals);
}

///////////////////////////////////////////////////////////////////////////
//     To assign nonUniformVoxelGridFilter outputs for single datatype
///////////////////////////////////////////////////////////////////////////
void nonUniformVoxelGridFilterImplAssignOutputs_single(void* filterObj,
        uint32_T* pIndices,
        float* pNormal,
        bool needNormals)
{
    NonUniformVoxelGridAlgImpl<float>* algorithm = (NonUniformVoxelGridAlgImpl<float>*)filterObj;
    
    if (needNormals) {
        algorithm->getOutputs(pIndices, pNormal);
    } else {
        algorithm->getOutputs(pIndices);
    }
    
    if(filterObj != nullptr){
        delete((NonUniformVoxelGridAlgImpl<float>*)filterObj);
        filterObj = nullptr;
    }
}

///////////////////////////////////////////////////////////////////////////
//     To assign nonUniformVoxelGridFilter outputs for double datatype
///////////////////////////////////////////////////////////////////////////
void nonUniformVoxelGridFilterImplAssignOutputs_double(void* filterObj,
        uint32_T* pIndices,
        double* pNormal,
        bool needNormals)
{
    NonUniformVoxelGridAlgImpl<double>* algorithm = (NonUniformVoxelGridAlgImpl<double>*)filterObj;
    
    if (needNormals) {
        algorithm->getOutputs(pIndices, pNormal);
    } else {
        algorithm->getOutputs(pIndices);
    }
    
    if(filterObj != nullptr) {
        delete((NonUniformVoxelGridAlgImpl<double>*)filterObj);
        filterObj = nullptr;
    }
}
#endif
