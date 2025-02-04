//////////////////////////////////////////////////////////////////////////////
//
// APIs for interfaces for ndtUtils that are used in codegeneration
// Copyright 2019-2024 The MathWorks, Inc.
//
//////////////////////////////////////////////////////////////////////////////
// vision_builtins does not need this source file

#include "ndtUtilsCore_api.hpp"
#include "ndtUtils.hpp"


void computeJaHa_single(void* mPose, void* mR, void* mJa, void* mHa)
{
    vision::ndt::computeJaHa<float>(static_cast<float*>(mPose), static_cast<float*>(mR), static_cast<float*>(mJa), static_cast<float*>(mHa));
}

void computeJaHa_double(void* mPose, void* mR, void* mJa, void* mHa)
{
    vision::ndt::computeJaHa<double>(static_cast<double*>(mPose), static_cast<double*>(mR), static_cast<double*>(mJa),
            static_cast<double*>(mHa));
}

void computeScoreGradient_single(void* mIndices, void* p, boolean_T needsHessian, void* Ja,
        void* Ha, void* Jp, void* Hp, void* mvals, size_t numMvals, void* iCov,
        void* hessian, float d1, float d2, void* score, void* g, void* h, void* q, void* grad, size_t numIndices)
{
    vision::ndt::computeScoreGradient<float>(static_cast<uint32_T*>(mIndices), static_cast<float*>(p) , needsHessian,
            static_cast<float*>(Ja), static_cast<float*>(Ha), static_cast<float*>(Jp), static_cast<float*>(Hp),
            static_cast<float*>(mvals), numMvals, static_cast<float*>(iCov), static_cast<float*>(hessian),
            d1, d2, static_cast<float*>(score), static_cast<float*>(g), static_cast<float*>(h),
            static_cast<float*>(q), static_cast<float*>(grad), numIndices);
}

void computeScoreGradient_double(void* mIndices, void* p, boolean_T needsHessian, void* Ja,
        void* Ha, void* Jp, void* Hp, void* mvals, size_t numMvals, void* iCov,
        void* hessian, double d1, double d2, void* score, void* g, void* h, void* q, void* grad, size_t numIndices)
{
    vision::ndt::computeScoreGradient<double>(static_cast<uint32_T*>(mIndices), static_cast<double*>(p), needsHessian,
            static_cast<double*>(Ja), static_cast<double*>(Ha), static_cast<double*>(Jp), static_cast<double*>(Hp),
            static_cast<double*>(mvals), numMvals, static_cast<double*>(iCov), static_cast<double*>(hessian),
            d1, d2, static_cast<double*>(score), static_cast<double*>(g), static_cast<double*>(h),
            static_cast<double*>(q), static_cast<double*>(grad), numIndices);
}