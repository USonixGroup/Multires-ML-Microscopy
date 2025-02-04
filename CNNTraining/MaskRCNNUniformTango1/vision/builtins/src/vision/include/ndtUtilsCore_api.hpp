/* Copyright 2019-2024 The MathWorks, Inc. */

#ifndef _NDTUTILSCORE_API_HPP
#define _NDTUTILSCORE_API_HPP

#include "vision_defines.h"
#include <stddef.h> // Scope for size_t for portable codegen

/* NDT ComputeScoreDerivatives */
EXTERN_C LIBMWCVSTRT_API
        void computeJaHa_single(void* mPose, void* mR, void* mJa, void* mHa);

EXTERN_C LIBMWCVSTRT_API
        void computeJaHa_double(void* mPose, void* mR, void* mJa, void* mHa);

EXTERN_C LIBMWCVSTRT_API
        void computeScoreGradient_single(void* mIndices, void* p, boolean_T needsHessian, void* Ja,
        void* Ha, void* Jp, void* Hp, void* mvals, size_t numMvals, void* iCov,
        void* hessian, float d1, float d2, void* score, void* g, void* h, void* q, void* grad, size_t numIndices);

EXTERN_C LIBMWCVSTRT_API
        void computeScoreGradient_double(void* mIndices, void* p, boolean_T needsHessian, void* Ja,
        void* Ha, void* Jp, void* Hp, void* mvals, size_t numMvals, void* iCov,
        void* hessian, double d1, double d2, void* score, void* g, void* h, void* q, void* grad, size_t numIndices);

#endif