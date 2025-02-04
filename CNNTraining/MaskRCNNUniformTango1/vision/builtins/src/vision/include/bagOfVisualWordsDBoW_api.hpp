/* Copyright 2024 The MathWorks, Inc. */

#ifndef _BAG_OF_VISUAL_WORDS_DBOW_API_
#define _BAG_OF_VISUAL_WORDS_DBOW_API_

#include "vision_defines.h"

EXTERN_C LIBMWCVSTRT_API
void* DBoW_Bag_createBagFromFile(const char* bagFile, int32_T* depthLevel, 
                                 int32_T* branchingFactor, const char* normalization);

EXTERN_C LIBMWCVSTRT_API
void DBoW_Bag_getSerializedBag(void* objPtr, const char* serializedBag);

#endif
