/* Copyright 2024 The MathWorks, Inc. */

#ifndef COMPILE_FOR_VISION_BUILTINS

#include "bagOfVisualWordsDBoW_api.hpp"
#include "BagOfVisualWordsDBoWImpl.hpp"
#include <fstream>
#include <string>
#include <vector>
#include "cgCommon.hpp"

using namespace vision;

void* DBoW_Bag_createBagFromFile(const char* bagFile, int32_T* depthLevel, 
                                 int32_T* branchingFactor, const char* normalization) {

    const std::string vocab(bagFile);
    BagOfVisualWordsDBoWImpl* objPtr = new BagOfVisualWordsDBoWImpl(vocab);
    const std::pair<int, int> db = objPtr->getDepthAndBranching();
    depthLevel[0] = db.first;
    branchingFactor[0] = db.second;
    normalization = objPtr->getNormalization().c_str();
    return static_cast<void*>(objPtr);
}

void DBoW_Bag_getSerializedBag(void* objPtr, const char* serializedBag) {
    if(objPtr == nullptr) {
        serializedBag = "";
    } else {
        serializedBag = static_cast<BagOfVisualWordsDBoWImpl*>(objPtr)->getSerializedBag().c_str();
    }
}
#endif
