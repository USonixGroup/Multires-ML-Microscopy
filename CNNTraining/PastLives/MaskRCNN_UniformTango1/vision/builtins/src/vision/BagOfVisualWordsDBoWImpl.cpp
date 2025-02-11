////////////////////////////////////////////////////////////////////////////////
//  BagOfVisualWordsDBoWImpl.cpp
//
//  This is the implementation of Bag of Visual Words.
//
//  Copyright 2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#include "BagOfVisualWordsDBoWImpl.hpp"

using namespace vision;
using namespace DBoW2;

// The following warning suppressions are required for codegen on windows.
// The warnings are coming from 3p/dbow, so we can't fix them.
#if defined _MSC_VER
#pragma warning( disable: 4244 )
#pragma warning( disable: 4251 )
#pragma warning( disable: 4267 )
#pragma warning( disable: 4701 )
#endif

BagOfVisualWordsDBoWImpl::BagOfVisualWordsDBoWImpl(const std::vector<std::vector<cv::Mat>>& _features, const int _depthLevel, const int _branchingFactor, const std::string& _normalization)
{
    depthLevel = _depthLevel;
    branchingFactor = _branchingFactor;
    normalization = _normalization;

    ScoringType scoring;
    if(normalization == "L1") {
        scoring = L1_NORM;
    }
    else{
        scoring = L2_NORM;
    }
    vocabulary = OrbVocabularySerializable(branchingFactor, depthLevel, TF_IDF, scoring);
    vocabulary.create(_features);
}

BagOfVisualWordsDBoWImpl::BagOfVisualWordsDBoWImpl(std::stringstream& ifs) {
    vocabulary.loadFromFileStream(ifs);
    
    depthLevel = vocabulary.getDepthLevels();
    branchingFactor = vocabulary.getBranchingFactor();
    normalization = "L1";
    if (vocabulary.getScoringType() == L2_NORM) {
      normalization = "L2";
    }
}

BagOfVisualWordsDBoWImpl::BagOfVisualWordsDBoWImpl(const std::string& vocabFile)
{
    try {
        vocabulary.loadFromTextFile(vocabFile);
    } catch(...) {
        try {
            vocabulary.load(vocabFile); // YAML
        } catch(...) {
            vocabulary.loadFromBinaryFile(vocabFile);
        }
    }
    depthLevel = vocabulary.getDepthLevels();
    branchingFactor = vocabulary.getBranchingFactor();
    normalization = "L1";
    if (vocabulary.getScoringType() == L2_NORM) {
      normalization = "L2";
    }
}

std::pair<int, int> BagOfVisualWordsDBoWImpl::getDepthAndBranching()
{
    return std::make_pair(depthLevel,branchingFactor);
}

std::string BagOfVisualWordsDBoWImpl::getNormalization() const
{
    return normalization;
}

std::string BagOfVisualWordsDBoWImpl::getSerializedBag()
{
    std::stringstream buffer;
    vocabulary.saveToFileStream(buffer);
    std::string retval = buffer.str();
    return retval;
}