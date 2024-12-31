////////////////////////////////////////////////////////////////////////////////
// BagOfVisualWordsDBoWImpl.hpp
//
// Header file for the implementation of BagOfVisualWordsDBoW 
//
// Copyright 2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef BAGOFVISUALWORDSDBOWIMPL_HPP
#define BAGOFVISUALWORDSDBOWIMPL_HPP

#include "DBoW2.h"
#include "TemplatedVocabularySerializable.hpp"

namespace vision {

    class BagOfVisualWordsDBoWImpl {
    public:
        BagOfVisualWordsDBoWImpl() = delete;

        /**
        * @brief Constructor using features
        */
        BagOfVisualWordsDBoWImpl(const std::vector<std::vector<cv::Mat>>& features, const int depthLevel, const int branchingFactor,const std::string& normalization);

        /**
        * @brief Overloaded Constructor using vocabulary file
        */
        explicit BagOfVisualWordsDBoWImpl(const std::string& vocabFile);

        /**
        * @brief Overloaded Constructor using string stream
        */
        explicit BagOfVisualWordsDBoWImpl(std::stringstream& ifs);

        /**
        * @brief Getter for Normalization
        *
        * @return Normalization as a string
        *
        */
        std::string getNormalization() const;

        /**
        * @brief Getter for Serialized bag.
        *
        * @return Serialized bag as a string
        *
        */
        std::string getSerializedBag();

        /**
        * @brief Getter for Tree Properties
        *
        * @return Depth Levels and Branching Factors as a pair
        *
        */
        std::pair<int, int> getDepthAndBranching();

    private:

        /**
        * DBoW2 Templated Vocabulary
        */
        OrbVocabularySerializable vocabulary;

        /**
        * Vocabulary tree depth levels
        */
        int depthLevel;

        /**
        * Vocabulary tree branchign factor
        */
        int branchingFactor;

        /**
        * Vocabulary tree normalization method
        */
        std::string normalization;
    };
}

#endif