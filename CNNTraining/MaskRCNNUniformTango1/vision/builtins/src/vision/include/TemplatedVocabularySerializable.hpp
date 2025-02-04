////////////////////////////////////////////////////////////////////////////////
// TemplatedVocabularySerializable.hpp
//
// Header file for the serilizable interface for DBoW2's Vocabulary
//
// Copyright 2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef __D_T_TEMPLATED_VOCABULARY_SERIALIZABLE__
#define __D_T_TEMPLATED_VOCABULARY_SERIALIZABLE__

#include <cassert>
#include <cstdlib>
#include <vector>
#include <numeric>
#include <fstream>
#include <string>
#include <algorithm>

#include <opencv2/core.hpp>

#ifdef COMPILE_FOR_VISION_BUILTINS
#include "DBoW2/DBoW2.h"
#else
#include "DBoW2.h"
#endif

#ifdef _MSC_VER
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT
#endif

namespace DBoW2 {

    /**
    * Generic Serializable Vocabulary
    * @param TDescriptor class of descriptor
    * @param F class of descriptor functions
    */
    template<class TDescriptor, class F>
        class DLL_EXPORT TemplatedVocabularySerializable : public TemplatedVocabulary<TDescriptor, F> {
            public:
            /**
            * Initiates an empty vocabulary
            * @param k branching factor
            * @param L depth levels
            * @param weighting weighting type
            * @param scoring scoring type
            */
            TemplatedVocabularySerializable(const int k = 10, const int L = 5,
                                            const WeightingType weighting = TF_IDF, const ScoringType scoring = L1_NORM);

            /**
            * Creates the vocabulary by loading a file
            * @param filename
            */
            TemplatedVocabularySerializable(const std::string& filename);

            /**
            * Creates the vocabulary by loading a file
            * @param filename
            */
            TemplatedVocabularySerializable(const char* filename);

            /**
            * Copy constructor
            * @param voc
            */
            TemplatedVocabularySerializable(const TemplatedVocabularySerializable<TDescriptor, F>& voc);

            /**
            * Destructor
            */
            virtual ~TemplatedVocabularySerializable();

            /**
            * Assigns the given vocabulary to this by copying its data and removing
            * all the data contained by this vocabulary before
            * @param voc
            * @return reference to this vocabulary
            */
            TemplatedVocabularySerializable<TDescriptor, F>& operator=(const TemplatedVocabularySerializable<TDescriptor, F>& voc);

            /**
            * Loads the vocabulary from a stream
            * @param ifs
            */
            void loadFromFileStream(std::istream& ifs);

            /**
            * Saves the vocabulary to a stream
            * @param ifs
            */
            void saveToFileStream(std::ostream& ofs);
        };

    // --------------------------------------------------------------------------

    template<class TDescriptor, class F>
        TemplatedVocabularySerializable<TDescriptor, F>::TemplatedVocabularySerializable(const int k, const int L,
                                                                                         const WeightingType weighting, const ScoringType scoring)
            : TemplatedVocabulary<TDescriptor, F>(k, L, weighting, scoring) { }

    // --------------------------------------------------------------------------

    template<class TDescriptor, class F>
        TemplatedVocabularySerializable<TDescriptor, F>::TemplatedVocabularySerializable(const std::string& filename)
            : TemplatedVocabulary<TDescriptor, F>(filename) { }

    // --------------------------------------------------------------------------

    template<class TDescriptor, class F>
        TemplatedVocabularySerializable<TDescriptor, F>::TemplatedVocabularySerializable(const char* filename)
            : TemplatedVocabulary<TDescriptor, F>(filename) { }

    // --------------------------------------------------------------------------

    template<class TDescriptor, class F>
        TemplatedVocabularySerializable<TDescriptor, F>::TemplatedVocabularySerializable(
            const TemplatedVocabularySerializable<TDescriptor, F>& voc)
            : TemplatedVocabulary<TDescriptor, F>(voc) { }

    // --------------------------------------------------------------------------

    template<class TDescriptor, class F>
        TemplatedVocabularySerializable<TDescriptor, F>::~TemplatedVocabularySerializable() { }

    // --------------------------------------------------------------------------

    template<class TDescriptor, class F>
        TemplatedVocabularySerializable<TDescriptor, F>&
        TemplatedVocabularySerializable<TDescriptor, F>::operator=(const TemplatedVocabularySerializable<TDescriptor, F>& voc) {

            //
            // The following code is sourced from 3p/DBoW2. Investigate if 
            // we can resuse Base class operator= function. (g3256266)
            //

            this->m_k = voc.m_k;
            this->m_L = voc.m_L;
            this->m_scoring = voc.m_scoring;
            this->m_weighting = voc.m_weighting;

            this->createScoringObject();

            this->m_nodes.clear();
            this->m_words.clear();

            this->m_nodes = voc.m_nodes;
            this->createWords();

            return *this;
        }

    // --------------------------------------------------------------------------

    template<class TDescriptor, class F>
        void TemplatedVocabularySerializable<TDescriptor, F>::loadFromFileStream(std::istream& ifs) {

            //
            // The following code is sourced from 3p/DBoW2
            //

            this->m_words.clear();
            this->m_nodes.clear();

            std::string s;
            getline(ifs, s);
            std::stringstream ss;
            ss << s;
            int mk = 0, ml = 0; // These variables are important to avoid compiler optimizations on them. (g3258085)
            ss >> mk;
            ss >> ml;
            this->m_k = mk;
            this->m_L = ml;
            int n1, n2;
            ss >> n1;
            ss >> n2;

            if (mk < 0 || 20 < mk || ml < 1 || 10 < ml || n1 < 0 || 5 < n1 || n2 < 0 || 3 < n2) {
                throw std::string("Vocabulary loading failed");
            }

            this->m_scoring = static_cast<ScoringType>(n1);
            this->m_weighting = static_cast<WeightingType>(n2);
            this->createScoringObject();

            const auto expected_nodes = static_cast<int>((std::pow(static_cast<double>(this->m_k), static_cast<double>(this->m_L) + 1.0) - 1) / (this->m_k - 1));
            this->m_nodes.reserve(expected_nodes);

            this->m_words.reserve(static_cast<uint64_t>(std::pow(static_cast<double>(this->m_k), static_cast<double>(this->m_L) + 1.0)));

            this->m_nodes.resize(1);
            this->m_nodes.at(0).id = 0;

            while (!ifs.eof()) {
                std::string s_node;
                getline(ifs, s_node);
                if (s_node == "") {
                    continue;
                }
                std::stringstream ss_node;
                ss_node << s_node;

                const int n_id = static_cast<int>(this->m_nodes.size());
                this->m_nodes.resize(this->m_nodes.size() + 1);
                this->m_nodes.at(n_id).id = n_id;

                int p_id;
                ss_node >> p_id;
                this->m_nodes.at(n_id).parent = p_id;
                this->m_nodes.at(p_id).children.push_back(n_id);

                int is_leaf;
                ss_node >> is_leaf;

                std::stringstream ss_desc;
                for (int i = 0; i < F::L; ++i) {
                    std::string s_desc;
                    ss_node >> s_desc;
                    ss_desc << s_desc << " ";
                }
                F::fromString(this->m_nodes.at(n_id).descriptor, ss_desc.str());

                ss_node >> this->m_nodes.at(n_id).weight;

                if (static_cast<bool>(is_leaf)) {
                    const int w_id = static_cast<int>(this->m_words.size());
                    this->m_words.resize(w_id + 1);

                    this->m_nodes.at(n_id).word_id = w_id;
                    this->m_words.at(w_id) = &this->m_nodes.at(n_id);
                }
                else {
                    this->m_nodes.at(n_id).children.reserve(this->m_k);
                }
            }
        }

    template<class TDescriptor, class F>
        void TemplatedVocabularySerializable<TDescriptor, F>::saveToFileStream(std::ostream& ofs) {

            //
            // The following code is sourced from 3p/DBoW2
            //

            ofs << this->m_k << " " << this->m_L << " "
                << " " << this->m_scoring << " " << this->m_weighting << std::endl;

            for (size_t i = 1; i < this->m_nodes.size(); ++i) {
                const typename TemplatedVocabulary<TDescriptor, F>::Node& node = this->m_nodes.at(i);

                ofs << node.parent << " ";

                if (node.isLeaf()) {
                    ofs << 1 << " ";
                }
                else {
                    ofs << 0 << " ";
                }

                ofs << F::toString(node.descriptor) << " " << static_cast<double>(node.weight) << std::endl;
            }
        }

    // --------------------------------------------------------------------------

    /**
    * Writes printable information of the vocabulary
    * @param os stream to write to
    * @param voc
    */
    template<class TDescriptor, class F>
        std::ostream& operator<<(std::ostream& os,
                                 const TemplatedVocabularySerializable<TDescriptor, F>& voc) {
        return TemplatedVocabulary<TDescriptor, F>::operator<<(voc);
    }

} // namespace DBoW2

/// ORB Vocabulary
typedef DBoW2::TemplatedVocabularySerializable<DBoW2::FORB::TDescriptor, DBoW2::FORB> OrbVocabularySerializable;

#endif