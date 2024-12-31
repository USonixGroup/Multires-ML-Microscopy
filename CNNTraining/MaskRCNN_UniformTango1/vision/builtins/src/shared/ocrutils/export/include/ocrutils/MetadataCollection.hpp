/* Copyright 2013-2024 The MathWorks, Inc. */
////////////////////////////////////////////////////////////////////////////////
// This file contains the MetadataCollection class. MetadataCollection
// aggregates all metadata for each element of a page: symbols, words, text
// lines, paragraphs, and blocks. The class provides methods to collect
// metadata from tesseracts results, and copy the results into output buffers.
//
// Internally, the MetadataCollection class uses vectors of Metadata objects to
// store the metadata for each individual page element. Because we do not know
// the number of words or characters before we iterate through tesseract's
// results, we need to dynamically aggregate metadata as we traverse
// tesseract's results.
////////////////////////////////////////////////////////////////////////////////
#ifndef _METADATACOLLECTION_H_
#define _METADATACOLLECTION_H_

#include "Metadata.hpp"

#ifdef COMPILE_FOR_VISION_BUILTINS
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

#include <tesseract/baseapi.h>

#include "ocrutils_util.hpp"

namespace ocrutils
{
    typedef std::vector<Metadata> MetadataVector;
    typedef Metadata PageMetadata;
    typedef Metadata BlockMetadata;
    typedef Metadata ParagraphMetadata;
    typedef Metadata TextLineMetadata;
    typedef Metadata WordMetadata;
    typedef Metadata SymbolMetadata;
    
    ////////////////////////////////////////////////////////////////////////////////
    // Splits a UTF-8 symbol, which may contain multiple UTF-8 characters, into
    // an array of strings. Also returns the number of UTF-8 chars that make up
    // the symbol.  
    //
    // Valid UTF-8 characters start with values < 128 or between 192 and 252.
    // See the UTF-8 spec on-line for more info.
    ////////////////////////////////////////////////////////////////////////////////
    OCRUTILS_API
    int splitUTF8Symbol(const char * str,
                        std::vector<std::string> & utf8Strings);
   
    class OCRUTILS_API MetadataCollection
    {
        public:
            void collectMetadata(tesseract::ResultIterator * results);
            void assignSizes(int32_T * numChars,
                    int32_T * numWords, int32_T * numTextLines,
                    int32_T * numParagraphs, int32_T * numBlocks) const;

        public:
            MetadataVector mBlocks;
            MetadataVector mParagraphs;
            MetadataVector mTextLines;
            MetadataVector mWords;
            MetadataVector mCharacters;
    };


 } // end namespace ocrutils

#endif /* _METADATACOLLECTION_H_ */
