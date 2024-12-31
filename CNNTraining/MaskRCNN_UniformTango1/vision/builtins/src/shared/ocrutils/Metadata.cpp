/* Copyright 2013-2024 The MathWorks, Inc. */
////////////////////////////////////////////////////////////////////////////////
// This file defines the OCR Metadata class that holds metadata for exactly
// one item such as a single character or a single word or a single text
// line, etc.
// 
// The metadata for each item is then aggregated and stored in
// individual Metadata vectors (i.e. vector<Metadata>) for each of the 5
// item types: characters, words, text lines, paragraphs, and blocks. This
// collection class is defined in the MetadataCollection.hpp file. We use
// Metadata as an intermediate class to hold the data to simplify the data
// collection process.
//
// The information included in the metadata is the bounding box location,
// confidence, parent of an element (e.g. which character does this word
// belong to), and the index into the character array (e.g. what is the
// starting and ending index within the raw character array for this word).
////////////////////////////////////////////////////////////////////////////////

// REVIEW: moved Metadata.cpp from ocrutils/export to ocrutils/
#include "ocrutils/Metadata.hpp"

#include <tesseract/baseapi.h>
#include <vector>
#include <string>

namespace ocrutils 
{
       
    typedef std::vector<int> BBoxVector;

    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    ////////////////////////////////////////////////////////////////////////////////
    Metadata::Metadata(const tesseract::ResultIterator & it,
            const tesseract::PageIteratorLevel level,
            int start, int parentPosition, int myPosition, 
            const char * utf8String, const bool isSymbolModifier)
    {
        this->mPageLevel      = level;
        this->mMyPosition     = myPosition;
        this->mParentPosition = parentPosition;

        this->mStart = start;
        this->mEnd   = start;


        // There appears to be a bug in Tesseract. Whenever this is no good
        // choice for a character, a space is returned. Trying to get the
        // symbol confidence in this situation leads to an assertion in
        // Tesseract. So to avoid this for symbols, we get the text and if it's
        // a space , then we set the confidence to the same as the whole word.
        // This keeps the char confidence consistent with the word confidence
        // in cases where a word is just a space.  
        if (level == tesseract::RIL_SYMBOL)
        {
        
            this->setBBox(it, level, isSymbolModifier);

            if(utf8String != NULL)
            {
                if (*utf8String == ' ')
                {
                    // assign this degenerate symbol it's word confidence
                    this->mConfidence = it.Confidence(tesseract::RIL_WORD);
                }
                else
                {
                    this->mConfidence = it.Confidence(level);
                }   

                // Store symbol chars for g1149203. Another bug in tesseract
                // where it seems to be inserting characters that don't appear
                // to be in the metadata.
                this->mUTF8String = std::string(utf8String);
            }
            else 
            {
                // There is no text! Something went really haywire. Set
                // confidence to -1 to treat this as a space. This code is here
                // just to be safe. We should never be able to get into this
                // state because it would mean the result iterator is pointing
                // a word of length zero.                
                this->mConfidence = -1.0F; 
                this->mUTF8String = " ";
            }
        }
        else 
        {   
            this->setBBox(it,level);
            this->mConfidence = it.Confidence(level);                
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Constructor for white space and new line characters. These are "imaginary"
    // characters that we need to account for so that the indexing works out in the
    // end. These characters are assigned a confidence of -1, which we convert to
    // NaNs in MATLAB.
    ////////////////////////////////////////////////////////////////////////////////
    Metadata::Metadata(const tesseract::PageIteratorLevel level,
            int start, int parentPosition, int myPosition, bool isSpace)
    {
        this->mPageLevel      = level;
        this->mMyPosition     = myPosition;
        this->mParentPosition = parentPosition;

        // set artificial conf for spaces and new lines. These get set
        // to NaN in MATLAB.
        this->mConfidence = isSpace ? -2.0F : -1.0F; 
        this->mUTF8String = isSpace ? " " : "\n";

        this->mStart = start;
        this->mEnd   = start;

        // Spaces and new-lines are not assigned bounding boxes by tesseract.
        // The bboxes for spaces are computed in MATLAB.
        this->mBBox.push_back(0);
        this->mBBox.push_back(0);
        this->mBBox.push_back(0);
        this->mBBox.push_back(0); 

    }

    ////////////////////////////////////////////////////////////////////////////////
    // Increase the "size" of an element by another element. This is used to update
    // the start and end index of an element. When a word is added to a text line
    // we need to update the text lines starting and ending index by the length of
    // the word. Similarly, when we add text lines to paragraphs and paragraphs to
    // blocks. 
    ////////////////////////////////////////////////////////////////////////////////
    void Metadata::incrementByElementSize(const Metadata & elem)
    {
        switch (this->mPageLevel)
        {
            case tesseract::RIL_BLOCK : // block.paragraph
                this->mEnd = elem.getEndIndex();
                break;
            case tesseract::RIL_PARA :
                // +1 for space at end of word/new-line end of paragraph
                this->mEnd = elem.getEndIndex() + 1;
                break;
            case tesseract::RIL_TEXTLINE :
                // +1 for space at end of word/new-line end of paragraph
                this->mEnd = elem.getEndIndex() + 1; 
                break;
            case tesseract::RIL_WORD :
                this->mEnd++;
                break;
            case tesseract::RIL_SYMBOL:
                this->mEnd++;
                break;
            default:
                this->mEnd++;
        }
    }

} // end namespace ocrutils
