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

#ifndef _METADATA_H_
#define _METADATA_H_

#include <tesseract/baseapi.h>
#include <vector>
#include <string>

namespace ocrutils 
{
       
    typedef std::vector<int> BBoxVector;

    class Metadata
    {
        public:

            Metadata() { }
            
            ////////////////////////////////////////////////////////////////////
            //  Construct Metadata object given inputs:
            //     it: iterator to results
            //     level: position in the page (symbol, word, textline, etc)
            //     start: starting position in the text array. 
            //     parentPosition: parent's position in the collection index
            //     myPosition: object's position in collection index
            //     utf8String: UTF-8 char data for the element. Used for symbol
            //                 processing. 
            //     isSymbolModifier: whether or not symbol is for a character
            //                       modifier which requires special handling
            //                       for bbox data.
            ////////////////////////////////////////////////////////////////////
            Metadata(const tesseract::ResultIterator & it,
                    const tesseract::PageIteratorLevel level,
                    int start, int parentPosition, int myPosition, 
                    const char * utf8String = NULL, 
                    const bool isSymbolModifier = false);

            Metadata(const tesseract::PageIteratorLevel level,
                    int start, int parentPosition, int myPosition, bool isSpace = false);

            void incrementByElementSize(const Metadata & elem);

            inline int getPositionInArray() const
            {
                return mMyPosition;
            }

            inline int getParentPosition() const
            {
                return mParentPosition;
            }

            inline const BBoxVector & getBBox() const
            {
                return mBBox;
            }

            inline int getStartIndex() const
            {
                return mStart;
            }

            inline int getEndIndex() const
            {
                return mEnd;
            }

            inline float getConfidence() const
            {
                return mConfidence;
            }

            inline std::string getUTF8String() const
            {
                return mUTF8String;
            }

            inline void setBBox(const tesseract::ResultIterator & it,
                    const tesseract::PageIteratorLevel level, 
                    const bool isSymbolModifier = false)
            {
                // Bounding box definition in tesseract
                // (left,top)
                //     *-----------
                //     |          |
                //     |          |
                //     -----------* (right,bottom)

                // Description from tesseract's pageiterator.h
                // Coordinate system:
                // Integer coordinates are at the cracks between the pixels.
                // The top-left corner of the top-left pixel in the image is at (0,0).
                // The bottom-right corner of the bottom-right pixel in the image is at
                // (width, height).
                // Every bounding box goes from the top-left of the top-left contained
                // pixel to the bottom-right of the bottom-right contained pixel, so
                // the bounding box of the single top-left pixel in the image is:
                // (0,0)->(1,1).

                int left,top,right,bottom;
                it.BoundingBox(level,&left,&top,&right,&bottom);

                int width  = right - left;
                int height = bottom - top;

                mBBox.push_back( left + 1); // +1 for CVST x,y format
                mBBox.push_back( top  + 1); 

                // Some languages have modifers that change the shape
                // of a character. When this type of modified symbol
                // is present it is made up of more than one UTF-8 character.
                // These modifiers are not valid characters by themselves and only
                // make sense with other characters. Because of this, the width
                // height is set to zero. 
                if (isSymbolModifier) 
                {
                    mBBox.push_back( 0 );
                    mBBox.push_back( 0 );
                }
                else
                {
                    mBBox.push_back( width   );
                    mBBox.push_back( height  );
                }
            }

        private:
            ////////////////////////////////////////////////////////////////////////////
            // The PageIteratorLevel is a tesseract type that defines a level in
            // the page heirarchy. The levels are block, paragraph, text line,
            // word, and character (a.k.a symbol in tesseract). This member
            // variable lets us know what type of page element this metadata is
            // for.
            ////////////////////////////////////////////////////////////////////////////
            tesseract::PageIteratorLevel mPageLevel;

            ////////////////////////////////////////////////////////////////////////////
            // mParentPosition: The position of my parent within the
            // MetadataCollection vector. This allows us to associate a specific
            // character with the word it belongs to and retrieve metadata about
            // the parent.
            ////////////////////////////////////////////////////////////////////////////
            int mParentPosition;

            ////////////////////////////////////////////////////////////////////////////
            // mMyPosition
            ////////////////////////////////////////////////////////////////////////////
            int mMyPosition;

            ////////////////////////////////////////////////////////////////////////////
            // mStart and mEnd are the starting and ending indices into an array
            // that holds all the raw text data. For words, text lines, paragraphs,
            // or blocks, mStart and mEnd provide an easy way to access all the
            // characters that "make up" a word, text line, paragraph, or block.
            ////////////////////////////////////////////////////////////////////////////
            int mStart;
            int mEnd;

            ////////////////////////////////////////////////////////////////////////////
            // mConfidence: This is the classifcation confidence tesseract
            // assigns to the recognized character, word, text line, paragraph,
            // or block. For things like text lines, paragraphs and blocks,
            // tesseract just averages the confidence of the words that make up
            // the line or paragraph. For things like spaces and new lines, we
            // set the confidence to -1 so that we can easily identify spaces
            // and new lines in MATLAB.
            ////////////////////////////////////////////////////////////////////////////
            float mConfidence;

            ////////////////////////////////////////////////////////////////////////////
            //mBBox: This is the bounding box around the item. It is a 4
            //element vector of [x y width height]. The x y coordinate values
            //are 1-based and follow the CVST coordinate system conventions.
            ////////////////////////////////////////////////////////////////////////////
            BBoxVector mBBox;

            ///////////////////////////////////////////////////////////////////////////////
            // mUTF8String: Holds the UTF-8 chars for a symbol. 
            ///////////////////////////////////////////////////////////////////////////////
            std::string mUTF8String;
    };

}
#endif /* _METADATA_H_ */
