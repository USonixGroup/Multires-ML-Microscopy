/* Copyright 2013-2024 The MathWorks, Inc. */
////////////////////////////////////////////////////////////////////////////////
// This file contains utility functions for OCR metadata.
////////////////////////////////////////////////////////////////////////////////

#ifndef _METADATA_UTILS_H_
#define _METADATA_UTILS_H_

#include "Metadata.hpp"
#include "MetadataCollection.hpp"
#include "ocrutils_util.hpp"

using namespace ocrutils;

////////////////////////////////////////////////////////////////////////////////
// Copy bbox information into row of M-by-4 matrix
////////////////////////////////////////////////////////////////////////////////
inline void setBBox(double * bbox,
                    const BBoxVector & bboxVector,
                    const size_t M)
{    
    bbox[0]   = bboxVector[0];
    bbox[M]   = bboxVector[1];
    bbox[2*M] = bboxVector[2];
    bbox[3*M] = bboxVector[3];       
}

////////////////////////////////////////////////////////////////////////////////
// Copy start and end index into character array; charIndex is a pointer to a
// row in an M-by-2 matrix. 
////////////////////////////////////////////////////////////////////////////////
inline void setCharacterIndex(int * charIndex,
                              const Metadata & element,
                              const size_t M)
{
    charIndex[0] = element.getStartIndex();
    charIndex[M] = element.getEndIndex();
}

////////////////////////////////////////////////////////////////////////////////
// Extract metadata contents to arrays of primitive types for bounding box, 
// parent index, confidence, and character index.
////////////////////////////////////////////////////////////////////////////////
OCRUTILS_API
void copyMetadata(const MetadataVector & metadata,
                  double * bbox, int * parentIndex,
                  float * confidence, int * characterIndex,
                  const bool isCharacter = false);

///////////////////////////////////////////////////////////////////////////////
// Get text from metadata and concatenate it into one long string. This is used
// when tesseract's text data does not match the metadata we collect
// (g1149203). 
///////////////////////////////////////////////////////////////////////////////
OCRUTILS_API
std::string getTextFromMetadata(const MetadataVector & metadata);
    

#endif /* _METADATA_UTILS_H_ */
