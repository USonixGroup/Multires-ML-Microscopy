/* Copyright 2013-2024 The MathWorks, Inc. */
////////////////////////////////////////////////////////////////////////////////
// This file contains utility functions for OCR metadata.
////////////////////////////////////////////////////////////////////////////////

// REVIEW: moved metadata_utils.cpp from ocrutils/export to ocrutils/
#include "ocrutils/metadata_utils.hpp"
#include "ocrutils/Metadata.hpp"
#include "ocrutils/MetadataCollection.hpp"

////////////////////////////////////////////////////////////////////////////////
// Extract metadata contents to arrays of primitive types for bounding box, 
// parent index, confidence, and character index.
////////////////////////////////////////////////////////////////////////////////
void copyMetadata(const MetadataVector & metadata,
                  double * bbox, int * parentIndex,
                  float * confidence, int * characterIndex,
                  const bool isCharacter)
{
    
    const size_t numel = metadata.size();
    for (size_t i = 0; i < numel; ++i)
    {
        setBBox(bbox++, metadata[i].getBBox(), numel);

        *parentIndex++ = metadata[i].getParentPosition();
	
        *confidence++  = metadata[i].getConfidence();

        if (!isCharacter)
        {
	   setCharacterIndex(characterIndex++, metadata[i], numel);
        }
                
    }            
}

///////////////////////////////////////////////////////////////////////////////
// Get text from metadata and concatenate it into one long string. This is used
// when tesseract's text data does not match the metadata we collect
// (g1149203). 
///////////////////////////////////////////////////////////////////////////////
std::string getTextFromMetadata(const MetadataVector & metadata)
{
    std::string utf8String("");
    for(size_t i = 0; i < metadata.size(); ++i)
    {
        auto str = metadata[i].getUTF8String();
        utf8String += str;
    }
    return utf8String;
}
    

