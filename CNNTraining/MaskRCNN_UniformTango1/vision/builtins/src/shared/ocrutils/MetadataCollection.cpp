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

// REVIEW: moved MetadataCollection.cpp from ocrutils/export to ocrutils/
#include "ocrutils/MetadataCollection.hpp"
#include "ocrutils/Metadata.hpp"

#ifdef COMPILE_FOR_VISION_BUILTINS
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

#include <tesseract/baseapi.h>

namespace ocrutils
{
    
    ////////////////////////////////////////////////////////////////////////////////
    // Splits a UTF-8 symbol, which may contain multiple UTF-8 characters, into
    // an array of strings. Also returns the number of UTF-8 chars that make up
    // the symbol.  
    //
    // Valid UTF-8 characters start with values < 128 or between 192 and 252.
    // See the UTF-8 spec on-line for more info.
    ////////////////////////////////////////////////////////////////////////////////
    int splitUTF8Symbol(const char * str,
            std::vector<std::string> & utf8Strings)
    {
        
        if (str == nullptr)
            return 0;

        // initialize start and end to beginning of string
        auto start = str;
        auto end   = str;
        
        // lambda to determine start of UTF-8 char
        auto isStartOfUTF8Char = [](const char * s) {
            auto strValue = (unsigned char)(*s);
            return (strValue < 128 || (strValue >= 192 && strValue <= 252));
        };

        int count = 0;
        // Start at first char and increment count.
        if ((*str != '\0') && isStartOfUTF8Char(str))
        {
            // start of utf8 char
            count++;			
            str++; // move to next char
        }

        // Process remaining chars, increment count, and store UTF-8 symbol as strings
        while (*str != '\0') {
            if (isStartOfUTF8Char(str))
			{
                // start of another utf8 char also end of last one
                end = str;

                utf8Strings.push_back(std::string(start,end));
                
                start = str; // reset start
                count++;			
            }
            str++; // onto next char
        }

        // add last utf-8 symbol
        utf8Strings.push_back(std::string(start,str));

        return count; 
    }
   
    void MetadataCollection::assignSizes(int32_T * numChars,
                    int32_T * numWords, int32_T * numTextLines,
                    int32_T * numParagraphs, int32_T * numBlocks) const
    {
        *numChars      = static_cast<int32_T>(mCharacters.size());
        *numWords      = static_cast<int32_T>(mWords.size());
        *numTextLines  = static_cast<int32_T>(mTextLines.size());
        *numParagraphs = static_cast<int32_T>(mParagraphs.size());
        *numBlocks     = static_cast<int32_T>(mBlocks.size());
    }

    
    ////////////////////////////////////////////////////////////////////////////////
    // Collect metadata from tesseract's results. Collection proceeds by
    // looping into the OCR results from the top down: block -> paragraph ->
    // text line -> word -> symbol. As we make our way down, we gather metadata
    // for each item.
    ////////////////////////////////////////////////////////////////////////////////
    void MetadataCollection::collectMetadata(tesseract::ResultIterator * results)
    {

        const tesseract::PageIteratorLevel block     = tesseract::RIL_BLOCK;
        const tesseract::PageIteratorLevel paragraph = tesseract::RIL_PARA;
        const tesseract::PageIteratorLevel textline  = tesseract::RIL_TEXTLINE;
        const tesseract::PageIteratorLevel word      = tesseract::RIL_WORD;
        const tesseract::PageIteratorLevel symbol    = tesseract::RIL_SYMBOL;

        if (results != 0)
        {
            results->Begin(); // move to beginning
            tesseract::ResultIterator blockIter  = *results;

            WordMetadata * currentWord;
            PageMetadata page;
            int index = 1;

            int blockCount(0),paragraphCount(0),textlineCount(0),wordCount(0);

            // Start iterating through the results. We start at the top level
            // from the blocks and iterate down to the symbols (a.k.a the
            // characters.

            do // iterate over blocks
            {
                if (blockIter.Empty(block)) continue;

                blockCount++;

                BlockMetadata b(blockIter, block, index, 0, blockCount);
                mBlocks.push_back(b);
                BlockMetadata * currentBlock = &mBlocks.back();

                tesseract::ResultIterator paraIter = blockIter; // start of paragraphs in block

                do // iterate over paragraphs until we reach start of next block
                {
                    if (paraIter.Empty(paragraph)) continue;

                    paragraphCount++;
                    ParagraphMetadata p(paraIter, paragraph, index, blockCount, paragraphCount);
                    mParagraphs.push_back(p);
                    ParagraphMetadata * currentParagraph = &mParagraphs.back();

                    tesseract::ResultIterator tlIter = paraIter; // start of first text line in paragraph
                    do // iterate over textlines
                    {
                        if (tlIter.Empty(textline)) continue;
                        textlineCount++;
                        TextLineMetadata tl(tlIter, textline, index, paragraphCount, textlineCount);

                        mTextLines.push_back(tl);
                        TextLineMetadata * currentTextLine = &mTextLines.back();
                        tesseract::ResultIterator wordIter = tlIter; // start of first word in text line

                        bool hasWordsLeftInTextline(0); // exit condition for following do-while
                        do // iterate over words
                        {
                            if (wordIter.Empty(word)) continue;
                            wordCount++;
                            WordMetadata w(wordIter, word, index, textlineCount, wordCount);

                            mWords.push_back(w);
                            currentWord = &mWords.back();
                            tesseract::ResultIterator symIter = wordIter; // start of first symbol in word
                            do // iterate over symbols until we point to start of next word
                            {
                                if (symIter.Empty(symbol)) continue;

                                // check number of UTF-8 encode characters.
                                // Indic languages may have 1 symbol that is
                                // made up of more than one UTF-8 character.
                                // In this case we must count the symbol
                                // multiple times so that we remain consistent
                                // with our Text array in MATLAB. 
                                char * s = symIter.GetUTF8Text(symbol);
                                std::vector<std::string> utf8Strings;
                                int numChars = splitUTF8Symbol(s,utf8Strings);

                                // add first character, this will have the full symbol
                                // bounding box
                                SymbolMetadata sym(symIter,symbol, index++, wordCount,0, utf8Strings[0].c_str()); 
                                mCharacters.push_back(sym);
                                currentWord->incrementByElementSize(sym);
                                
                                // If the symbol is made up of more than one
                                // UTF-8 character, then add the remaining
                                // character modifiers for this symbol.
                                const bool isModifier = true;
                                for (int i = 1; i < numChars; i++) {
                                    SymbolMetadata modifier(symIter, symbol,
                                            index++, wordCount, 0, utf8Strings[i].c_str(),
                                            isModifier);
                                    mCharacters.push_back(modifier);
                                    currentWord->incrementByElementSize(modifier);
                                }
                                delete[] s;

                                // The following while condition will continue
                                // if we have more symbols to process in the
                                // current word.  Or stop if we have reached
                                // the end of the current word and are now at
                                // the start of the next word. 

                            } while(symIter.Next(symbol) && !symIter.IsAtBeginningOf(word)); // end symbol 
                            currentTextLine->incrementByElementSize(*currentWord);

                            // Add place holder for space or new line
                            // character. Tesseract does not include spaces or
                            // new lines it's results. Therefore, inorder to
                            // keep the raw text buffer, which does include
                            // spaces and newline characters, consistent with
                            // the metadata buffers, we add place holders for
                            // spaces between words or newlines at the end of
                            // text lines.

                            hasWordsLeftInTextline = wordIter.Next(word) && !wordIter.IsAtBeginningOf(textline);
                            bool isSpace = hasWordsLeftInTextline;
                             
                            SymbolMetadata spaceOrNewLine(symbol, index++,
                                    wordCount,0,isSpace);
                            mCharacters.push_back(spaceOrNewLine);

                        } while(hasWordsLeftInTextline); // end word

                        currentParagraph->incrementByElementSize(*currentTextLine);

                    } while(tlIter.Next(textline) && !tlIter.IsAtBeginningOf(paragraph)); // end text line

                    currentBlock->incrementByElementSize(*currentParagraph);

                    // add new line symbol place holder for new line that
                    // occurs at the end of a line.
                    SymbolMetadata newLine(symbol, index++, wordCount,0);
                    mCharacters.push_back(newLine);

                } while(paraIter.Next(paragraph) && !paraIter.IsAtBeginningOf(block)); // end paragraph

                page.incrementByElementSize(*currentBlock);

            } while (blockIter.Next(block)); // end block
        }  
    }
} // end namespace ocrutils

