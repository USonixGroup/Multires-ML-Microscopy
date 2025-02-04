////////////////////////////////////////////////////////////////////////////////
//  This header contains assertion macro for Kdtree module
//  Copyright 2018 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef KDTREE_UTIL_HPP
#define KDTREE_UTIL_HPP

#include <assert.h>
#include <stdio.h>

#if defined(NDEBUG)
#define VISION_ASSERT(EXPR) (void)0
#else
#define VISION_ASSERT(EXPR) assert(EXPR)
#endif

#if defined(NDEBUG)
#define VISION_ASSERT_MSG(EXPR, MSG) (void)0
#else
#define VISION_ASSERT_MSG(EXPR, MSG)            \
do {                                            \
        if (! (EXPR)) {                         \
                printf("%s \n",MSG);            \
                assert((EXPR));                 \
        }                                       \
} while (false)
#endif
    
    template<typename DataType>
    void copyToArray_RowMajor(DataType *src, DataType *dst, int numDstRows, int numDstCols)
    {
        for( int i = 0; i < numDstCols; i++ )
        {
            for( int j = 0; j < numDstRows; j++ )
            {
                dst[j*numDstCols+i] = *src++;
            }
        }
    }
#endif
