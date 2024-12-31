////////////////////////////////////////////////////////////////////////////////
// Declaring the builtin functions exposed by this module, along with module
// initialization and module termination. When adding new functions, they need
// to be declared here.
//
// Copyright 2022-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef G2OCVLIB_H
#define G2OCVLIB_H

#include "mcos.h"
#include "version.h"
#include "mex.h"

#define DECLARE_FCN(NAME) void NAME(int nlhs, mxArray *plhs[], \
                                    int nrhs, const mxArray *prhs[])

/// @mwModuleInitialize
DLL_EXPORT_SYM void MLInitialize_g2ocv()
{
    
    mcos::COSNamespace * pkg(mcos::COSNamespace::getNamespace("vision.internal"));
    if (pkg == NULL)
    {
        // the +vision/+internal directory should always exist
        assert(false);
    }
}

/// @mwModuleTerminate
DLL_EXPORT_SYM void MLTerminate_g2ocv(){}


#endif
