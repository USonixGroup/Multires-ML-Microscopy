//////////////////////////////////////////////////////////////////////////////
// This file declares all the builtin functions exposed by this module.
// When you add a new function, you must declare it in this header.
//
// Copyright 2022-2024 The MathWorks, Inc.
//
//////////////////////////////////////////////////////////////////////////////
#ifndef vslamlib_h
#define vslamlib_h

#include "VisionInternalPackageInfo.hpp"

//////////////////////////////////////////////////////////////////////////////
// Class registration - specify full class name including namespace
//////////////////////////////////////////////////////////////////////////////
MATLAB_CLS_REG("vision.internal.MonoVisualSLAM");
MATLAB_CLS_REG("vision.internal.RGBDVisualSLAM");
MATLAB_CLS_REG("vision.internal.StereoVisualSLAM");

///////////////////////////////////////////////////////////////
// module initialization
///////////////////////////////////////////////////////////////
/// @mwModuleInitialize 


extern "C" DLL_EXPORT_SYM  void MLInitialize_vslam()
{
    static bool initialized = false;
    if (!initialized)
    {
        mcos::COSNamespace * pkg(mcos::COSNamespace::getNamespace("vision.internal"));
        if (pkg == NULL)
        {
            // Create the namespace if it doesn't exist
            // and add classes
            mcos::COSNamespace::addNamespace(new VisionNamespaceInfo());
        }
        else
        {
            // add classes to existing namespace
            addVSLAMClasses(pkg);
		}
        initialized = true;
    }
}

///////////////////////////////////////////////////////////////
// module termination
// - nothing to terminate
///////////////////////////////////////////////////////////////
/// @mwModuleTerminate 

extern "C" DLL_EXPORT_SYM void MLTerminate_vslam()
{

}

// Register functions

#endif // vslamlib_h
