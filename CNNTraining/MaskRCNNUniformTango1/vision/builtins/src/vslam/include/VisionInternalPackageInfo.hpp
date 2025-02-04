// Copyright 2022-2024 The MathWorks, Inc.
#ifndef vision_internal_package
#define vision_internal_package

#include "mcos.h"
#include "MonoVisualSLAM.hpp"
#include "RGBDVisualSLAM.hpp"
#include "StereoVisualSLAM.hpp"
#include "version.h"
#include "mex.h"

//////////////////////////////////////////////////////////////////////////////
// This header defines the classes needed to create the vision.internal 
// namespace and adds OpenCV classes exposed via MCOS to that namespace.  Namespace
// initialization occurs when the ocv module is loaded (see ocvlib.cpp for 
// more details).
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Adds classes to the vision.internal namespace
//  - list any additional classes to add to vision.internal by adding 
//    this line, ns->addClass(...), for every class you want to add
//////////////////////////////////////////////////////////////////////////////
void addVSLAMClasses(mcos::COSNamespace *ns)
{
    // add OpenCV classes to the vision.internal namespace
    ns->addClass(new vision::MonoVisualSLAM());
    ns->addClass(new vision::RGBDVisualSLAM());
    ns->addClass(new vision::StereoVisualSLAM());
}

//////////////////////////////////////////////////////////////////////////////
// Namespace Info for internal namespace
//////////////////////////////////////////////////////////////////////////////
class VisionInternalNamespaceInfo : public mcos::COSNamespaceInfo
{
  public:
    // define the namespace name
    virtual const char *getName(void) {
        return "internal";
    }
    // called during namespace initialization
    virtual void initializeNamespace(mcos::COSNamespace *ns)
    {
        // add classes to internal namespace
        addVSLAMClasses(ns);
    }

};

//////////////////////////////////////////////////////////////////////////////
// Namespace Info for the vision namespace
//////////////////////////////////////////////////////////////////////////////
class VisionNamespaceInfo : public mcos::COSNamespaceInfo
{
 public:
    // define the namespace name
    virtual const char *getName(void) {
        return "vision";
    }
    // called during namespace initialization
    virtual void initializeNamespace(mcos::COSNamespace *ns)
    {
        // add internal as a sub-namespace under vision
        ns->addInnerNamespace(new VisionInternalNamespaceInfo());
    }
};

#endif
