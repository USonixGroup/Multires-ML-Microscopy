////////////////////////////////////////////////////////////////////////////////
//  MonoVisualSLAM.hpp
//
// This header implements the MCOS C++ API for the MonoVisualSLAM interface
//
//  Copyright 2022-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef MONOVISUALSLAM_HPP
#define MONOVISUALSLAM_HPP

#include <BaseVisualSLAM.hpp>
#include <MonoVisualSLAMProxy.hpp>

// Here, the full name of the class is defined as well as the
// type of policy used access the stored class instance. The COSClassInfo
// tells MATLAB how to allocate and deallocate an instance of the
// external class, and registers MATLAB-visible properties and methods.
namespace vision {
class MonoVisualSLAM : public BaseVisualSLAM<vision::vslam::MonoVisualSLAMProxy>
{
public:
    MonoVisualSLAM(void) 
        : BaseVisualSLAM<vision::vslam::MonoVisualSLAMProxy>(
            "MonoVisualSLAM",
            "MCOS MonoVisualSLAM Class") {
    }
    
    void initializeClass(mcos::COSClass* cosClass, const mcos::COSClient* privateClient) override;
};
} // namespace vision

namespace mcos {
namespace factory {
MCOS_FACTORY_TYPE(vision::vslam::MonoVisualSLAMProxy,
                  "vision.internal.MonoVisualSLAM",
                  StaticCastPolicy);
}
} // namespace mcos

#endif // MONOVISUALSLAM_HPP