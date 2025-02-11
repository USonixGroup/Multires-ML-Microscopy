////////////////////////////////////////////////////////////////////////////////
//  StereoVisualSLAM.hpp
//
//  This header implements the MCOS C++ API for the StereoVisualSLAM interface
//
//  Copyright 2023-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef STEREOVISUALSLAM_HPP
#define STEREOVISUALSLAM_HPP

#include <BaseVisualSLAM.hpp>
#include <StereoVisualSLAMProxy.hpp>

// Here, the full name of the class is defined as well as the
// type of policy used access the stored class instance. The COSClassInfo
// tells MATLAB how to allocate and deallocate an instance of the
// external class, and registers MATLAB-visible properties and methods.
namespace vision {
class StereoVisualSLAM : public BaseVisualSLAM<vision::vslam::StereoVisualSLAMProxy>
{
public:
    StereoVisualSLAM(void) 
        : BaseVisualSLAM<vision::vslam::StereoVisualSLAMProxy>(
            "StereoVisualSLAM",
            "MCOS StereoVisualSLAM Class"){
    }
};
} // namespace vision

namespace mcos {
namespace factory {
MCOS_FACTORY_TYPE(vision::vslam::StereoVisualSLAMProxy,
                  "vision.internal.StereoVisualSLAM",
                  StaticCastPolicy);
}
} // namespace mcos

#endif // STEREOVISUALSLAM_HPP