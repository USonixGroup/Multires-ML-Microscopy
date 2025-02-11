////////////////////////////////////////////////////////////////////////////////
//  RGBDVisualSLAM.hpp
//
//  This header implements the MCOS C++ API for the RGBDVisualSLAM interface
//
//  Copyright 2023-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef RGBDVISUALSLAM_HPP
#define RGBDVISUALSLAM_HPP

#include <BaseVisualSLAM.hpp>
#include <RGBDVisualSLAMProxy.hpp>

// Here, the full name of the class is defined as well as the
// type of policy used access the stored class instance. The COSClassInfo
// tells MATLAB how to allocate and deallocate an instance of the
// external class, and registers MATLAB-visible properties and methods.
namespace vision {
class RGBDVisualSLAM : public BaseVisualSLAM<vision::vslam::RGBDVisualSLAMProxy>
{
public:
    RGBDVisualSLAM(void)
        : BaseVisualSLAM<vision::vslam::RGBDVisualSLAMProxy>(
            "RGBDVisualSLAM",
            "MCOS RGBDVisualSLAM Class") {
    }
};
} // namespace vision

namespace mcos {
namespace factory {
MCOS_FACTORY_TYPE(vision::vslam::RGBDVisualSLAMProxy,
                  "vision.internal.RGBDVisualSLAM",
                  StaticCastPolicy);
}
} // namespace mcos

#endif // RGBDVISUALSLAM_HPP
