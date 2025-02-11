////////////////////////////////////////////////////////////////////////////////
//  stereovslam_core_api.cpp
//  implements the APIs in stereovslam_core_api.hpp for stereo visual SLAM
//
//  Copyright 2023-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/libmwvslamcore_util.hpp"
    #include "vslamcore/stereovslam_core_api.hpp"
    #include "vslamcore/StereoVisualSLAMImpl.hpp"
    #include "vslamcore/Configuration.hpp"
    #include "../../ocv/include/cgCommon.hpp"
#else
    #include "libmwvslamcore_util.hpp"
    #include "stereovslam_core_api.hpp"
    #include "StereoVisualSLAMImpl.hpp"
    #include "Configuration.hpp"
    #include "cgCommon.hpp"
#endif

using namespace vision;
using namespace vslam;

void* StereoVisualSLAM_constructor(
    const double fx,
    const double fy,
    const double cx,
    const double cy,
    const double baseline,
    const BaseParams* baseParams,
    const StereoParams* stereoParams,
    const char* vocabFile,
    const int threadLevel) {

    // IMU not supported for stereovslam
    const ConfigurationStereo config(*baseParams, defaultIMUParams(), *stereoParams);

    return static_cast<void*>(new StereoVisualSLAMImpl (fx, fy, cx, cy, baseline, config, vocabFile, static_cast<VSlamConcurrency>(threadLevel)));
}

void StereoVisualSLAM_addFrame(
    void* objPtr,
    const uint8_T* I1,
    const uint8_T* I2,
    const real32_T* disparity,
    const int nRows,
    const int nCols) {

    cv::Mat I1_mat, I2_mat, disparity_mat;

    constexpr bool isRGB{ false }; // only grayscale image allowed

    cArrayToMat_RowMaj<uint8_T>(I1, nRows, nCols, isRGB, I1_mat);
    cArrayToMat_RowMaj<uint8_T>(I2, nRows, nCols, isRGB, I2_mat);
    if (disparity != nullptr) {
        cArrayToMat_RowMaj<real32_T>(disparity, nRows, nCols, isRGB, disparity_mat);
    }

    return static_cast<StereoVisualSLAMImpl*>(objPtr)->addFrame(I1_mat, I2_mat, disparity_mat);
}

StereoParams StereoVisualSLAM_defaultStereoParams() {
    return defaultStereoParams();
}