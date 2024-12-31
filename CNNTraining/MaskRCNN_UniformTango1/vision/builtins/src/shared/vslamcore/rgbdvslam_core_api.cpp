////////////////////////////////////////////////////////////////////////////////
//  rgbdvslam_core_api.cpp
//  implements the APIs in rgbdvslam_core_api.hpp for RGB-D visual SLAM
//
//  Copyright 2023-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/libmwvslamcore_util.hpp"
    #include "vslamcore/rgbdvslam_core_api.hpp"
    #include "vslamcore/RGBDVisualSLAMImpl.hpp"
    #include "vslamcore/Configuration.hpp"
    #include "../../ocv/include/cgCommon.hpp"
#else
    #include "libmwvslamcore_util.hpp"
    #include "rgbdvslam_core_api.hpp"
    #include "RGBDVisualSLAMImpl.hpp"
    #include "Configuration.hpp"
    #include "cgCommon.hpp"
#endif

using namespace vision;
using namespace vslam;

void* RGBDVisualSLAM_constructor(
    const double fx,
    const double fy,
    const double cx,
    const double cy,
    const BaseParams* baseParams,
    const RGBDParams* rgbdParams,
    const char* vocabFile,
    const int threadLevel) {

    // IMU not supported for rgbdvslam
    const ConfigurationRGBD config(*baseParams, defaultIMUParams(), *rgbdParams);

    return static_cast<void*>(new RGBDVisualSLAMImpl (fx, fy, cx, cy, config, vocabFile, static_cast<VSlamConcurrency>(threadLevel)));
}

void RGBDVisualSLAM_addFrame(
    void* objPtr,
    const uint8_T* imgColorData,
    const real32_T* imgDepthData,
    const int nRows,
    const int nCols) {

    cv::Mat frameColor, frameDepth;

    constexpr bool isRGB{ false }; // only grayscale image allowed

    cArrayToMat_RowMaj<uint8_T>(imgColorData, nRows, nCols, isRGB, frameColor);
    cArrayToMat_RowMaj<real32_T>(imgDepthData, nRows, nCols, isRGB, frameDepth);

    return static_cast<RGBDVisualSLAMImpl*>(objPtr)->addFrame(frameColor, frameDepth);
}