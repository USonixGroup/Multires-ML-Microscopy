////////////////////////////////////////////////////////////////////////////////
//  vslam_core_api.cpp
//  implements the APIs in vslam_core_api.hpp for monocular visual SLAM
//
//  Copyright 2023-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/libmwvslamcore_util.hpp"
    #include "vslamcore/vslam_core_api.hpp"
    #include "vslamcore/MonoVisualSLAMImpl.hpp"
    #include "vslamcore/Configuration.hpp"
    #include "../../ocv/include/cgCommon.hpp"
#else
    #include "libmwvslamcore_util.hpp"
    #include "vslam_core_api.hpp"
    #include "MonoVisualSLAMImpl.hpp"
    #include "Configuration.hpp"
    #include "cgCommon.hpp"
#endif

using namespace vision;
using namespace vslam;

void* MonoVisualSLAM_constructor(
    const double fx,
    const double fy,
    const double cx,
    const double cy,
    const BaseParams* baseParams,
    const IMUParams* imuParams,
    const char* vocabFile,
    const int threadLevel,
    const double* camToIMU){

    // None of the MonoParams in the configuration are exposed to the user
    const ConfigurationMono config(*baseParams, *imuParams, defaultMonoParams());

    const cv::Matx44d camToIMUMatx = (camToIMU) ? cv::Matx44d(camToIMU) : cv::Matx44d();

    return static_cast<void*>(new MonoVisualSLAMImpl (fx, fy, cx, cy, config, vocabFile, static_cast<VSlamConcurrency>(threadLevel), camToIMUMatx));
}

void MonoVisualSLAM_addFrame(
    void* objPtr,
    const uint8_T* imgData,
    const int nRows,
    const int nCols,
    const double* imuGData,
    const double* imuAData,
    const int imuRows) {

    // Use OpenCV smart pointer to manage image and IMU data
    cv::Mat frame, imuG, imuA;

    constexpr bool isRGB{ false }; // only grayscale image allowed

    constexpr bool isMultiChannelIMU{ false }; // only single-channel imuRows-by-3 imu data allowed
    constexpr int imuCols{ 3 };

    cArrayToMat_RowMaj<uint8_T>(imgData, nRows, nCols, isRGB, frame);
    if (imuRows > 0) {
        cArrayToMat_RowMaj<double>(imuGData, imuRows, imuCols, isMultiChannelIMU, imuG);
        cArrayToMat_RowMaj<double>(imuAData, imuRows, imuCols, isMultiChannelIMU, imuA);
    }

    return static_cast<MonoVisualSLAMImpl*>(objPtr)->addFrame(frame, imuG, imuA);
}

void MonoVisualSLAM_storeGravityRotationAndScale(void* objPtr, const double* gRot, const double poseScale) {
    cv::Matx44d gRotMatx(gRot);
    static_cast<MonoVisualSLAMImpl*>(objPtr)->storeGravityRotationAndScale(gRotMatx, poseScale);
}