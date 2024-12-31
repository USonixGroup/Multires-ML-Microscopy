////////////////////////////////////////////////////////////////////////////////
//  basevslam_core_api.cpp
//  Implements the APIs in basevslam_core_api.hpp for shared functionality of
//  monocular, stereo, and RGBD visual SLAM.
//
//  Copyright 2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/libmwvslamcore_util.hpp"
    #include "vslamcore/basevslam_core_api.hpp"
    #include "vslamcore/BaseVisualSLAMInterface.hpp"
    #include "../../ocv/include/cgCommon.hpp"
#else
    #include "libmwvslamcore_util.hpp"
    #include "basevslam_core_api.hpp"
    #include "BaseVisualSLAMInterface.hpp"
    #include "cgCommon.hpp"
#endif

using namespace vision;
using namespace vslam;

/**
* @brief Casts void* to a BaseVisualSLAMInterface*. BaseVisualSLAMInterface is the
*        root class for MonoVisualSLAMImpl, StereoVisualSLAMImpl, and RGBDVisualSLAMImpl.
*
* @param[in] objPtr, a void pointer that must point to (a subclass of) BaseVisualSLAMInterface.
* @return pointer to a BaseVisualSLAMInterface.
*/
BaseVisualSLAMInterface* getInterfacePtr(void* objPtr) {
    return static_cast<BaseVisualSLAMInterface*>(objPtr);
}

bool BaseVisualSLAM_hasNewKeyFrame(void* objPtr) {
    return getInterfacePtr(objPtr)->hasNewKeyFrame();
}

void BaseVisualSLAM_getWorldPoints(void* objPtr, double* xyzPoints) {
    std::vector<cv::Vec3d> worldPoints = getInterfacePtr(objPtr)->getWorldPoints();

    for (int i = 0; i != static_cast<int>(worldPoints.size()); ++i) {
        xyzPoints[i * 3]     = worldPoints[i][0];
        xyzPoints[i * 3 + 1] = worldPoints[i][1];
        xyzPoints[i * 3 + 2] = worldPoints[i][2];
    }
}

void BaseVisualSLAM_getCameraPoses(void* objPtr, double* tformA) {
    const std::vector<cv::Matx44d> camPoses = getInterfacePtr(objPtr)->getCameraPoses().second;

    for (int n = 0; n != static_cast<int>(camPoses.size()); ++n) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                tformA[n * 16 + i * 4 + j] = camPoses[n](i, j);
            }
        }
    }
}

void BaseVisualSLAM_getKeyFrameIndex(void* objPtr, int* keyFrameIndices) {
    const std::vector<int> kfIndicesInternal = getInterfacePtr(objPtr)->getKeyFrameIndex();

    for (int i = 0; i != static_cast<int>(kfIndicesInternal.size()); ++i) {
        keyFrameIndices[i] = kfIndicesInternal[i] + 1; // MATLAB uses 1-based indexing 
    }
}

void BaseVisualSLAM_getViewIDs(void* objPtr, int* viewIDs) {
    const std::vector<int> viewIDsInternal = getInterfacePtr(objPtr)->getViewIDs();
    for (int i = 0; i != static_cast<int>(viewIDsInternal.size()); ++i) {
        viewIDs[i] = viewIDsInternal[i];
    }
}

int BaseVisualSLAM_getNumWorldPoints(void* objPtr) {
    return static_cast<int>(getInterfacePtr(objPtr)->getWorldPoints().size());
}

int BaseVisualSLAM_getNumCameraPoses(void* objPtr) {
    return static_cast<int>(getInterfacePtr(objPtr)->getCameraPoses().second.size());
}

bool BaseVisualSLAM_isInitialized(void* objPtr) {
    return getInterfacePtr(objPtr)->getIsMapInitialized();
}

bool BaseVisualSLAM_isLoopRecentlyClosed(void* objPtr) {
    return getInterfacePtr(objPtr)->getIsLoopRecentlyClosed();
}

int BaseVisualSLAM_getNumTrackedPoints(void* objPtr) {
    return getInterfacePtr(objPtr)->getNumTrackedPoints();
}

bool BaseVisualSLAM_isDone(void* objPtr) {
    return getInterfacePtr(objPtr)->isDone();
}

void BaseVisualSLAM_reset(void* objPtr) {
    return getInterfacePtr(objPtr)->reset();
}

int BaseVisualSLAM_getNumIMUMeasurements(void* objPtr, const int viewId) {
    return getInterfacePtr(objPtr)->getNumIMUMeasurements(viewId);
}

void BaseVisualSLAM_getViewIMUMeasurements(void* objPtr, const int viewId, double* imuG, double* imuA) {
    const std::pair<std::vector<double>, std::vector<double>> imuData = getInterfacePtr(objPtr)->getViewIMUMeasurements(viewId);
    for (int i = 0; i < imuData.first.size(); ++i) {
        imuG[i] = imuData.first[i];
        imuA[i] = imuData.second[i];
    }
}

BaseParams BaseVisualSLAM_defaultBaseParams() {
    return defaultBaseParams();
}

IMUParams BaseVisualSLAM_defaultIMUParams() {
    return defaultIMUParams();
}