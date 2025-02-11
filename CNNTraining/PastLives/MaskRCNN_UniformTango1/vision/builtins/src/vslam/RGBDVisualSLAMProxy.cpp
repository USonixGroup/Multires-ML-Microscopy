////////////////////////////////////////////////////////////////////////////////
//  RGBDVisualSLAMProxy.cpp
//
//  This is the implementation of RGBDVisualSLAM proxy class providing interface
//  to the RGBDVisualSLAM class from MATLAB.
//
//  Copyright 2023-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////
#include <cstdio>
#include <RGBDVisualSLAMProxy.hpp>
#include <VSLAMLogger.hpp>
#include <ocvmex/libmwocvmex_util.hpp>
#include <ocvmex/ocvmex_published_c_api.hpp>

#include "i18n/MessageCatalog.hpp"
#include "resources/vision/vslam_utils.hpp"

#include "SecretHandshake.hpp"

namespace vision {
    namespace vslam {

        #pragma clang diagnostic push
        #pragma clang diagnostic ignored "-Wdeprecated-declarations"
        void RGBDVisualSLAMProxy::configure(int nlhs,
            mcos::COSValue* plhs,
            int nrhs,
            const mcos::COSValue* prhs) {

            constexpr int maxlhs(0), minrhs(19), maxrhs(19);
            mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);

            const mxArray* intrinsics = mcos::pointerValue<const mxArray*>(prhs[1]);

            mxArray* mxFocalLenth = mxGetPropertyShared(intrinsics, 0, "FocalLength");
            double* focalLength = matrix::getTypedData<double>(mxFocalLenth);
            mxArray* mxPrincipalPoint = mxGetPropertyShared(intrinsics, 0, "PrincipalPoint");
            double* principalPoint = matrix::getTypedData<double>(mxPrincipalPoint);
            mxArray* mxImageSize = mxGetPropertyShared(intrinsics, 0, "ImageSize");
            double* imageSize = matrix::getTypedData<double>(mxImageSize);

            ConfigurationRGBD config;
            config.baseParams.nRows = static_cast<int>(imageSize[0]);
            config.baseParams.nCols = static_cast<int>(imageSize[1]);

            const mxArray* mxScalarFactor = mcos::pointerValue<const mxArray*>(prhs[2]);
            config.baseParams.scaleFactor = static_cast<float>(mxGetScalar(mxScalarFactor));

            const mxArray* mxNumLevels = mcos::pointerValue<const mxArray*>(prhs[3]);
            config.baseParams.numLevels = static_cast<int>(mxGetScalar(mxNumLevels));

            const mxArray* mxMaxNumPoints = mcos::pointerValue<const mxArray*>(prhs[4]);
            config.baseParams.numFeatures = static_cast<int>(mxGetScalar(mxMaxNumPoints));

            const mxArray* maxTrackedFeatureRange = mcos::pointerValue<const mxArray*>(prhs[5]);
            double* trackedFeatureRange = matrix::getTypedData<double>(maxTrackedFeatureRange);
            config.baseParams.minNumPnPPoints = static_cast<int>(trackedFeatureRange[0]);
            config.baseParams.minNumPointWeakFrame = static_cast<int>(trackedFeatureRange[1]);

            const mxArray* mxNaxSkippedFrames = mcos::pointerValue<const mxArray*>(prhs[6]);
            config.baseParams.numSkippedFrames = static_cast<int>(mxGetScalar(mxNaxSkippedFrames));

            const mxArray* mxDepthScaleFactor = mcos::pointerValue<const mxArray*>(prhs[7]);
            config.rgbdParams.depthScaleFactor = static_cast<float>(mxGetScalar(mxDepthScaleFactor));

            const mxArray* mxDepthRange = mcos::pointerValue<const mxArray*>(prhs[8]);
            double* depthRange = matrix::getTypedData<double>(mxDepthRange);
            config.rgbdParams.depthRange[0] = static_cast<float>(depthRange[0]);
            config.rgbdParams.depthRange[1] = static_cast<float>(depthRange[1]);

            const mxArray* mxLoopClosureThreshold = mcos::pointerValue<const mxArray*>(prhs[9]);
            config.baseParams.minNumMatchesLoop = static_cast<int>(mxGetScalar(mxLoopClosureThreshold));

            const mxArray* mxVerbose = mcos::pointerValue<const mxArray*>(prhs[10]);
            config.baseParams.verbose = static_cast<int>(mxGetScalar(mxVerbose));
            if(config.baseParams.verbose == 1) {
                config.loggerPtr = std::make_shared<VSLAMLogger>(config.baseParams.verbose);
            } else if(config.baseParams.verbose == 2 || config.baseParams.verbose == 3) {
                std::string logFileName = std::tmpnam(nullptr);
                config.loggerPtr = std::make_shared<VSLAMLogger>(config.baseParams.verbose, logFileName);
            }

            const mxArray* mxVocabFile = mcos::pointerValue<const mxArray*>(prhs[11]);
            std::string fileLocation = mxArrayToString(mxVocabFile);

            const mxArray* mxThreadLevel = mcos::pointerValue<const mxArray*>(prhs[12]);
            const VSlamConcurrency desiredLevel = static_cast<VSlamConcurrency>(mxGetScalar(mxThreadLevel));

            const mxArray* mxMaxReprojError = mcos::pointerValue<const mxArray*>(prhs[13]);
            config.baseParams.maxReprojError = static_cast<float>(mxGetScalar(mxMaxReprojError));

            const mxArray* mxMaxCosParallax = mcos::pointerValue<const mxArray*>(prhs[14]);
            config.baseParams.maxCosParallax = static_cast<double>(mxGetScalar(mxMaxCosParallax));

            const mxArray* mxMaxCeresRE = mcos::pointerValue<const mxArray*>(prhs[15]);
            config.baseParams.maxCeresRE = static_cast<double>(mxGetScalar(mxMaxCeresRE));

            const mxArray* mxMaxNumIterationsBA = mcos::pointerValue<const mxArray*>(prhs[16]);
            config.baseParams.maxNumIterationsBA = static_cast<int>(mxGetScalar(mxMaxNumIterationsBA));

            const mxArray* mxOptimizationInterval = mcos::pointerValue<const mxArray*>(prhs[17]);
            config.baseParams.optimizationInterval = static_cast<int>(mxGetScalar(mxOptimizationInterval));

            const mxArray* mxMinNumMatches = mcos::pointerValue<const mxArray*>(prhs[18]);
            config.baseParams.minNumMatches = static_cast<int>(mxGetScalar(mxMinNumMatches));

            visualSLAMImplObj = std::make_shared<vision::vslam::RGBDVisualSLAMImpl>(
                focalLength[0],
                focalLength[1],
                principalPoint[0] - 1, // Convert from 1-based indexing to 0-based indexing
                principalPoint[1] - 1,
                config,
                fileLocation.c_str(),
                desiredLevel);

            mxDestroyArray(mxFocalLenth);
            mxDestroyArray(mxPrincipalPoint);
            mxDestroyArray(mxImageSize);
        }
        #pragma clang diagnostic pop

        void RGBDVisualSLAMProxy::addFrame(int nlhs,
            mcos::COSValue* plhs,
            int nrhs,
            const mcos::COSValue* prhs) {
            constexpr int maxlhs(0), minrhs(3), maxrhs(3);
            mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);

            const mxArray* colorI = mcos::pointerValue<const mxArray*>(prhs[1]);
            const mxArray* depthI = mcos::pointerValue<const mxArray*>(prhs[2]);
            cv::Mat colorImage, depthImage;
            ocvMxArrayToImage_uint8(colorI, colorImage);
            ocvMxArrayToImage_single(depthI, depthImage);

            visualSLAMImplObj->addFrame(colorImage, depthImage);
        }

    } // namespace vslam
} // namespace vision