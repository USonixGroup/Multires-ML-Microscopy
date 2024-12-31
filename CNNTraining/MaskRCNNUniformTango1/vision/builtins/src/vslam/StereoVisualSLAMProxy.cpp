////////////////////////////////////////////////////////////////////////////////
//  StereoVisualSLAMProxy.cpp
//
//  This is the implementation of StereoVisualSLAM proxy class providing interface
//  to the StereoVisualSLAM class from MATLAB.
//
//  Copyright 2023-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////
#include <cstdio>
#include <StereoVisualSLAMProxy.hpp>
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
        void StereoVisualSLAMProxy::configure(int nlhs,
            mcos::COSValue* plhs,
            int nrhs,
            const mcos::COSValue* prhs) {

            constexpr int maxlhs(0), minrhs(20), maxrhs(20);
            mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);

            const mxArray* intrinsics = mcos::pointerValue<const mxArray*>(prhs[1]);

            mxArray* mxFocalLenth = mxGetPropertyShared(intrinsics, 0, "FocalLength");
            double* focalLength = matrix::getTypedData<double>(mxFocalLenth);
            mxArray* mxPrincipalPoint = mxGetPropertyShared(intrinsics, 0, "PrincipalPoint");
            double* principalPoint = matrix::getTypedData<double>(mxPrincipalPoint);
            mxArray* mxImageSize = mxGetPropertyShared(intrinsics, 0, "ImageSize");
            double* imageSize = matrix::getTypedData<double>(mxImageSize);

            ConfigurationStereo config;
            config.baseParams.nRows = static_cast<int>(imageSize[0]);
            config.baseParams.nCols = static_cast<int>(imageSize[1]);

            const mxArray* mxBaseline = mcos::pointerValue<const mxArray*>(prhs[2]);
            const double baseline = static_cast<double>(mxGetScalar(mxBaseline));

            const mxArray* mxScaleFactor = mcos::pointerValue<const mxArray*>(prhs[3]);
            config.baseParams.scaleFactor = static_cast<float>(mxGetScalar(mxScaleFactor));

            const mxArray* mxNumLevels = mcos::pointerValue<const mxArray*>(prhs[4]);
            config.baseParams.numLevels = static_cast<int>(mxGetScalar(mxNumLevels));

            const mxArray* mxMaxNumPoints = mcos::pointerValue<const mxArray*>(prhs[5]);
            config.baseParams.numFeatures = static_cast<int>(mxGetScalar(mxMaxNumPoints));

            const mxArray* maxTrackedFeatureRange = mcos::pointerValue<const mxArray*>(prhs[6]);
            double* trackedFeatureRange = matrix::getTypedData<double>(maxTrackedFeatureRange);
            config.baseParams.minNumPnPPoints = static_cast<int>(trackedFeatureRange[0]);
            config.baseParams.minNumPointWeakFrame = static_cast<int>(trackedFeatureRange[1]);

            const mxArray* mxNaxSkippedFrames = mcos::pointerValue<const mxArray*>(prhs[7]);
            config.baseParams.numSkippedFrames = static_cast<int>(mxGetScalar(mxNaxSkippedFrames));

            const mxArray* mxDisparityRange = mcos::pointerValue<const mxArray*>(prhs[8]);
            double* disparityRange = matrix::getTypedData<double>(mxDisparityRange);
            config.stereoParams.minDisparity = static_cast<int>(disparityRange[0]);
            config.stereoParams.maxDisparity = static_cast<int>(disparityRange[1]);

            const mxArray* mxUniquenessThreshold = mcos::pointerValue<const mxArray*>(prhs[9]);
            config.stereoParams.uniquenessThreshold = static_cast<int>(mxGetScalar(mxUniquenessThreshold));

            const mxArray* mxLoopClosureThreshold = mcos::pointerValue<const mxArray*>(prhs[10]);
            config.baseParams.minNumMatchesLoop = static_cast<int>(mxGetScalar(mxLoopClosureThreshold));

            const mxArray* mxVerbose = mcos::pointerValue<const mxArray*>(prhs[11]);
            config.baseParams.verbose = static_cast<int>(mxGetScalar(mxVerbose));
            if(config.baseParams.verbose == 1) {
                config.loggerPtr = std::make_shared<VSLAMLogger>(config.baseParams.verbose);
            } else if(config.baseParams.verbose == 2 || config.baseParams.verbose == 3) {
                std::string logFileName = std::tmpnam(nullptr);
                config.loggerPtr = std::make_shared<VSLAMLogger>(config.baseParams.verbose, logFileName);
            }

            const mxArray* mxVocabFile = mcos::pointerValue<const mxArray*>(prhs[12]);
            std::string fileLocation = mxArrayToString(mxVocabFile);

            const mxArray* mxThreadLevel = mcos::pointerValue<const mxArray*>(prhs[13]);
            const VSlamConcurrency desiredLevel = static_cast<VSlamConcurrency>(mxGetScalar(mxThreadLevel));

            const mxArray* mxMaxReprojError = mcos::pointerValue<const mxArray*>(prhs[14]);
            config.baseParams.maxReprojError = static_cast<float>(mxGetScalar(mxMaxReprojError));

            const mxArray* mxMaxCosParallax = mcos::pointerValue<const mxArray*>(prhs[15]);
            config.baseParams.maxCosParallax = static_cast<double>(mxGetScalar(mxMaxCosParallax));

            const mxArray* mxMaxCeresRE = mcos::pointerValue<const mxArray*>(prhs[16]);
            config.baseParams.maxCeresRE = static_cast<double>(mxGetScalar(mxMaxCeresRE));

            const mxArray* mxMaxNumIterationsBA = mcos::pointerValue<const mxArray*>(prhs[17]);
            config.baseParams.maxNumIterationsBA = static_cast<int>(mxGetScalar(mxMaxNumIterationsBA));

            const mxArray* mxOptimizationInterval = mcos::pointerValue<const mxArray*>(prhs[18]);
            config.baseParams.optimizationInterval = static_cast<int>(mxGetScalar(mxOptimizationInterval));

            const mxArray* mxMinNumMatches = mcos::pointerValue<const mxArray*>(prhs[19]);
            config.baseParams.minNumMatches = static_cast<int>(mxGetScalar(mxMinNumMatches));

            visualSLAMImplObj = std::make_shared<vision::vslam::StereoVisualSLAMImpl>(
                focalLength[0],
                focalLength[1],
                principalPoint[0] - 1, // Convert from 1-based indexing to 0-based indexing
                principalPoint[1] - 1,
                baseline,
                config,
                fileLocation.c_str(),
                desiredLevel);

            mxDestroyArray(mxFocalLenth);
            mxDestroyArray(mxPrincipalPoint);
            mxDestroyArray(mxImageSize);
        }
        #pragma clang diagnostic pop

        void StereoVisualSLAMProxy::addFrame(int nlhs,
            mcos::COSValue* plhs,
            int nrhs,
            const mcos::COSValue* prhs) {
            constexpr int maxlhs(0), minrhs(4), maxrhs(4);
            mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);

            cv::Mat I1, I2, disparity;

            const mxArray* mxI1 = mcos::pointerValue<const mxArray*>(prhs[1]);
            const mxArray* mxI2 = mcos::pointerValue<const mxArray*>(prhs[2]);
            ocvMxArrayToImage_uint8(mxI1, I1);
            ocvMxArrayToImage_uint8(mxI2, I2);

            // If prhs[3] points to an empty matrix in MATLAB, disparity will remain an empty cv::Mat
            const mxArray* mxDisparity = mcos::pointerValue<const mxArray*>(prhs[3]);
            ocvMxArrayToImage_single(mxDisparity, disparity);
            
            visualSLAMImplObj->addFrame(I1, I2, disparity);
        }

    } // namespace vslam
} // namespace vision