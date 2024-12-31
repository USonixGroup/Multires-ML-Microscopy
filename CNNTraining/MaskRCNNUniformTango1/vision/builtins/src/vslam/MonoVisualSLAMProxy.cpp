////////////////////////////////////////////////////////////////////////////////
//  MonoVisualSLAMProxy.cpp
//
//  This is the implementation of MonoVisualSLAM proxy class providing interface
//  to the MonoVisualSLAM class from MATLAB.
//
//  Copyright 2022-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////
#include <cstdio>
#include <MonoVisualSLAMProxy.hpp>
#include <VSLAMLogger.hpp>
#include <ocvmex/libmwocvmex_util.hpp>
#include <ocvmex/ocvmex_published_c_api.hpp>

#include "i18n/MessageCatalog.hpp"
#include "resources/vision/vslam_utils.hpp"

#include "SecretHandshake.hpp"

#include <fl/ustring.hpp>
#include <i18n/filesystem/codecvt_ustring_narrow_string.hpp>


namespace vision {
    namespace vslam {

        #pragma clang diagnostic push
        #pragma clang diagnostic ignored "-Wdeprecated-declarations"
        void MonoVisualSLAMProxy::configure(int nlhs,
            mcos::COSValue* plhs,
            int nrhs,
            const mcos::COSValue* prhs) {

            constexpr int maxlhs(0), minrhs(21), maxrhs(21);

            mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);

            const mxArray* intrinsics = mcos::pointerValue<const mxArray*>(prhs[1]);

            mxArray* mxFocalLenth = mxGetPropertyShared(intrinsics, 0, "FocalLength");
            double* focalLength = matrix::getTypedData<double>(mxFocalLenth);
            mxArray* mxPrincipalPoint = mxGetPropertyShared(intrinsics, 0, "PrincipalPoint");
            double* principalPoint = matrix::getTypedData<double>(mxPrincipalPoint);

            mxArray* mxImageSize = mxGetPropertyShared(intrinsics, 0, "ImageSize");
            double* imageSize = matrix::getTypedData<double>(mxImageSize);

            ConfigurationMono config;
            config.baseParams.nRows = static_cast<int>(imageSize[0]);
            config.baseParams.nCols = static_cast<int>(imageSize[1]);

            const mxArray* hasIMU = mcos::pointerValue<const mxArray*>(prhs[2]);
            config.imuParams.hasIMU = static_cast<bool>(mxGetScalar(hasIMU));

            if (config.imuParams.hasIMU) {

                const mxArray* imuParams = mcos::pointerValue<const mxArray*>(prhs[3]);

                mxArray* mxSampleRate = mxGetPropertyShared(imuParams, 0, "SampleRate");
                config.imuParams.sampleRate = static_cast<int>(mxGetScalar(mxSampleRate));

                mxArray* mxGyroscopeBiasNoise = mxGetPropertyShared(imuParams, 0, "GyroscopeBiasNoise");
                double* gyroscopeBiasNoise = matrix::getTypedData<double>(mxGyroscopeBiasNoise);

                mxArray* mxAccelerometerBiasNoise = mxGetPropertyShared(imuParams, 0, "AccelerometerBiasNoise");
                double* accelerometerBiasNoise = matrix::getTypedData<double>(mxAccelerometerBiasNoise);

                mxArray* mxGyroscopeNoise = mxGetPropertyShared(imuParams, 0, "GyroscopeNoise");
                double* gyroscopeNoise = matrix::getTypedData<double>(mxGyroscopeNoise);

                mxArray* mxAccelerometerNoise = mxGetPropertyShared(imuParams, 0, "AccelerometerNoise");
                double* accelerometerNoise = matrix::getTypedData<double>(mxAccelerometerNoise);

                config.imuParams.gyroscopeBiasNoise = gyroscopeBiasNoise[0];
                config.imuParams.accelerometerBiasNoise = accelerometerBiasNoise[0];
                config.imuParams.gyroscopeNoise = gyroscopeNoise[0];
                config.imuParams.accelerometerNoise = accelerometerNoise[0];

                mxDestroyArray(mxSampleRate);
                mxDestroyArray(mxGyroscopeBiasNoise);
                mxDestroyArray(mxAccelerometerBiasNoise);
                mxDestroyArray(mxGyroscopeNoise);
                mxDestroyArray(mxAccelerometerNoise);

                const mxArray* mxGravityDirection = mcos::pointerValue<const mxArray*>(prhs[4]);
                config.imuParams.gravityDirection = static_cast<int>(mxGetScalar(mxGravityDirection));

            }
            
            const mxArray* mxScalarFactor = mcos::pointerValue<const mxArray*>(prhs[5]);
            config.baseParams.scaleFactor = static_cast<float>(mxGetScalar(mxScalarFactor));

            const mxArray* mxNumLevels = mcos::pointerValue<const mxArray*>(prhs[6]);
            config.baseParams.numLevels = static_cast<int>(mxGetScalar(mxNumLevels));

            const mxArray* mxMaxNumPoints = mcos::pointerValue<const mxArray*>(prhs[7]);
            config.baseParams.numFeatures = static_cast<int>(mxGetScalar(mxMaxNumPoints));

            const mxArray* maxTrackedFeatureRange = mcos::pointerValue<const mxArray*>(prhs[8]);
            double* trackedFeatureRange = matrix::getTypedData<double>(maxTrackedFeatureRange);
            config.baseParams.minNumPnPPoints = static_cast<int>(trackedFeatureRange[0]);
            config.baseParams.minNumPointWeakFrame = static_cast<int>(trackedFeatureRange[1]);

            const mxArray* mxNaxSkippedFrames = mcos::pointerValue<const mxArray*>(prhs[9]);
            config.baseParams.numSkippedFrames = static_cast<int>(mxGetScalar(mxNaxSkippedFrames));

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

            const mxArray* cam2IMUTF = mcos::pointerValue<const mxArray*>(prhs[14]);
            mxArray* mxCamToIMU = mxGetPropertyShared(cam2IMUTF, 0, "A");
            double* camToIMUData = matrix::getTypedData<double>(mxCamToIMU); // column-major
            cv::Matx44d camToIMU;
            if (camToIMUData != nullptr) {
                camToIMU = cv::Matx44d(camToIMUData).t(); // convert to row-major
            }

            const mxArray* mxMaxReprojError = mcos::pointerValue<const mxArray*>(prhs[15]);
            config.baseParams.maxReprojError = static_cast<float>(mxGetScalar(mxMaxReprojError));

            const mxArray* mxMaxCosParallax = mcos::pointerValue<const mxArray*>(prhs[16]);
            config.baseParams.maxCosParallax = static_cast<double>(mxGetScalar(mxMaxCosParallax));

            const mxArray* mxMaxCeresRE = mcos::pointerValue<const mxArray*>(prhs[17]);
            config.baseParams.maxCeresRE = static_cast<double>(mxGetScalar(mxMaxCeresRE));

            const mxArray* mxMaxNumIterationsBA = mcos::pointerValue<const mxArray*>(prhs[18]);
            config.baseParams.maxNumIterationsBA = static_cast<int>(mxGetScalar(mxMaxNumIterationsBA));

            const mxArray* mxOptimizationInterval = mcos::pointerValue<const mxArray*>(prhs[19]);
            config.baseParams.optimizationInterval = static_cast<int>(mxGetScalar(mxOptimizationInterval));

            const mxArray* mxMinNumMatches = mcos::pointerValue<const mxArray*>(prhs[20]);
            config.baseParams.minNumMatches = static_cast<int>(mxGetScalar(mxMinNumMatches));

            visualSLAMImplObj = std::make_shared<vision::vslam::MonoVisualSLAMImpl>(
                focalLength[0],
                focalLength[1],
                principalPoint[0] - 1, // Convert from 1-based indexing to 0-based indexing
                principalPoint[1] - 1,
                config,
                fileLocation.c_str(),
                desiredLevel,
                camToIMU);

            mxDestroyArray(mxFocalLenth);
            mxDestroyArray(mxPrincipalPoint);
            mxDestroyArray(mxImageSize);
            mxDestroyArray(mxCamToIMU);
        }
        #pragma clang diagnostic pop

        void MonoVisualSLAMProxy::addFrame(int nlhs,
            mcos::COSValue* plhs,
            int nrhs,
            const mcos::COSValue* prhs) {
            constexpr int maxlhs(0), minrhs(4), maxrhs(4);
            mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);

            cv::Mat frame, imuG, imuA;

            const mxArray* mxImg = mcos::pointerValue<const mxArray*>(prhs[1]);
            ocvMxArrayToImage_uint8(mxImg, frame);

            const mxArray* mxImuG = mcos::pointerValue<const mxArray*>(prhs[2]);
            ocvMxArrayToImage_double(mxImuG, imuG);

            const mxArray* mxImuA = mcos::pointerValue<const mxArray*>(prhs[3]);
            ocvMxArrayToImage_double(mxImuA, imuA);

            visualSLAMImplObj->addFrame(frame, imuG, imuA);

        }


        void MonoVisualSLAMProxy::storeGravityRotationAndScale(int nlhs,
            mcos::COSValue* plhs,
            int nrhs,
            const mcos::COSValue* prhs) {

            constexpr int maxlhs(0), minrhs(3), maxrhs(3);
            mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);

            const mxArray* mxCamToLocalIMU = mcos::pointerValue<const mxArray*>(prhs[1]);
            const double* camToLocalIMUData = matrix::getTypedData<double>(mxCamToLocalIMU); // column major
            const cv::Matx44d camToLocalIMU = cv::Matx44d(camToLocalIMUData).t(); // convert to row-major

            const mxArray* mxScale = mcos::pointerValue<const mxArray*>(prhs[2]);
            const double scale = static_cast<double>(mxGetScalar(mxScale));

            visualSLAMImplObj->storeGravityRotationAndScale(camToLocalIMU, scale);
        }

    } // namespace vslam
} // namespace vision
