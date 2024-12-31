////////////////////////////////////////////////////////////////////////////////
// Perform bundle adjustment over camera poses and 3-D world points
// 
// Copyright 2022-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#include <sstream>
#include <memory>

#ifdef BUILDING_LIBMWVSLAMCORE
#include "vslamcore/View.hpp"
#include "vslamcore/Connection.hpp"
#include "vslamcore/converter.hpp"
#include "vslamcore/WorldPoint.hpp"
#include "vslamcore/MapPointSet.hpp"
#include "vslamcore/KeyFrameSet.hpp"
#include "vslamcore/bundleAdjustment.hpp"
#include "cerescodegen/factor_graph.hpp"
#include "cerescodegen/imu_factor.hpp"
#include "cerescodegen/common_factors_2.hpp"
#include "cerescodegen/camera_projection_factor.hpp"
#else
#include "View.hpp"
#include "Connection.hpp"
#include "converter.hpp"
#include "WorldPoint.hpp"
#include "MapPointSet.hpp"
#include "KeyFrameSet.hpp"
#include "bundleAdjustment.hpp"
#include "factor_graph.hpp"
#include "imu_factor.hpp"
#include "common_factors_2.hpp"
#include "camera_projection_factor.hpp"
#endif


namespace vision {
    namespace vslam {

        void bundleAdjustment(
            MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            const cv::Matx33d& intrinsics,
            const std::vector<int>& sortedViewIds,
            const std::vector<int>& fixedViewIds,
            const Configuration& config,
            bool* isReadyToAbortBA,
            const IMUInfo& imuInfo) {

            // New empty factor graph.
            auto fg = mw_ceres::FactorGraph();

            // For verbose use
            std::stringstream strBuffer;
            // The following object is required to capture all cout statements from ceres library.
            // This object gets destroyed when it goes out of scope.
            std::shared_ptr<CoutRedirectForCeres> tempCoutObj;

            // Measurement information matrix
            cv::Vec4d measInfo{ std::pow(intrinsics(1, 1) / 1.5, 2), 0.0, 0.0, std::pow(intrinsics(1, 1) / 1.5, 2) };

            constexpr size_t poseDim{ 7 };
            constexpr size_t pointDim{ 3 };
            constexpr size_t biasDim{ 6 };
            constexpr size_t velocityDim{ 3 };

            std::set<int> allMapPointIds;

            // Assign map points lower group ID to eliminate map points first to compute the Schur complement
            constexpr int pointGroupId{ 0 };
            constexpr int viewGroupId{ 1 };
            constexpr int velGroupId{ 1 };
            constexpr int biasGroupId{ 1 };

            // Number of KF after a successful scale estimation where we only fix the first pose 
            int limitKeyFramesPostAlignment{ 10 };

            // Used to store the previous view ID for viewId.
            int prevViewId = sortedViewIds.front();

            // solver options
            mw_ceres::CeresSolverOptions opt;

            // Fill factor graph
            for (const auto& viewId : sortedViewIds) {

                // Grab location of the points seen from the current view
                std::unordered_map<int, int> xyzPointsInView = kfSet.findWorldPointsInView(viewId);

                // LinearSolverOrdering will add a node to the graph even if the view doesn't observe any map point.
                // this check makes sure that a node in factor graph always has an associated factor.
                if (!xyzPointsInView.size())
                    continue;

                // Assign group ID to views
                opt.LinearSolverOrdering.insert({ viewId, viewGroupId });

                // Grab pose value of the current view and convert it to the format [x, y, z, qx, qy, qz, qw]
                auto currView = kfSet.findView(viewId);
                cv::Matx44d pose = currView->getPose();

                std::vector<double> viewPose;
                viewPose.reserve(poseDim);
                tform2eigenquatpose(pose, viewPose);

                for (const auto& pointPair : xyzPointsInView) {

                    int pointId = pointPair.first;
                    int pixelId = pointPair.second;

                    // Store map point ID
                    allMapPointIds.insert(pointId);

                    // Assign group ID to points
                    opt.LinearSolverOrdering.insert({ pointId, pointGroupId });

                    // Get 3-D xyz point location
                    cv::Vec3d xyzLocation = mpSet.getWorldPoint(pointId)->getLocation();

                    // Get 2-D image point location
                    cv::KeyPoint keyPt = currView->getFeaturePoints(pixelId);

                    // Set up new camera factor
                    std::vector<int> ids{ viewId, pointId };
                    auto camFactor = std::make_unique<mw_ceres::FactorCameraSE3AndPointXYZ>(ids);

                    // Set up the measurements vector aka pixel positions
                    cv::Vec4d measurement{ keyPt.pt.x - intrinsics(0, 2), keyPt.pt.y - intrinsics(1, 2), intrinsics(0, 0), intrinsics(1, 1) };

                    camFactor->setMeasurement(measurement.val);
                    camFactor->setInformation(measInfo.val);

                    // Add factor to factor graph
                    const int factorID = fg.addFactor(std::move(camFactor));

                    // Store factor type
                    fg.storeFactorID(factorID, mw_ceres::FactorTypeInt["factorCameraSE3AndPointXYZ"]);

                    // Set the initial guess of the xyz points
                    fg.setVariableState({ pointId }, { xyzLocation.val[0],xyzLocation.val[1],xyzLocation.val[2] }, pointDim);

                    // Set the initial guess of the camera pose (after adding the factor)
                    if (!imuInfo.isIMUAligned) {
                        fg.setVariableState({ viewId }, viewPose, poseDim);
                    }

                }

                // Extract the velocity and bias of the view
                std::pair<std::vector<double>, std::vector<double>> viewVelocityAndBias = currView->getVelocityAndBias();

                // IMU fusion
                if (imuInfo.isIMUAligned) {

                    // First frame has no previous frame
                    if (viewId == sortedViewIds.front()) {

                        // pose prior   
                        auto posePriorFctr = make_unique< mw_ceres::FactorPoseSE3Prior>(vector<int>{ viewId });
                        posePriorFctr->setMeasurement(viewPose.data());

                        vector<double> priorPoseInfo(36, 0.0);
                        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> infoMat1(priorPoseInfo.data());
                        infoMat1.diagonal() << 4e4, 4e4, 4e4, 1e4, 1e4, 1e4; // for sigma [ 0.005, 0.005, 0.005, 0.01, 0.01, 0.01]
                        //              m    m    m    rad   rad   rad
                        posePriorFctr->setInformation(priorPoseInfo.data());

                        opt.LinearSolverOrdering.insert({ viewId, viewGroupId });

                        fg.addFactor(std::move(posePriorFctr));

                        // velocity prior
                        auto velPriorFctr = make_unique<mw_ceres::FactorVel3Prior>(vector<int>{ viewId + 2 });
                        velPriorFctr->setMeasurement(viewVelocityAndBias.first.data());

                        vector<double> priorVelInfo{ 100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 100.0 };
                        velPriorFctr->setInformation(priorVelInfo.data());

                        opt.LinearSolverOrdering.insert({ viewId + 2, velGroupId });

                        fg.addFactor(std::move(velPriorFctr));

                        // imu bias prior
                        auto biasPriorFctr = make_unique<mw_ceres::FactorIMUBiasPrior>(vector<int>{ viewId + 3 });
                        biasPriorFctr->setMeasurement(viewVelocityAndBias.second.data());

                        vector<double> priorBiasInfo(36, 0.0);
                        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> infoMat2(priorBiasInfo.data());
                        infoMat2.diagonal() << 1e6, 1e6, 1e6, 1e6, 1e6, 1e6;
                        biasPriorFctr->setInformation(priorBiasInfo.data());

                        opt.LinearSolverOrdering.insert({ viewId + 3, biasGroupId });

                        fg.addFactor(std::move(biasPriorFctr));

                    }
                    else {

                        std::vector<double> g{ 0.0, 0.0, config.imuParams.gravityDirection * 9.81 };

                        std::vector<double> covGyroNoise{config.imuParams.gyroscopeNoise, 0.0, 0.0, 0.0, config.imuParams.gyroscopeNoise, 0.0, 0.0, 0.0, config.imuParams.gyroscopeNoise};
                        std::vector<double> covAccNoise{config.imuParams.accelerometerNoise, 0.0, 0.0, 0.0, config.imuParams.accelerometerNoise, 0.0, 0.0, 0.0, config.imuParams.accelerometerNoise};
                        std::vector<double> covGyroBiasNoise{config.imuParams.gyroscopeBiasNoise, 0.0, 0.0, 0.0, config.imuParams.gyroscopeBiasNoise, 0.0, 0.0, 0.0, config.imuParams.gyroscopeBiasNoise};
                        std::vector<double> covAccBiasNoise{config.imuParams.accelerometerBiasNoise, 0.0, 0.0, 0.0, config.imuParams.accelerometerBiasNoise, 0.0, 0.0, 0.0, config.imuParams.accelerometerBiasNoise};

                        // Extract the IMU measurement of the view
                        std::pair<std::vector<double>, std::vector<double>> imuViewMeasurements = currView->getIMU();

                        // Set up the IDs for the IMU factor
                        opt.LinearSolverOrdering.insert({ viewId + 2, velGroupId });
                        opt.LinearSolverOrdering.insert({ viewId + 3, biasGroupId });

                        std::vector<int> ids{ prevViewId, prevViewId + 2, prevViewId + 3, viewId, viewId + 2, viewId + 3 };
                        //{ POSE(i-1), VELOCITY(i-1), BIAS(i-1), POSE(i), VELOCITY(i), BIAS(i) }

                        //TODO: Remove const_cast when FactorIMU const issue is fixed by Nav
                        IMUInfo& imuInfoNonConst = const_cast<IMUInfo&>(imuInfo);

                        // Set up the IMU factor
                        auto imuFactor = std::make_unique<mw_ceres::FactorIMU>(ids.data(), config.imuParams.sampleRate, g.data(),
                            covGyroBiasNoise.data(), covAccBiasNoise.data(), covGyroNoise.data(), covAccNoise.data(),
                            imuViewMeasurements.first.data(), imuViewMeasurements.second.data(), imuViewMeasurements.first.size(), imuInfoNonConst.camToIMUTransform.val);

                        // Add the IMU factor to the factor graph
                        fg.addFactor(std::move(imuFactor));

                    }

                    // Set up initial guess for the pose, velocity and bias nodes
                    fg.setVariableState({ viewId }, viewPose, poseDim);
                    fg.setVariableState({ viewId + 2 }, viewVelocityAndBias.first, velocityDim);
                    fg.setVariableState({ viewId + 3 }, viewVelocityAndBias.second, biasDim);

                }

                if (std::find(fixedViewIds.begin(), fixedViewIds.end(), viewId) != fixedViewIds.end()) {
                    fg.fixVariable({ viewId });
                    if (imuInfo.isIMUAligned) {
                        fg.fixVariable({ viewId + 2 });
                        fg.fixVariable({ viewId + 3 });
                    }
                }

                // Store for the next KF
                prevViewId = viewId;

            }

            // Use sparse schur solver to take advantage of the sparse structure
            opt.LinearSolverType = mw_ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;

            // Use low initial trust region (large damping factor) to keep the solutions close to initial guess
            opt.TrustRegionStrategyType = mw_ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
            opt.InitialTrustRegionRadius = config.imuParams.hasIMU ? 0.01 : 0.1;

            // Enable verbose at Verbose=3
            if (config.baseParams.verbose == 3) {
                opt.VerbosityLevel = 2;
                // Replace cout buffer with string buffer, so that all cout statements go to string buffer
                tempCoutObj = std::make_shared<CoutRedirectForCeres>(strBuffer.rdbuf());
            }
            else {
                opt.VerbosityLevel = 0;
            }

            // Set up the max iteration number
            opt.MaxNumIterations = config.imuParams.hasIMU ? config.imuParams.maxNumIterationsIMUBA : config.baseParams.maxNumIterationsBA;

            // Optimize factor graph
            opt.UpdateStateEveryIteration = true;
            opt.AbortOptimization = isReadyToAbortBA;

            // Store problem for final residual evaluation
            constexpr bool saveProblem{ true };
            mw_ceres::CeresSolutionInfo solnInfo = fg.optimize(opt, { -1 }, saveProblem); //{ -1 } => optimize all non-fixed nodes

            kfSet.lock();

            // Before performing BA update, check if a loop was detected 
            // and abort was called. If yes, then abort BA.
            if (isReadyToAbortBA != nullptr && *isReadyToAbortBA) {
                *isReadyToAbortBA = false;
                kfSet.unlock();
                return;
            }

            // Update view poses
            for (const auto& viewId : sortedViewIds) {

                // Grab updated pose
                cv::Matx44d pose;
                std::vector<int> idv{ viewId };
                eigenquatpose2tform(fg.getVariableState(idv), pose);

                auto currView = kfSet.findView(viewId);

                // Update view velocity and bias if the IMU has been aligned
                if (imuInfo.isIMUAligned) {
                    kfSet.updateVelocityAndBias(viewId, fg.getVariableState({ viewId + 2 }), fg.getVariableState({ viewId + 3 }));
                }

                cv::Matx33d rot;
                cv::Vec3d tran;
                pose2rt(pose, rot, tran);

                if (std::find(fixedViewIds.begin(), fixedViewIds.end(), viewId) == fixedViewIds.end()) {
                    kfSet.updateViewPose(viewId, rot, tran);
                }
            }

            // Update map point locations
            const std::unordered_map<int, std::unordered_map<int, std::vector<double>>> allMapPointResiduals =
                fg.getIndividualFactorResidualAssumingStateUnchanged(allMapPointIds, mw_ceres::FactorTypeInt["factorCameraSE3AndPointXYZ"]);

            const double reprojThreshold = config.imuParams.hasIMU ? config.imuParams.maxCeresREIMU :
                config.isMono ? config.baseParams.maxCeresRE * 2 : config.baseParams.maxCeresRE;

            for (const auto& pointIdAndErrors : allMapPointResiduals) {

                // Get 3-D xyz point location after optimization
                const int pointId = pointIdAndErrors.first;
                std::vector<int> idp{ pointId };
                const auto xyzInWorld = fg.getVariableState(idp);

                double sumOfSquares{ 0 };
                for (const auto& factorIdAndErrors : pointIdAndErrors.second) {
                    auto residualVec = factorIdAndErrors.second;
                    sumOfSquares += residualVec[0] * residualVec[0] / measInfo[0] + residualVec[1] * residualVec[1] / measInfo[3];
                }

                // Perform outlier rejection
                if (static_cast<double>(sumOfSquares / pointIdAndErrors.second.size()) > reprojThreshold) {

                    // Reject map point with large reprojection error
                    mpSet.setInvalid(pointId, kfSet);
                }
                else {

                    // Update 3-D world point location
                    mpSet.updateWorldPoint(pointId, cv::Vec3d(xyzInWorld[0], xyzInWorld[1], xyzInWorld[2]));

                    // Update additional attributes
                    mpSet.updateLimitsAndDirection(pointId, kfSet);
                }
            }

            if (config.baseParams.verbose == 3) {
                // Dump string buffer to logging
                config.loggerPtr->logMessageV3("vision::vslamVerboseCpp::BAStart");
                config.loggerPtr->logMessageV3("", { strBuffer.str() });
                config.loggerPtr->logMessageV3("vision::vslamVerboseCpp::BAEnd");
            }
            kfSet.unlock();
        }

        void bundleAdjustmentMotionOnly(
            const cv::Matx33d& intrinsics,
            const std::vector<cv::Vec3d>& worldPoints,
            const std::vector<cv::Vec2f>& imagePoints,
            cv::Matx33d& currExtrinsics_R,
            cv::Vec3d& currExtrinsics_t,
            const Configuration& config,
            const IMUInfo& imuInfo,
            const std::shared_ptr<View>& previousView,
            const std::vector<double>& viewGyro,
            const std::vector<double>& viewAccel) {

            // New empty factor graph
            auto fg = mw_ceres::FactorGraph();

            // For verbose use
            std::stringstream strBuffer;
            // The following object is required to capture all cout statements from ceres library.
            // This object gets destroyed when it goes out of scope.
            std::shared_ptr<CoutRedirectForCeres> tempCoutObj;

            // Measurement information matrix
            cv::Vec4d measInfo{ std::pow(intrinsics(1, 1) / 1.5, 2), 0.0, 0.0, std::pow(intrinsics(1, 1) / 1.5, 2) };

            // Used to store the predicted velocities and biases
            std::vector<double> predictedPose(7, 0.0);
            std::vector<double> predictedVel(7, 0.0);

            constexpr size_t poseDim{ 7 };
            constexpr size_t pointDim{ 3 };
            constexpr size_t biasDim{ 6 };
            constexpr size_t velocityDim{ 3 };

            constexpr int prevViewId = 0; //(prevViewId=0, prevVelocityId=1, prevBiasId=2)
            constexpr int currViewId = 3; //(currViewId=3, currVelocityId=4, currBiasId=5)

            // Fill factor graph

            // Grab pose value of the current view and convert it to the format [x, y, z, qx, qy, qz, qw]
            cv::Matx33d currPose_R;
            cv::Vec3d currPose_t;
            extr2pose(currExtrinsics_R, currExtrinsics_t, currPose_R, currPose_t);
            cv::Matx44d pose = rt2pose(currPose_R, currPose_t);

            std::vector<double> viewPose;
            viewPose.reserve(poseDim);
            tform2eigenquatpose(pose, viewPose);

            cv::Matx44d previousPose = previousView->getPose();
            std::vector<double> prevPose;
            viewPose.reserve(poseDim);
            tform2eigenquatpose(previousPose, prevPose);

            constexpr int offset = 6; // Avoid duplicate IDs with the view (offset=currBiasId+1)
            for (int pointIdx = 0; pointIdx < static_cast<int>(worldPoints.size()); pointIdx++) {

                int pointId = pointIdx + offset;
                // Get 3-D xyz point location
                cv::Vec3d xyzLocation = worldPoints[pointIdx];

                // Get 2-D image point location
                cv::Vec2f keyPt = imagePoints[pointIdx];

                // Set up new camera factor
                std::vector<int> ids{ currViewId, pointId };
                auto camFactor = std::make_unique<mw_ceres::FactorCameraSE3AndPointXYZ>(ids);

                // Set up the measurements vector aka pixel positions
                cv::Vec4d measurement{ keyPt[0] - intrinsics(0, 2), keyPt[1] - intrinsics(1, 2), intrinsics(0, 0), intrinsics(1, 1) };

                // Add factor to factor graph
                camFactor->setMeasurement(measurement.val);
                camFactor->setInformation(measInfo.val);
                fg.addFactor(std::move(camFactor));

                // Set the initial guess of the xyz points
                fg.setVariableState({ pointId }, { xyzLocation.val[0],xyzLocation.val[1],xyzLocation.val[2] }, pointDim);
                fg.fixVariable({ pointId });
            }


            if (imuInfo.isIMUAligned) {

                std::vector<double> g{ 0.0, 0.0, config.imuParams.gravityDirection * 9.81 };

                std::vector<double> covGyroNoise{config.imuParams.gyroscopeNoise, 0.0, 0.0, 0.0, config.imuParams.gyroscopeNoise, 0.0, 0.0, 0.0, config.imuParams.gyroscopeNoise};
                std::vector<double> covAccNoise{config.imuParams.accelerometerNoise, 0.0, 0.0, 0.0, config.imuParams.accelerometerNoise, 0.0, 0.0, 0.0, config.imuParams.accelerometerNoise};
                std::vector<double> covGyroBiasNoise{config.imuParams.gyroscopeBiasNoise, 0.0, 0.0, 0.0, config.imuParams.gyroscopeBiasNoise, 0.0, 0.0, 0.0, config.imuParams.gyroscopeBiasNoise};
                std::vector<double> covAccBiasNoise{config.imuParams.accelerometerBiasNoise, 0.0, 0.0, 0.0, config.imuParams.accelerometerBiasNoise, 0.0, 0.0, 0.0, config.imuParams.accelerometerBiasNoise};

                std::vector<int> imuIDS{ prevViewId, prevViewId + 1, prevViewId + 2, currViewId, currViewId + 1, currViewId + 2 };
                //{ POSE(i-1), VELOCITY(i-1), BIAS(i-1), POSE(i), VELOCITY(i), BIAS(i) }

                // TODO: Remove const_cast when FactorIMU const issue is fixed by Nav
                std::vector<double>& viewGyroNonConst = const_cast<std::vector<double>&>(viewGyro);
                std::vector<double>& viewAccelNonConst = const_cast<std::vector<double>&>(viewAccel);
                IMUInfo& imuInfoNonConst = const_cast<IMUInfo&>(imuInfo);

                // Set up the IMU factor
                auto imuFactor = std::make_unique<mw_ceres::FactorIMU>(imuIDS.data(), config.imuParams.sampleRate, g.data(),
                    covGyroBiasNoise.data(), covAccBiasNoise.data(), covGyroNoise.data(), covAccNoise.data(),
                    viewGyroNonConst.data(), viewAccelNonConst.data(), viewGyroNonConst.size(), imuInfoNonConst.camToIMUTransform.val);

                // Extract the pose, velocity and bias of the previous view
                std::pair<std::vector<double>, std::vector<double>> prevVelocityAndBias = previousView->getVelocityAndBias();
                cv::Matx44d previousPose = previousView->getPose();

                std::vector<double> prevPose;
                viewPose.reserve(poseDim);
                tform2eigenquatpose(previousPose, prevPose);

                imuFactor->predict(prevVelocityAndBias.second.data(), prevPose.data(), prevVelocityAndBias.first.data(), predictedPose.data(), predictedVel.data());

                // Add the IMU factor to the factor graph
                fg.addFactor(std::move(imuFactor));

                fg.setVariableState({ prevViewId }, prevPose, poseDim);
                fg.setVariableState({ prevViewId + 1 }, prevVelocityAndBias.first, velocityDim);
                fg.setVariableState({ prevViewId + 2 }, prevVelocityAndBias.second, biasDim);

                fg.setVariableState({ currViewId }, predictedPose, poseDim);
                fg.setVariableState({ currViewId + 1 }, predictedVel, velocityDim);
                fg.setVariableState({ currViewId + 2 }, prevVelocityAndBias.second, biasDim);
            }
            else {

                // Add current view pose as variable state
                // Set the initial guess of the camera pose (after adding the factor)
                fg.setVariableState({ currViewId }, viewPose, poseDim);

            }

            if (imuInfo.isIMUAligned) {
                fg.fixVariable({ prevViewId + 1 });
                fg.fixVariable({ prevViewId + 2 });
            }

            // Optimize factor graph
            mw_ceres::CeresSolverOptions opt;
            opt.MaxNumIterations = config.baseParams.maxNumIterationsBA;
            opt.VerbosityLevel = 0;

            config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::PoseBeforeMotionBA");
            // Print pose in next line
            config.loggerPtr->logMessageV2("", { pose2String(pose) });

            // Enable verbose at Verbose=3
            if (config.baseParams.verbose == 3) {
                opt.VerbosityLevel = 2;
                // Replace cout buffer with string buffer, so that all cout statements go to string buffer
                tempCoutObj = std::make_shared<CoutRedirectForCeres>(strBuffer.rdbuf());
            }
            else {
                opt.VerbosityLevel = 0;
            }

            mw_ceres::CeresSolutionInfo solnInfo = fg.optimize(opt, { -1 }); //{ -1 } => optimize all non-fixed nodes


            // Grab updated pose
            std::vector<int> idv{ currViewId };
            eigenquatpose2tform(fg.getVariableState(idv), pose);

            if (config.baseParams.verbose == 3) {
                // Dump string buffer to logging
                config.loggerPtr->logMessageV3("vision::vslamVerboseCpp::MotionBAStart");
                config.loggerPtr->logMessageV3("", { strBuffer.str() });
                config.loggerPtr->logMessageV3("vision::vslamVerboseCpp::MotionBAEnd");
            }

            config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::PoseAfterMotionBA");
            // Print pose in next line
            config.loggerPtr->logMessageV2("", { pose2String(pose) });

            pose2rt(pose, currPose_R, currPose_t);
            extr2pose(currPose_R, currPose_t, currExtrinsics_R, currExtrinsics_t);
        }

    } // namespace vslam
} // namespace vision
