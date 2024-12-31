////////////////////////////////////////////////////////////////////////////////
// Pose graph optimization functions using the Ceres solver
// 
// Copyright 2023-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/optimizePoseGraph.hpp"
    #include "vslamcore/converter.hpp"
    #include "cerescodegen/factor_graph.hpp"
    #include "cerescodegen/common_factors_2.hpp"
    #include "cerescodegen/imu_factor.hpp"
    #include "cerescodegen/camera_projection_factor.hpp"
#else
    #include "optimizePoseGraph.hpp"
    #include "converter.hpp"
    #include "factor_graph.hpp"
    #include "common_factors_2.hpp"
    #include "imu_factor.hpp"
    #include "camera_projection_factor.hpp"
#endif

namespace vision {
    namespace vslam {
        void optimizePoseGraph(std::pair<std::vector<int>, std::vector<cv::Matx44d>>& viewIdsAndPoses,
            const std::vector<std::shared_ptr<Connection>>& odometryConnections,
            const std::vector<std::pair<std::shared_ptr<Connection>, double>>& loopConnections,
            std::vector<double>& optimScales,
            const Configuration& config) {

            const int numNodes = static_cast<int>(viewIdsAndPoses.first.size());
            const int numEdges = numNodes + static_cast<int>(loopConnections.size());

            constexpr size_t  poseDim = 7; // [x,y,z, qx, qy, qz, qw]

            // Create the factor graph
            auto fg = mw_ceres::FactorGraph();

            // Add odeometry edges
            for (const auto& conn : odometryConnections) {
                // Skip weak connections
                if (static_cast<int>(conn->getMatches().size()) < config.baseParams.minNumMatchesPGO)
                    continue;
                
                auto edgeIds = conn->getViewIdPair();

                // SIM3 factor ID: [pose1ID, scale1ID, pose2ID, scale2ID]
                std::vector<int> ids{ edgeIds.first, edgeIds.first+1, edgeIds.second, edgeIds.second+1};
                auto camRelPoseFctr = std::make_unique<mw_ceres::FactorTwoPosesSIM3>(ids);

                std::vector<double> edge;
                edge.reserve(poseDim+1); // +1 for scale
                tform2eigenquatpose(conn->getRelPose(), edge);
                edge.push_back(0);  // std::log(1)
                camRelPoseFctr->setMeasurement(edge.data());
                fg.addFactor(std::move(camRelPoseFctr));
            }

            // Add loop closure edges
            for (const auto& connAndScale : loopConnections) {

                auto edgeIds = connAndScale.first->getViewIdPair();

                // SIM3 factor ID: [pose1ID, scale1ID, pose2ID, scale2ID]
                std::vector<int> ids{ edgeIds.first, edgeIds.first+1, edgeIds.second, edgeIds.second+1 }; 
                auto camRelPoseFctr = std::make_unique<mw_ceres::FactorTwoPosesSIM3>(ids);

                std::vector<double> edge;
                edge.reserve(poseDim+1);  //+1 for scale
                tform2eigenquatpose(connAndScale.first->getRelPose(), edge);
                edge.push_back(std::log(connAndScale.second));
                camRelPoseFctr->setMeasurement(edge.data());
                fg.addFactor(std::move(camRelPoseFctr));
            }

            // Add absolute camera poses. This needs to happen after adding
            // the edges.
            for (int i = 0; i < numNodes; ++i) {
                std::vector<int> poseId{ viewIdsAndPoses.first[i] };
                std::vector<double> pose; 
                pose.reserve(poseDim);
                tform2eigenquatpose(viewIdsAndPoses.second[i], pose);
                fg.setVariableState(poseId, pose, poseDim);
            }


            // If using IMU, fix 15% of the poses and scales. If not, fix only the first pose and scale.
            if(config.imuParams.hasIMU){
                constexpr double fixedRatio{ 0.15 };
                int numFixedNodes = static_cast<int>(std::round(viewIdsAndPoses.first.size() * fixedRatio)); 
                std::vector<int> fixedViewIdVec, fixedScaleIdVec;
                fixedViewIdVec = std::vector<int>(viewIdsAndPoses.first.begin(), viewIdsAndPoses.first.begin() + numFixedNodes);
                fixedScaleIdVec = fixedViewIdVec;
                std::for_each(fixedScaleIdVec.begin(), fixedScaleIdVec.end(), [](int& d) { d += 1; });
                fg.fixVariable(fixedViewIdVec);
                fg.fixVariable(fixedScaleIdVec);
            }
            else{
                std::vector<int> fixedViewIdVec{ 0 }, fixedScaleIdVec{ 1 };
                fg.fixVariable(fixedViewIdVec);
                fg.fixVariable(fixedScaleIdVec);
            }

            // Optimize
            mw_ceres::CeresSolverOptions opt;
            opt.LinearSolverType = mw_ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
            opt.TrustRegionStrategyType = mw_ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
            opt.InitialTrustRegionRadius = 0.1;
            opt.VerbosityLevel = 0; 
            mw_ceres::CeresSolutionInfo solnInfo = fg.optimize(opt, {-1});
            if (!config.imuParams.hasIMU) {
                opt.MaxNumIterations = config.baseParams.maxNumIterationsPGO;
            }

            // Update absolute camera poses
            for (int i = 0; i < numNodes; ++i) {
                std::vector<int> poseId{ viewIdsAndPoses.first[i] }, scaleId{ viewIdsAndPoses.first[i]+1 };
                eigenquatpose2tform(fg.getVariableState(poseId), viewIdsAndPoses.second[i]);
                auto v = fg.getVariableState(poseId);
                optimScales.push_back(std::exp(fg.getVariableState(scaleId)[0]));
            }
        }

        void optimizePoseGraph(std::pair<std::vector<int>, std::vector<cv::Matx44d>>& viewIdsAndPoses,
            const std::vector<std::shared_ptr<Connection>>& odometryConnections,
            const std::vector<std::shared_ptr<Connection>>& loopConnections,
            const Configuration& config) {

            const int numNodes = static_cast<int>(viewIdsAndPoses.first.size());
            const int numEdges = numNodes + static_cast<int>(loopConnections.size());

            constexpr size_t  poseDim = 7; // [x,y,z, qx, qy, qz, qw]

            // Create the factor graph
            auto fg = mw_ceres::FactorGraph();

            // Add odeometry edges
            for (const auto& conn : odometryConnections) {

                // Skip weak connections
                if (static_cast<int>(conn->getMatches().size()) < config.baseParams.minNumMatchesPGO)
                    continue;

                auto edgeIds = conn->getViewIdPair();
                std::vector<int> ids{ edgeIds.first, edgeIds.second };
                auto camRelPoseFctr = std::make_unique<mw_ceres::FactorTwoPosesSE3>(ids);

                std::vector<double> edge;
                edge.reserve(poseDim);
                tform2eigenquatpose(conn->getRelPose(), edge);
                camRelPoseFctr->setMeasurement(edge.data());
                fg.addFactor(std::move(camRelPoseFctr));
            }

            // Add loop closure edges
            for (const auto& conn : loopConnections) {

                auto edgeIds = conn->getViewIdPair();
                std::vector<int> ids{ edgeIds.first, edgeIds.second };
                auto camRelPoseFctr = std::make_unique<mw_ceres::FactorTwoPosesSE3>(ids);

                std::vector<double> edge;
                edge.reserve(poseDim);
                tform2eigenquatpose(conn->getRelPose(), edge);
                camRelPoseFctr->setMeasurement(edge.data()); 
                fg.addFactor(std::move(camRelPoseFctr));
            }

            // Add absolute camera poses. This needs to happen after adding
            // the edges.
            for (int i = 0; i < numNodes; ++i) {
                std::vector<int> id{ viewIdsAndPoses.first[i] };
                std::vector<double> node;
                node.reserve(poseDim);
                tform2eigenquatpose(viewIdsAndPoses.second[i], node);
                fg.setVariableState(id, node, poseDim);
            }

            // Fix the very first view 
            std::vector<int> fixedViewIdVec{0};
            fg.fixVariable(fixedViewIdVec);

            // Optimize
            mw_ceres::CeresSolverOptions opt;
            opt.LinearSolverType = mw_ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
            opt.TrustRegionStrategyType = mw_ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
            opt.InitialTrustRegionRadius = 0.1;
            opt.MaxNumIterations = config.baseParams.maxNumIterationsPGO;
            opt.VerbosityLevel = 0; 
            mw_ceres::CeresSolutionInfo solnInfo = fg.optimize(opt, {-1});

            // Update absolute camera poses
            for (int i = 0; i < numNodes; ++i) {
                std::vector<int> id{ viewIdsAndPoses.first[i] };
                eigenquatpose2tform(fg.getVariableState(id), viewIdsAndPoses.second[i]);
            }
        }
    }
}