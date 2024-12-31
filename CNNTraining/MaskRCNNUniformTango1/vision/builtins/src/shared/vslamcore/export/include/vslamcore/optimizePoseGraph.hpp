#ifndef POSE_GRAPH_OPTIMIZATION_CERES_HPP
#define POSE_GRAPH_OPTIMIZATION_CERES_HPP

#include "Connection.hpp"
#include "Configuration.hpp"

namespace vision {
    namespace vslam {
        /**
        * @brief Optimize a similarity pose graph in the monocular visual SLAM workflow
        *
        * @param[in, out] viewIdsAndPoses view Ids and view poses. View poses are updated after optimization.
        * @param[in] odometryConnections odometry connections, specified as a vector of pointers to Connection,
        *            which contains the relative orientation R and relative translation t. The relative pose of 
        *            an odometry connection is represented as a 3-D rigid transform:
        *            | R, t |
        *            | 0, 1 |
        * @param[in] loopConnections loop closure connections, specified as a vector of pairs. Each pair contains
        *            a pointer to Connection, which contains the relative orientation R and relative translation t,  
        *            and the corresponding scale s. The relative pose of a loop closure connection is represented 
        *            as a 3-D similarity transform:
        *            | s*R, t |
        *            | 0,   1 |
        * @param[out] optimScales, optimized scales associated with view poses, returned as a vector of double values.        
        *             These values are used to update the map after similarity pose graph optimization in the monocular 
        *             visual SLAM workflow
        * @param[in] config contains the configuration of the optimizer.
        */
        void optimizePoseGraph(
            std::pair<std::vector<int>, std::vector<cv::Matx44d>>& viewIdsAndPoses,
            const std::vector<std::shared_ptr<Connection>>& odometryConnections,
            const std::vector<std::pair<std::shared_ptr<Connection>, double>>& loopConnections,
            std::vector<double>& optimScales,
            const Configuration& config);

        /**
        * @brief Optimize a rigid pose graph in the stereo/RGB-D visual SLAM workflow
        *
        * @param[in, out] viewIdsAndPoses view Ids and view poses. View poses are updated after optimization.
        * @param[in] odometryConnections odometry connections, specified as a vector of pointers to Connection,
        *            which contains the relative orientation R and relative translation t. The relative pose of 
        *            an odometry connection is represented as a 3-D rigid transform:
        *            | R, t |
        *            | 0, 1 |
        * @param[in] loopConnections loop closure connections, specified as a vector of pointers to Connection,
        *            which contains the relative orientation R and relative translation t. The relative pose of 
        *            a loop closure connection is represented as a 3-D rigid transform:
        *            | R, t |
        *            | 0, 1 |
        * @param[in] config contains the configuration of the optimizer.
        */
        void optimizePoseGraph(
            std::pair<std::vector<int>, std::vector<cv::Matx44d>>& viewIdsAndPoses,
            const std::vector<std::shared_ptr<Connection>>& odometryConnections,
            const std::vector<std::shared_ptr<Connection>>& loopConnections,
            const Configuration& config);
    }
}
#endif //POSE_GRAPH_OPTIMIZATION_CERES_HPP
