////////////////////////////////////////////////////////////////////////////////
// ID generator class, used to generate pointIds and viewIds
// 
// Copyright 2023-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////
#ifndef IDGENERATOR_HPP
#define IDGENERATOR_HPP


#include <unordered_map>
#include <utility>
#include <vector>

namespace vision {
    namespace vslam {
        enum class NodeType { POINT_XYZ, POSE_SE3, VEL3, IMU_BIAS, SCALE };

        enum class SensorConfig { CAMERA_ONLY_SE3, CAMERA_ONLY_SIM3, CAMERA_IMU_SE3, CAMERA_IMU_SIM3 };

        struct NodeID {
            int identifier;
            NodeType type;
        };

        class NodeIDGenerator {

        private:
            int counter;
            std::vector<NodeID> allIDs;

        public:
            NodeIDGenerator();
            NodeID newIDs(const NodeType type);
            std::vector<NodeID> newIDs(const NodeType type, const int N);
            std::vector<NodeID> newIDs(const SensorConfig sc);
        };
    }// namespace vslam
}// namespace vision
#endif //IDGENERATOR_HPP
