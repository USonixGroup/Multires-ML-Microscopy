////////////////////////////////////////////////////////////////////////////////
// ID generator class, used to generate pointIds and viewIds
// 
// Copyright 2023-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/NodeIDGenerator.hpp"
#else
    #include "NodeIDGenerator.hpp"
#endif

namespace vision {
    namespace vslam {
        
        NodeIDGenerator::NodeIDGenerator() : counter(1) {}
        
        NodeID NodeIDGenerator::newIDs(NodeType type) {
            NodeID out1 = { counter, type };
            allIDs.push_back(out1);
            counter++;
            return out1;
        }

        std::vector<NodeID> NodeIDGenerator::newIDs(NodeType type, int N) {
            std::vector<NodeID> out2;
            out2.reserve(N);
            for (int i = 0; i != N; ++i) {
                NodeID id = { counter, type };
                allIDs.push_back(id);
                out2.push_back(id);
                counter++;
            }
            return out2;
        }

        std::vector<NodeID> NodeIDGenerator::newIDs(SensorConfig sc) {
            std::vector<NodeID> out2;
            out2.reserve(4);

            if (sc == SensorConfig::CAMERA_ONLY_SE3) {
                NodeID id = { counter, NodeType::POSE_SE3 };
                out2.push_back(id);
                counter++;
            }
            else if (sc == SensorConfig::CAMERA_ONLY_SIM3) {
                NodeID id1 = { counter, NodeType::POSE_SE3 };
                NodeID id2 = { counter + 1, NodeType::SCALE };
                counter = counter + 2;
                out2.insert(out2.end(), { id1, id2 });
            }
            else if (sc == SensorConfig::CAMERA_IMU_SE3) {
                NodeID id1 = { counter, NodeType::POSE_SE3 };
                NodeID id2 = { counter + 1, NodeType::VEL3 };
                NodeID id3 = { counter + 2, NodeType::IMU_BIAS };
                counter = counter + 3;
                out2.insert(out2.end(), { id1, id2, id3 });
            }
            else if (sc == SensorConfig::CAMERA_IMU_SIM3) {
                NodeID id1 = { counter, NodeType::POSE_SE3 };
                NodeID id2 = { counter + 1, NodeType::SCALE };
                NodeID id3 = { counter + 2, NodeType::VEL3 };
                NodeID id4 = { counter + 3, NodeType::IMU_BIAS };
                counter = counter + 4;
                out2.insert(out2.end(), { id1, id2, id3, id4 });
            }

            return out2;
        }

        
    } // namespace vslam
} // namespace vision