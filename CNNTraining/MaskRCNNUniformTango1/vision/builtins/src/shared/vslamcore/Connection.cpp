////////////////////////////////////////////////////////////////////////////////
// Connection class storing relative camera pose and feature matches
// 
// Copyright 2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/Connection.hpp"
    #include "vslamcore/converter.hpp"
#else
    #include "Connection.hpp"
    #include "converter.hpp"
#endif

namespace vision {
    namespace vslam {

        Connection::Connection(
            const int viewId1,
            const int viewId2,
            const cv::Matx33d& rot,
            const cv::Vec3d& trans,
            const std::vector<std::pair<int, int>>& matches) :
            relRotation(rot), relTranslation(trans), matchedPairs(matches) {
            viewIdPair = std::make_pair(viewId1, viewId2);
        }

        Connection::~Connection() {}

        cv::Matx44d Connection::getRelPose() const {
            return rt2pose(relRotation, relTranslation);
        }

        cv::Matx33d Connection::getRelRotation() const {
            return relRotation;
        }

        cv::Vec3d Connection::getRelTranslation() const {
            return relTranslation;
        }

        std::pair<int, int> Connection::getViewIdPair() const {
            return viewIdPair;
        }

        std::vector<std::pair<int, int>> Connection::getMatches() const {
            return matchedPairs;
        }

        void Connection::setRelPose(const cv::Matx33d& rot, const cv::Vec3d& trans) {
            setRelRotation(rot);
            setRelTranslation(trans);
        }

        void Connection::setRelRotation(const cv::Matx33d& rot) {
            relRotation = rot;
        }

        void Connection::setRelTranslation(const cv::Vec3d& trans) {
            relTranslation = trans;
        }

        void Connection::addMatches(const std::vector<std::pair<int, int>>& matches) {
            matchedPairs.insert(matchedPairs.end(), matches.begin(), matches.end());
        }

        void Connection::addMatches(const std::pair<int, int>& matches) {
            matchedPairs.push_back(matches);
        }
    } // namespace vslam
} // namespace vision