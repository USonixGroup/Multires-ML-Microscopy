//////////////////////////////////////////////////////////////////////////////
// Utilities for ArUco marker related operations.
//
// Copyright 2023 The MathWorks, Inc.
//////////////////////////////////////////////////////////////////////////////

#include "opencv2/opencv.hpp"

namespace vision {
namespace aruco  {

std::unordered_map<std::string, cv::aruco::PredefinedDictionaryType> getSupportedFamilies();

cv::aruco::Dictionary getPredefinedDictionary(const std::string& family);

} // namespace aruco
} // namespace vision