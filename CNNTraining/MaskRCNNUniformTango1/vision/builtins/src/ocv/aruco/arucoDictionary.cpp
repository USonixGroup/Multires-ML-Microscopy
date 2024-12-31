//////////////////////////////////////////////////////////////////////////////
// Utilities for ArUco marker related operations.
//
// Copyright 2023 The MathWorks, Inc.
//////////////////////////////////////////////////////////////////////////////

#include "aruco/arucoDictionary.hpp"

namespace vision {
namespace aruco  {

//////////////////////////////////////////////////////////////////////////////
// Helper function for the utilities in this file to return the list of  
// supported ArUco marker families.
//////////////////////////////////////////////////////////////////////////////
std::unordered_map<std::string, cv::aruco::PredefinedDictionaryType> getSupportedFamilies(){
    std::unordered_map<std::string, cv::aruco::PredefinedDictionaryType> supportedFamilies = {
        {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
        {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
        {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
        {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
        {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
        {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
        {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
        {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
        {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
        {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
        {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
        {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
        {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
        {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
        {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
        {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
        {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}
    };

    return supportedFamilies;
}

//////////////////////////////////////////////////////////////////////////////
// Utility function to get an OpenCV ArUco dictionary given a corresponding
// std::string family.
//////////////////////////////////////////////////////////////////////////////
cv::aruco::Dictionary getPredefinedDictionary(const std::string& familyKey){
    auto supportedFamilies = getSupportedFamilies();

    auto family = supportedFamilies.at(familyKey);
    
    return cv::aruco::getPredefinedDictionary(family);
}

    
} // namespace aruco
} // namespace vision