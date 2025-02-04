////////////////////////////////////////////////////////////////////////////////
//  ParameterStruct.hpp
//
//  ParameterStruct header file declaring C-style structs used in Configuration
//
//  Copyright 2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef PARAMETERSTRUCT_H
#define PARAMETERSTRUCT_H

/**
* @brief Struct containing generic visual SLAM parameters
*/
typedef struct {
    // Image size
    int nRows;
    int nCols;

    // Map initialization
    // Minimum number of points required to initialize the map from two images (mono)
    // or a pair of stereo/RGB-D images.
    int minNumWorldPoints;

    // Tracking last key frame
    // Maximum reprojection error for PnP solver. Increase the value to find more 
    // 2-D to 3-D pairs
    float maxReprojError;
    // Minimum number of 2-D to 3-D pairs required to for PnP solver. Decrease the value when the
    // tracking is weak to allow recovery from weak tracking, e.g., the images are downsampled.
    int minNumPnPPoints;

    // Tracking local map
    // If the number of points tracked in the current frame is above minNumPointWeakFrame, then the
    // frame is not a keyframe since the tracking is strong.
    int minNumPointWeakFrame;
    // Minimum number of matched pairs for a keyframe to be a local keyframe. Reduce the value
    // when the tracking is weak and more local keyframes are required. These local keyframes will
    // be refined during bundle adjustment. Increase the value to improve accuracy, though this may
    // result in reduced speed.
    int minNumMatches;
    // Maximum number of skipped frame allowed.
    int numSkippedFrames;

    // Add new key frames
    // Minimum number of matched features between two keyframes. Reduce this value to create a more 
    // dense keyframe set.
    int minNumMatchesNewConnection;

    // Feature extraction and matching
    // Default number of features extracted from each frame. Increase this value when the image resolution
    // is large.
    int numFeatures;
    // Default number of features extracted from the first two keyframes to initialize the map.
    int numFeaturesInit;
    // Scale factor
    float scaleFactor;
    // Number of pyramid levels
    int numLevels;
    // Match threshold for ORB feature
    int matchThreshold;
    // Threshold of ratio check when matching features without spatial constraint. Increase this value to
    // find more matched features though more outlier can be introduced.
    float maxRatio;
    // Threshold of ratio check when matching features with spatial constraint. Increase this value to
    // find more matched features though more outlier can be introduced.
    float maxRatioInRadius;
    // Loop closure threshold. Decrease this value to find loop closures more aggressively.
    int minNumMatchesLoop;

    int verbose;

    // Creating new map points
    // Threshold of ratio check when matching features without spatial constraint. Increase this value to
    // find more matched features, thus more new 3-D map points, though more outlier can be introduced.
    float maxRatioMapping;
    // If the number of new matched pairs between the current keyframe and one of previous keyframes is
    // below minNumMatchesMapping, skip triangulation of the matched pairs.
    int minNumMatchesMapping;
    // If two keyframes are close, i.e., the camera movement doesn't result in a large parallax with respect
    // to the local map points, then don't triangulate these two views to create new map points.
    double minSinParallax;
    // For a 3-D map point created from triangulation to be valid, its parallax corresponding to the two key
    // frames should be more than maxCosParallax. Increase this value to create more new map points though 
    // they may be located very far away from the keyframes.
    double maxCosParallax;

    // Bundle adjustment
    // Minimum number of matched pairs to decide which local keyframes are involved in BA. Increase this
    // value to improve accuracy at slower speed.
    int minNumMatchesBA;
    // Minimum number of fixed keyframes to be fixed in BA. These fixed keyframes anchor the local keyframes
    // to avoid drastic changes in keyframe poses after optimization.
    int maxNumFixedViewsBA;
    // Maximum number of iteration when calling the Ceres solver for BA. Increase this value to improve accuracy  
    // at slower speed.
    int maxNumIterationsBA;
    // Maximum reprojection error allowed for a 3-D map point to be valid after BA. Increase this value to
    // keep more recently created map points though the map accuracy can be impacted.
    double maxCeresRE;

    // Pose graph optimization
    // Minimum number of matched pairs to decide which keyframes are involved in PGO. Increase this
    // value to include only strongly-connected keyframes which will allow more drastic change in the 
    // keyframe poses during optimization. Decrease this value to include more keyframes thus more constraints
    // in PGO. The change in the refined poses will be smaller.
    int minNumMatchesPGO;
    // optimizationInterval defines how frequent PGO is performed. Increase this value to improve speed.
    int optimizationInterval;
    // Maximum number of iterations when calling the Ceres solver for PGO. Increase this value to improve accuracy 
    // at slower speed.
    int maxNumIterationsPGO;
} BaseParams;

/**
* @brief Struct containing parameters for IMU
*/
typedef struct {
    bool hasIMU;
    int sampleRate;
    int maxNumIterationsIMUBA;
    double maxCeresREIMU;
    double gyroscopeBiasNoise;
    double accelerometerBiasNoise;
    double gyroscopeNoise;
    double accelerometerNoise;
    int gravityDirection;
} IMUParams;

/**
* @brief Struct containing parameters for monocular visual SLAM
*/
typedef struct {
    // Map initialization
    double maxReprojErrorMapInit;
    double minCosParallaxInit;
    double maxRatioHomography;
} MonoParams;

/**
* @brief Struct containing parameters for stereo visual SLAM
*/
typedef struct {
    int minDisparity;
    int maxDisparity;
    float maxDepthFactor;
    int uniquenessThreshold;
} StereoParams;

/**
* @brief Struct containing parameters for RGBD visual SLAM
*/
typedef struct {
    // Depth image
    float depthScaleFactor;
    float depthRange[2];
} RGBDParams;

/**
* @brief Initializes BaseParams C-style struct with default values
*
* @return BaseParams struct
*/
BaseParams defaultBaseParams();

/**
* @brief Initializes IMUParams C-style struct with default values
*
* @return IMUParams struct
*/
IMUParams defaultIMUParams();

/**
* @brief Initializes MonoParams C-style struct with default values
*
* @return MonoParams struct
*/
MonoParams defaultMonoParams();

/**
* @brief Initializes StereoParams C-style struct with default values
*
* @return StereoParams struct
*/
StereoParams defaultStereoParams();

/**
* @brief Initializes RGBDParams C-style struct with default values
*
* @return RGBDParams struct
*/
RGBDParams defaultRGBDParams();

#endif //PARAMETERSTRUCT_H
