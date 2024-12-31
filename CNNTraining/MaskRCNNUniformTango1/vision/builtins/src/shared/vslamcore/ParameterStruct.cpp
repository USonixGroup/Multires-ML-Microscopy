////////////////////////////////////////////////////////////////////////////////
// Set default values for ParameterStruct
// 
// Copyright 2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/ParameterStruct.hpp"
#else
    #include "ParameterStruct.hpp"
#endif

#include <string>

BaseParams defaultBaseParams() {
    BaseParams params;

    // Image size
    params.nRows = 0;
    params.nCols = 0;

    // Map initialization
    params.minNumWorldPoints = 100;

    // Tracking last key frame
    params.maxReprojError = 4.f;
    params.minNumPnPPoints = 10;

    // Tracking local map
    params.minNumPointWeakFrame = 100;
    params.minNumMatches = 20;
    params.numSkippedFrames = 20;

    // Add new key frames
    params.minNumMatchesNewConnection = 5;

    // Features
    params.numFeatures = 1000;
    params.numFeaturesInit = 1000;
    params.scaleFactor = 1.2f;
    params.numLevels = 8;
    params.matchThreshold = 100;
    params.maxRatio = 0.9f;
    params.maxRatioInRadius = 1.f;
    params.minNumMatchesLoop = 50;

    params.verbose = 0;

    // Creating new map points
    params.maxRatioMapping = 0.9f;
    params.minNumMatchesMapping = 5;
	params.minSinParallax = 0.0175; // sind(1)
    params.maxCosParallax = 0.9986; // cosd(3)
	
	// Bundle adjustment
	params.minNumMatchesBA = 10;
    params.maxNumFixedViewsBA = 5;
    params.maxNumIterationsBA = 10;
    params.maxCeresRE = 5.99;


    // Pose graph optimization
    params.minNumMatchesPGO = 20;
    params.optimizationInterval = 20;
    params.maxNumIterationsPGO = 50;
	
    return params;
}

IMUParams defaultIMUParams() {
    IMUParams params;

    // Default values are based on factorIMUParameters()
    params.hasIMU = false;
    params.sampleRate = 100;
    params.maxNumIterationsIMUBA = 20;
    params.maxCeresREIMU = 8.99;
    params.gyroscopeBiasNoise = 1;
    params.accelerometerBiasNoise = 1;
    params.gyroscopeNoise = 1;
    params.accelerometerNoise = 1;
    params.gravityDirection = 1;

    return params;
}

MonoParams defaultMonoParams() {
    MonoParams params;

    // Map initialization
    params.maxReprojErrorMapInit = 2.4474; //std::sqrt(5.99)
    params.minCosParallaxInit = 0.9998; // cosd(1)
    params.maxRatioHomography = 0.75;

    return params;
}

StereoParams defaultStereoParams() {
    StereoParams params;

    params.minDisparity = 0;
    params.maxDisparity = 48; // disparityRange = [minDisparity, maxDisparity]
    params.maxDepthFactor = 200.f;
    params.uniquenessThreshold = 15;

    return params;
}

RGBDParams defaultRGBDParams() {
    RGBDParams params;

    params.depthScaleFactor = 5000.f;
    params.depthRange[0] = 0.1f;
    params.depthRange[1] = 5.f;

    return params;
}