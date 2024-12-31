////////////////////////////////////////////////////////////////////////////////
// Definitions for Configuration constructors.
// 
// Copyright 2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/Configuration.hpp"
#else
    #include "Configuration.hpp"
#endif

namespace vision {
    namespace vslam {
        Configuration::Configuration(const bool isMonoIn) : isMono(isMonoIn), baseParams(defaultBaseParams()), imuParams(defaultIMUParams()), loggerPtr(std::make_shared<VSLAMLoggerBase>(0)) {}
        Configuration::Configuration(const BaseParams& base, const IMUParams& imu, const bool isMonoIn) : baseParams(base), imuParams(imu), isMono(isMonoIn), loggerPtr(std::make_shared<VSLAMLoggerBase>(base.verbose)) {}
        
        ConfigurationMono::ConfigurationMono() : Configuration(true), monoParams(defaultMonoParams()) {}
        ConfigurationMono::ConfigurationMono(const BaseParams& base, const IMUParams& imu, const MonoParams& mono) : Configuration(base, imu, true), monoParams(mono) {};

        ConfigurationStereo::ConfigurationStereo() : Configuration(false), stereoParams(defaultStereoParams()) {}
        ConfigurationStereo::ConfigurationStereo(const BaseParams& base, const IMUParams& imu, const StereoParams& stereo) : Configuration(base, imu, false), stereoParams(stereo) {};
        
        ConfigurationRGBD::ConfigurationRGBD() : Configuration(false), rgbdParams(defaultRGBDParams()) {}
        ConfigurationRGBD::ConfigurationRGBD(const BaseParams& base, const IMUParams& imu, const RGBDParams& rgbd) : Configuration(base, imu, false), rgbdParams(rgbd) {};
    }
}