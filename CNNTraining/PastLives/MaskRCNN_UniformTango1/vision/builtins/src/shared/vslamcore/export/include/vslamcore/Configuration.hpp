////////////////////////////////////////////////////////////////////////////////
//  Configuration.hpp
//
//  Configuration header file declaring Configuration structs for Monocular, 
//  Stereo, and RGBD Visual SLAM
//
//  Copyright 2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP

#include <memory>
#include <utility>
#include <string>
#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/VSLAMLoggerBase.hpp"
#else
    #include "VSLAMLoggerBase.hpp"
#endif
#include "ParameterStruct.hpp"
#include "libmwvslamcore_util.hpp"

namespace vision {
    namespace vslam {
        /**
        * @brief Configuration struct containing all generic visual SLAM parameters
        *
        *   Note: This is the base class and it is not intended to be initialized explicitly
        */
        struct Configuration {
            // Struct containing generic visual SLAM parameters
            BaseParams baseParams;

            // Struct containing IMU parameters
            IMUParams imuParams;

            // Flag indicating if the Configuration is for a monocular camera
            const bool isMono;

			// Logging utility
            std::shared_ptr<VSLAMLoggerBase> loggerPtr;

            // Constructors used only by derived structs
            protected:
                Configuration(const bool isMonoIn);
                Configuration(const BaseParams& base, const IMUParams& imu, const bool isMonoIn);
        };

        /**
        * @brief Configuration struct containing all parameters for monocular visual SLAM
        */
        struct LIBMWVSLAMCORE_API ConfigurationMono : public Configuration {
            // Constructors for ConfigurationStereo used in StereoVisualSLAMImpl
            ConfigurationMono();
            ConfigurationMono(const BaseParams& base, const IMUParams& imu, const MonoParams& mono);

            MonoParams monoParams;
        };

        /**
        * @brief Configuration struct containing all parameters for stereo visual SLAM
        */
        struct LIBMWVSLAMCORE_API ConfigurationStereo : public Configuration {
            // Constructors for ConfigurationStereo used in StereoVisualSLAMImpl
            ConfigurationStereo();
            ConfigurationStereo(const BaseParams& base, const IMUParams& imu, const StereoParams& stereo);

            // Struct containing stereo parameters
            StereoParams stereoParams;
        };

        /**
        * @brief Configuration struct containing all parameters for RGBD visual SLAM
        */
        struct LIBMWVSLAMCORE_API ConfigurationRGBD : public Configuration {
            // Constructors for ConfigurationRGBD used in RGBDVisualSLAMImpl
            ConfigurationRGBD();
            ConfigurationRGBD(const BaseParams& base, const IMUParams& imu, const RGBDParams& rgbd);

            // Struct containing RGBD parameters
            RGBDParams rgbdParams;
        };

    }// namespace vslam
}// namespace vision
#endif //CONFIGURATION_HPP