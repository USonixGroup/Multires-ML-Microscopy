////////////////////////////////////////////////////////////////////////////////
//  RGBDVisualSLAMProxy.hpp
//
//  This is the RGBDVisualSLAM proxy class providing interface to the
//  RGBDVisualSLAM class from MATLAB.
//
//  Copyright 2023-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef RGBDVISUALSLAMPROXY_HPP
#define RGBDVISUALSLAMPROXY_HPP

#include <BaseVisualSLAMProxy.hpp>

#include "vslamcore/RGBDVisualSLAMImpl.hpp"

namespace vision {
    namespace vslam {

        class RGBDVisualSLAMProxy : public BaseVisualSLAMProxy<RGBDVisualSLAMImpl> {
            public:
                ////////////////////////////////////////////////////////////////////////////////
                // Initialize the SLAM system
                ////////////////////////////////////////////////////////////////////////////////
                void configure(int nlhs, mcos::COSValue* plhs, int nrhs, const mcos::COSValue* prhs) override;

                ////////////////////////////////////////////////////////////////////////////////
                // Process image
                ////////////////////////////////////////////////////////////////////////////////
                void addFrame(int nlhs, mcos::COSValue* plhs, int nrhs, const mcos::COSValue* prhs) override;

        };

    } // namespace vslam
} // namespace vision
#endif // RGBDVISUALSLAMPROXY_HPP
