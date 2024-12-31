////////////////////////////////////////////////////////////////////////////////
//  StereoVisualSLAMProxy.hpp
//
//  This is the StereoVisualSLAM proxy class providing interface to the
//  StereoVisualSLAM class from MATLAB.
//
//  Copyright 2023-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef STEREOVISUALSLAMPROXY_HPP
#define STEREOVISUALSLAMPROXY_HPP

#include <BaseVisualSLAMProxy.hpp>

#include "vslamcore/StereoVisualSLAMImpl.hpp"

namespace vision {
    namespace vslam {

        class StereoVisualSLAMProxy : public BaseVisualSLAMProxy<StereoVisualSLAMImpl> {
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
#endif // STEREOVISUALSLAMPROXY_HPP