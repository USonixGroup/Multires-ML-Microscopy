////////////////////////////////////////////////////////////////////////////////
//  MonoVisualSLAMProxy.hpp
//
//  This is the MonoVisualSLAM proxy class providing interface to the
//  MonoVisualSLAM class from MATLAB.
//
//  Copyright 2022-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef MONOVISUALSLAMPROXY_HPP
#define MONOVISUALSLAMPROXY_HPP

#include <BaseVisualSLAMProxy.hpp>

#include "vslamcore/MonoVisualSLAMImpl.hpp"

namespace vision {
    namespace vslam {

        class MonoVisualSLAMProxy : public BaseVisualSLAMProxy<MonoVisualSLAMImpl> {
            public:
                ////////////////////////////////////////////////////////////////////////////////
                // Initialize the SLAM system
                ////////////////////////////////////////////////////////////////////////////////
                void configure(int nlhs, mcos::COSValue* plhs, int nrhs, const mcos::COSValue* prhs) override;

                ////////////////////////////////////////////////////////////////////////////////
                // Process image
                ////////////////////////////////////////////////////////////////////////////////
                void addFrame(int nlhs, mcos::COSValue* plhs, int nrhs, const mcos::COSValue* prhs) override;
                void storeGravityRotationAndScale(int nlhs, mcos::COSValue* plhs, int nrhs, const mcos::COSValue* prhs);

        };

    } // namespace vslam
} // namespace vision
#endif // MONOVISUALSLAMPROXY_HPP