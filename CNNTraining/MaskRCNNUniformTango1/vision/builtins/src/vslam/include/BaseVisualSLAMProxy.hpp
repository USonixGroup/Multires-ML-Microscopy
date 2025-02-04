////////////////////////////////////////////////////////////////////////////////
//  BaseVisualSLAMProxy.hpp
//
//  This is the BaseVisualSLAM proxy class template providing interface to the
//  BaseVisualSLAMInterface class from MATLAB.
//
//  Copyright 2023-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef BASEVISUALSLAMPROXY_HPP
#define BASEVISUALSLAMPROXY_HPP

#include "mcos.h"
#include "matrix/unique_mxarray_ptr.hpp"
#include "../../ocv/include/ocvutils.hpp"

#include "vslamcore/libmwvslamcore_util.hpp"

namespace vision {
    namespace vslam {

        template<typename TSLAMImpl>
        class BaseVisualSLAMProxy {
            public:
                BaseVisualSLAMProxy();
                virtual ~BaseVisualSLAMProxy();

                ////////////////////////////////////////////////////////////////////////////////
                // Initialize the SLAM system
                ////////////////////////////////////////////////////////////////////////////////
                virtual void configure(int nlhs, mcos::COSValue* plhs, int nrhs, const mcos::COSValue* prhs) = 0;

                ////////////////////////////////////////////////////////////////////////////////
                // Process image
                ////////////////////////////////////////////////////////////////////////////////
                virtual void addFrame(int nlhs, mcos::COSValue* plhs, int nrhs, const mcos::COSValue* prhs) = 0;

                ////////////////////////////////////////////////////////////////////////////////
                // Data queries
                ////////////////////////////////////////////////////////////////////////////////
                void getCameraPoses(int nlhs, mcos::COSValue* plhs, int nrhs, const mcos::COSValue* prhs);
                void getKeyFrameIndex(int nlhs, mcos::COSValue* plhs, int nrhs, const mcos::COSValue* prhs);
                void getViewIDs(int nlhs, mcos::COSValue* plhs, int nrhs, const mcos::COSValue* prhs);
                void getWorldPoints(int nlhs, mcos::COSValue* plhs, int nrhs, const mcos::COSValue* prhs);
                void getNumTrackedPoints(int nlhs, mcos::COSValue* plhs, int nrhs, const mcos::COSValue* prhs);
                void hasNewKeyFrame(int nlhs, mcos::COSValue* plhs, int nrhs, const mcos::COSValue* prhs);
                void getLogFileName(int nlhs, mcos::COSValue* plhs, int nrhs, const mcos::COSValue* prhs);
                void getViewIMUMeasurements(int nlhs, mcos::COSValue* plhs, int nrhs, const mcos::COSValue* prhs);

                ////////////////////////////////////////////////////////////////////////////////
                // Status queries
                ////////////////////////////////////////////////////////////////////////////////
                void isMapInitialized(int nlhs, mcos::COSValue* plhs, int nrhs, const mcos::COSValue* prhs);
                void isLoopRecentlyClosed(int nlhs, mcos::COSValue* plhs, int nrhs, const mcos::COSValue* prhs);
                void isDone(int nlhs, mcos::COSValue* plhs, int nrhs, const mcos::COSValue* prhs);
                void reset(int nlhs, mcos::COSValue* plhs, int nrhs, const mcos::COSValue* prhs);

            protected:

                ////////////////////////////////////////////////////////////////////////////////
                // Vocabulary file query
                ////////////////////////////////////////////////////////////////////////////////
                std::string getVocabFilePath();

                ////////////////////////////////////////////////////////////////////////////////
                // Visual SLAM Impl object
                ////////////////////////////////////////////////////////////////////////////////
                std::shared_ptr<TSLAMImpl> visualSLAMImplObj;
        };

    } // namespace vslam
} // namespace vision
#endif // BASEVISUALSLAMPROXY_HPP