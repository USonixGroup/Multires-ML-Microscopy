////////////////////////////////////////////////////////////////////////////////
//  BaseVisualSLAMProxy.cpp
//
//  This is the implementation of BaseVisualSLAM proxy class providing interface
//  to the BaseVisualSLAMInterface class from MATLAB.
//
//  Copyright 2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////
#include <BaseVisualSLAMProxy.hpp>
#include <ocvmex/libmwocvmex_util.hpp>
#include <ocvmex/ocvmex_published_c_api.hpp>

#include "i18n/filesystem/utility.hpp"
#include "i18n/ustring_conversions.hpp"
#include "i18n/MessageCatalog.hpp"
#include "resources/vision/vslam_utils.hpp"

#include "vslamcore/MonoVisualSLAMImpl.hpp"
#include "vslamcore/StereoVisualSLAMImpl.hpp"
#include "vslamcore/RGBDVisualSLAMImpl.hpp"

#include "SecretHandshake.hpp"

namespace vision {
    namespace vslam {

        template<typename TSLAMImpl>
        BaseVisualSLAMProxy<TSLAMImpl>::BaseVisualSLAMProxy(){
            // License checkout
            checkoutNavLicense();
        }

        template<typename TSLAMImpl>
        BaseVisualSLAMProxy<TSLAMImpl>::~BaseVisualSLAMProxy() = default;

        template<typename TSLAMImpl>
        void BaseVisualSLAMProxy<TSLAMImpl>::getCameraPoses(int nlhs,
            mcos::COSValue* plhs,
            int nrhs,
            const mcos::COSValue* prhs) {

            constexpr int maxlhs(1), minrhs(1), maxrhs(1);
            mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);

            std::vector<cv::Matx44d> camPoses = visualSLAMImplObj->getCameraPoses().second;

            const int numCameras = camPoses.size();
            constexpr char rigid3dClassName[] = "rigidtform3d";
            constexpr char transformationMatrixPropertyName[] = "A";

            mxArray* out = omCreateMCOSMatrix(numCameras, 1, rigid3dClassName);
            for (int n = 0; n < numCameras; n++) {
                mxArray* mxTform = mxArrayFromMatx(camPoses[n]);
                const mxArray* inputs[] = { mxTform };
                mxArray* mxTformObj = omConstructObjectWithClient(rigid3dClassName, 1, inputs, COSGetPublicClient());
                mcos::COSInterfacePtr cosI = omGetArrayElement(mxTformObj, 0);
                omSetArrayElement(out, n, cosI);

                mxDestroyArray(mxTform);
                mxDestroyArray(mxTformObj);
            }
            plhs[0] = mcos::COSValue(out);
        }

        template<typename TSLAMImpl>
        void BaseVisualSLAMProxy<TSLAMImpl>::getKeyFrameIndex(int nlhs, mcos::COSValue* plhs, int nrhs, const mcos::COSValue* prhs) {
            constexpr int maxlhs(1), minrhs(1), maxrhs(1);
            mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);

            auto keyFrameIDs(visualSLAMImplObj->getKeyFrameIndex());

            const int numIds = keyFrameIDs.size();
            matrix::unique_mxarray_ptr out = matrix::create(1, numIds, mxINT32_CLASS, mxREAL);
            int* ids = matrix::getTypedData<int>(out.get());
            for (int i = 0; i < numIds; i++) {
                ids[i] = keyFrameIDs[i] + 1; // +1 MATLAB indexing
            }
            plhs[0] = mcos::COSValue(matrix::to_matlab(std::move(out)));
        }

        template<typename TSLAMImpl>
        void BaseVisualSLAMProxy<TSLAMImpl>::getViewIDs(int nlhs, mcos::COSValue* plhs, int nrhs, const mcos::COSValue* prhs) {
            constexpr int maxlhs(1), minrhs(1), maxrhs(1);
            mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);

            auto viewIDs(visualSLAMImplObj->getViewIDs());

            const int numIds = viewIDs.size();
            matrix::unique_mxarray_ptr out = matrix::create(1, numIds, mxINT32_CLASS, mxREAL);
            int* ids = matrix::getTypedData<int>(out.get());
            for (int i = 0; i < numIds; i++) {
                ids[i] = viewIDs[i];
            }
            plhs[0] = mcos::COSValue(matrix::to_matlab(std::move(out)));
        }

        template<typename TSLAMImpl>
        void BaseVisualSLAMProxy<TSLAMImpl>::getWorldPoints(int nlhs,
            mcos::COSValue* plhs,
            int nrhs,
            const mcos::COSValue* prhs) {

            constexpr int maxlhs(1), minrhs(1), maxrhs(1);
            mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);

            std::vector<cv::Vec3d> worldPoints = visualSLAMImplObj->getWorldPoints();

            const int numWorldPoints = worldPoints.size();

            matrix::unique_mxarray_ptr out = matrix::create(numWorldPoints, 3, mxDOUBLE_CLASS, mxREAL);

            double* xyzPoints = matrix::getTypedData<double>(out.get());

            for (int i = 0; i != numWorldPoints; ++i) {
                xyzPoints[i] = worldPoints[i][0];
                xyzPoints[i + numWorldPoints] = worldPoints[i][1];
                xyzPoints[i + numWorldPoints * 2] = worldPoints[i][2];
            }

            plhs[0] = mcos::COSValue(matrix::to_matlab(std::move(out)));
        }

        template<typename TSLAMImpl>
        void BaseVisualSLAMProxy<TSLAMImpl>::getNumTrackedPoints(int nlhs,
            mcos::COSValue* plhs,
            int nrhs,
            const mcos::COSValue* prhs) {

            constexpr int maxlhs(1), minrhs(1), maxrhs(1);
            mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);

            matrix::unique_mxarray_ptr out = matrix::create(1, 1, mxDOUBLE_CLASS, mxREAL);
            double* outPtr = matrix::getTypedData<double>(out.get());
            *outPtr = visualSLAMImplObj->getNumTrackedPoints();
            plhs[0] = matrix::to_matlab(std::move(out));
        }

        template<typename TSLAMImpl>
        void BaseVisualSLAMProxy<TSLAMImpl>::hasNewKeyFrame(int nlhs,
            mcos::COSValue* plhs,
            int nrhs,
            const mcos::COSValue* prhs) {
            constexpr int maxlhs(1), minrhs(1), maxrhs(1);
            mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);

            auto out = matrix::create_logical(1, 1);
            bool* outPtr = mxGetLogicals(out.get());
            *outPtr = visualSLAMImplObj->hasNewKeyFrame();
            plhs[0] = matrix::to_matlab(std::move(out));
        }

        template<typename TSLAMImpl>
        void BaseVisualSLAMProxy<TSLAMImpl>::isDone(int nlhs,
            mcos::COSValue* plhs,
            int nrhs,
            const mcos::COSValue* prhs) {
            constexpr int maxlhs(1), minrhs(1), maxrhs(1);
            mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);

            auto out = matrix::create_logical(1, 1);
            bool* outPtr = mxGetLogicals(out.get());
            *outPtr = visualSLAMImplObj->isDone();
            plhs[0] = matrix::to_matlab(std::move(out));
        }

        template<typename TSLAMImpl>
        void BaseVisualSLAMProxy<TSLAMImpl>::isMapInitialized(int nlhs,
            mcos::COSValue* plhs,
            int nrhs,
            const mcos::COSValue* prhs) {

            constexpr int maxlhs(1), minrhs(1), maxrhs(1);
            mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);

            auto out = matrix::create_logical(1, 1);
            bool* outPtr = mxGetLogicals(out.get());
            *outPtr = visualSLAMImplObj->getIsMapInitialized();
            plhs[0] = matrix::to_matlab(std::move(out));
        }

        template<typename TSLAMImpl>
        void BaseVisualSLAMProxy<TSLAMImpl>::isLoopRecentlyClosed(int nlhs,
            mcos::COSValue* plhs,
            int nrhs,
            const mcos::COSValue* prhs) {

            constexpr int maxlhs(1), minrhs(1), maxrhs(1);
            mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);

            auto out = matrix::create_logical(1, 1);
            bool* outPtr = mxGetLogicals(out.get());
            *outPtr = visualSLAMImplObj->getIsLoopRecentlyClosed();
            plhs[0] = matrix::to_matlab(std::move(out));
        }

        template<typename TSLAMImpl>
        void BaseVisualSLAMProxy<TSLAMImpl>::getLogFileName(int nlhs,
            mcos::COSValue* plhs,
            int nrhs,
            const mcos::COSValue* prhs) {

            constexpr int maxlhs(1), minrhs(1), maxrhs(1);
            mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);

            std::string retval(visualSLAMImplObj->getLogFileName());
            plhs[0] = mxCreateString(retval.c_str());
        }

        template<typename TSLAMImpl>
        void BaseVisualSLAMProxy<TSLAMImpl>::reset(int nlhs,
            mcos::COSValue* plhs,
            int nrhs,
            const mcos::COSValue* prhs) {
            constexpr int maxlhs(0), minrhs(1), maxrhs(1);
            mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);
            if (visualSLAMImplObj != nullptr)
                visualSLAMImplObj->reset();
        }

        template<typename TSLAMImpl>
        std::string BaseVisualSLAMProxy<TSLAMImpl>::getVocabFilePath() {
            std::string fileLocation = fl::i18n::to_string(fl::filesystem::get_install_path());
            fileLocation += "/toolbox/vision/builtins/src/shared/vslamcore/bagOfFeatures.bin.gz";
            return fileLocation;
        }

        template<typename TSLAMImpl>
        void BaseVisualSLAMProxy<TSLAMImpl>::getViewIMUMeasurements(int nlhs,
            mcos::COSValue* plhs,
            int nrhs,
            const mcos::COSValue* prhs) {

            constexpr int maxlhs(2), minrhs(2), maxrhs(2);
            mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);

            const mxArray* vID = mcos::pointerValue<const mxArray*>(prhs[1]);
            int viewId = static_cast<int>(mxGetScalar(vID));

            std::pair< std::vector<double>, std::vector<double> > imuMeasurements = visualSLAMImplObj->getViewIMUMeasurements(viewId);

            std::vector<double> viewGyro = imuMeasurements.first;
            std::vector<double> viewAccel = imuMeasurements.second;

            const int numMeasurements = viewGyro.size() / 3;
            matrix::unique_mxarray_ptr outG = matrix::create(numMeasurements, 3, mxDOUBLE_CLASS, mxREAL);
            matrix::unique_mxarray_ptr outA = matrix::create(numMeasurements, 3, mxDOUBLE_CLASS, mxREAL);

            double* gyroMeasurements = matrix::getTypedData<double>(outG.get());
            double* AccelMeasurements = matrix::getTypedData<double>(outA.get());

            int index = 0;

            for (int i = 0; i != numMeasurements; ++i) {
                gyroMeasurements[i] = viewGyro[index];
                gyroMeasurements[i + numMeasurements] = viewGyro[index + 1];
                gyroMeasurements[i + numMeasurements * 2] = viewGyro[index + 2];

                AccelMeasurements[i] = viewAccel[index];
                AccelMeasurements[i + numMeasurements] = viewAccel[index + 1];
                AccelMeasurements[i + numMeasurements * 2] = viewAccel[index + 2];

                index = index + 3;
            }

            plhs[0] = mcos::COSValue(matrix::to_matlab(std::move(outG)));
            plhs[1] = mcos::COSValue(matrix::to_matlab(std::move(outA)));

        }

        template class BaseVisualSLAMProxy<MonoVisualSLAMImpl>;
        template class BaseVisualSLAMProxy<StereoVisualSLAMImpl>;
        template class BaseVisualSLAMProxy<RGBDVisualSLAMImpl>;

    } // namespace vslam
} // namespace vision