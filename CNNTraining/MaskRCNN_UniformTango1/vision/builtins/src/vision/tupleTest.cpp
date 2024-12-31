///////////////////////////////////////////////////////////////////////////
// This file contains the built-in function to perform tuple test.
// Copyright 2020 The MathWorks, Inc.
//
// The MATLAB API is
// indices = tupleTest(points1, points2, randInds, scale, numIters)
//
// points1 : M-by-N matrix, each row represents an N-dimensional point.
// points2 : M-by-N matrix, each row represents an N-dimensional point.
// randInds: 1-by-P matrix, represents the random indices, where P equal to 
//           numIters * 3.
// scale   : A positive scalar value in range (0,1).
// numIters: A positive scalar value used to define the max iterations to
//           run tuple test.
//
// points1, points2 and scale can be of type single or double.
// randInds and numIters is of type uint32.
// Type and dimensions of points2 must match with points1.
//
///////////////////////////////////////////////////////////////////////////

#include "tupleTest.hpp"

namespace vision{

    ////////////////////////////
    // Error checking for inputs
    ////////////////////////////
    void validateInputs(int nlhs, int nrhs, const mxArray* prhs[]){
        const int maxlhs(1), minrhs(5), maxrhs(5);
        mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);

        const bool isValidpt1      = (mxIsDouble(prhs[0]) || mxIsSingle(prhs[0]));
        const bool isValidpt2      = (mxIsDouble(prhs[1]) || mxIsSingle(prhs[1]));
        const bool isValidInds     = mxIsUint32(prhs[2]);
        const bool isValidScale    = (mxIsDouble(prhs[3]) || mxIsSingle(prhs[3]));
        const bool isvalidnumIters = mxIsUint32(prhs[4]);

        // points1, points2 and scale must be single or double
        if (! (isValidpt1 && isValidpt2 && isValidInds && isValidScale && isvalidnumIters) ) {
            throw fl::except::MakeException(vision::visionlib::unsupportedDataType());
        }

        // points1 and points2 must be of same class
        if(mxGetClassID(prhs[0]) != mxGetClassID(prhs[1])){
            throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
        }

        // points1 and points2 must be of same dimension. scale and numIters must be scalar
        if (! (mxGetNumberOfDimensions(prhs[0]) == 2 && mxIsSameSize(prhs[0], prhs[1])
            && mxIsScalar(prhs[3]) && mxIsScalar(prhs[4])) ) {
            throw fl::except::MakeException(vision::visionlib::invalidSizeOfInput());
        }

        // Atleast 3 points are needed for tuple test
        if (mxGetM(prhs[0]) < 3) {
            throw fl::except::MakeException(vision::visionlib::invalidInput());
        }

        // scale should be in range of (0,1) and numIters must be >0.
        if (mxGetScalar(prhs[3]) <= 0 || mxGetScalar(prhs[3]) >= 1 || mxGetScalar(prhs[4]) <= 0) {
            throw fl::except::MakeException(vision::visionlib::invalidInput());
        }

        // randInds must be a vector of size numIters*3
        if ((mxGetM(prhs[2]) !=1) || (mxGetN(prhs[2]) != mxGetScalar(prhs[4])*3)) {
            throw fl::except::MakeException(vision::visionlib::invalidInput());
        }
    }

/**
* @mwBuiltinFunction       vision.internal.tupleTestImpl
* @mwDoesNotCallMATLAB     true
* @mwDoesNotRaiseWarnings  true
*/
    BUILTIN_IMPLEMENTATION void tupleTest(int nlhs, matrix::unique_mxarray_ptr plhs[], int nrhs, const mxArray* prhs[]) {
        // Validate inputs
        validateInputs(nlhs, nrhs, prhs);

        const uint32_T numPoints = static_cast<const uint32_T>(mxGetM(prhs[0]));
        const uint32_T pointsDim = static_cast<const uint32_T>(mxGetN(prhs[0]));
        const uint32_T* randInds = static_cast<const uint32_T*>(mxGetData(prhs[2]));
        const uint32_T numIters  = static_cast<const uint32_T>(mxGetScalar(prhs[4]));

        plhs[0] = matrix::create(numPoints, 1, mxLOGICAL_CLASS, mxREAL);
        boolean_T *indices = static_cast<boolean_T*>(mxGetData(plhs[0].get()));
        // Find matcheed point indices
        if (mxIsDouble(prhs[0])) {
            const double* points1 = static_cast<const double*>(mxGetData(prhs[0]));
            const double* points2 = static_cast<const double*>(mxGetData(prhs[1]));
            double scale          = mxGetScalar(prhs[3]);
            findMatches<double>(indices, points1, points2, randInds, numPoints, pointsDim, scale, numIters);
        }
        else{
            const float* points1 = static_cast<const float*>(mxGetData(prhs[0]));
            const float* points2 = static_cast<const float*>(mxGetData(prhs[1]));
            float scale          = static_cast<float>(mxGetScalar(prhs[3]));
            findMatches<float>(indices, points1, points2, randInds, numPoints, pointsDim, scale, numIters);
        }
    }
}
