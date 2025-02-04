/* Copyright 2019-2022 The MathWorks, Inc. */

/**
 * @file
 * Builtin for pose graph optimization
 */

#include <vector>

#include <mex.h>
#include <mcos.h>
#include <matrix/matrix_developer_api.hpp>
#include <matrix/unique_mxarray_ptr.hpp>
#include <mfl_scalar/constants.hpp>
#include <fl/except/MsgIDException.hpp>
#include <vision/calibration/quaternion.hpp>

#include "resources/vision/ocvShared.hpp"
#include "g2ocv/libmwg2ocv_util.hpp"
#include "g2ocv/PoseGraphOptimizer.hpp"

namespace vision {
	/**
	* @brief Validate inputs and check if the pose graph is a similarity pose graph
	*/
	bool validatePoseGraph(int nlhs, matrix::unique_mxarray_ptr plhs[], int nrhs, const mxArray* prhs[]) {
		int maxlhs(3), minrhs(8), maxrhs(9);
		mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);

		if (!mxIsClass(prhs[0], "rigidtform3d") || !(mxIsClass(prhs[1], "rigidtform3d") ||
			!(mxIsCell(prhs[2]))) || !(mxIsDouble2dMatrix(prhs[3]))) {
			throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
		}

		if (!mxIsFullScalarDouble(prhs[4]) || !mxIsFullScalarDouble(prhs[5]) || !mxIsLogicalScalar(prhs[6]) || !mxIsFullScalarDouble(prhs[7])) {
			throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
		}

		bool isSimilarityPoseGraph = nrhs > 8;

		if (isSimilarityPoseGraph) {
			if (!mxIsDouble2dMatrix(prhs[8])) {
				throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
			}
		}

		return isSimilarityPoseGraph;
	}

	/**
	* @brief Convert mxArray full information matrix to it's compact form for rigid pose graph
	*
	* @param[in] infoMatFull mxArray pointing to full 36-element information matrix stored in column-major format
	*      in single or double-precision
	* @param[out] infoMatCompact Compact 21-element information matrix in double-precision
	*
	*/
	inline std::vector<double> compactifyRigidInformationMatrix(const mxArray* mxInfoMat) {
		std::vector<double> infoMatCompact;

		if (mxIsDouble(mxInfoMat)) {
			const double *infoMatFull = matrix::getTypedData<double>(mxInfoMat);

			infoMatCompact = {
				infoMatFull[0], infoMatFull[6], infoMatFull[12], infoMatFull[18], infoMatFull[24], infoMatFull[30],
				infoMatFull[7], infoMatFull[13], infoMatFull[19], infoMatFull[25], infoMatFull[31],
				infoMatFull[14], infoMatFull[20], infoMatFull[26], infoMatFull[32],
				infoMatFull[21], infoMatFull[27], infoMatFull[33],
				infoMatFull[28], infoMatFull[34],
				infoMatFull[35] };
		}
		else {
			const float *infoMatFull = matrix::getTypedData<float>(mxInfoMat);

			infoMatCompact = {
				(double)infoMatFull[0], (double)infoMatFull[6], (double)infoMatFull[12], (double)infoMatFull[18], (double)infoMatFull[24], (double)infoMatFull[30],
				(double)infoMatFull[7], (double)infoMatFull[13], (double)infoMatFull[19], (double)infoMatFull[25], (double)infoMatFull[31],
				(double)infoMatFull[14], (double)infoMatFull[20], (double)infoMatFull[26], (double)infoMatFull[32],
				(double)infoMatFull[21], (double)infoMatFull[27], (double)infoMatFull[33],
				(double)infoMatFull[28], (double)infoMatFull[34],
				(double)infoMatFull[35] };
		}

		infoMatCompact.reserve(infoMatCompact.size());

		return infoMatCompact;
	}
	
	/**
	* @brief Convert mxArray full information matrix to it's compact form for similarity pose graph
	*
	* @param[in] infoMatFull mxArray pointing to full 49-element information matrix stored in column-major format
	*      in single or double-precision
	* @param[out] infoMatCompact Compact 28-element information matrix in double-precision
	*
	*/
	inline std::vector<double> compactifySimilarityInformationMatrix(const mxArray* mxInfoMat) {
		std::vector<double> infoMatCompact;

		if (mxIsDouble(mxInfoMat)) {
			const double *infoMatFull = matrix::getTypedData<double>(mxInfoMat);

			infoMatCompact = {
				infoMatFull[0],  infoMatFull[7],  infoMatFull[14], infoMatFull[21], infoMatFull[28], infoMatFull[35], infoMatFull[42],
				infoMatFull[8],  infoMatFull[15], infoMatFull[22], infoMatFull[29], infoMatFull[36], infoMatFull[43],
				infoMatFull[16], infoMatFull[23], infoMatFull[30], infoMatFull[37], infoMatFull[44],
				infoMatFull[24], infoMatFull[31], infoMatFull[38], infoMatFull[45],
				infoMatFull[32], infoMatFull[39], infoMatFull[46],
				infoMatFull[40], infoMatFull[47],
				infoMatFull[48] };
		}
		else {
			const float *infoMatFull = matrix::getTypedData<float>(mxInfoMat);

			infoMatCompact = {
				(double)infoMatFull[0],  (double)infoMatFull[7],  (double)infoMatFull[14], (double)infoMatFull[21], (double)infoMatFull[28], (double)infoMatFull[35], (double)infoMatFull[42],
				(double)infoMatFull[8],  (double)infoMatFull[15], (double)infoMatFull[22], (double)infoMatFull[29], (double)infoMatFull[36], (double)infoMatFull[43],
				(double)infoMatFull[16], (double)infoMatFull[23], (double)infoMatFull[30], (double)infoMatFull[37], (double)infoMatFull[44],
				(double)infoMatFull[24], (double)infoMatFull[31], (double)infoMatFull[38], (double)infoMatFull[45],
				(double)infoMatFull[32], (double)infoMatFull[39], (double)infoMatFull[46],
				(double)infoMatFull[40], (double)infoMatFull[47],
				(double)infoMatFull[48] };
		}

		infoMatCompact.reserve(infoMatCompact.size());

		return infoMatCompact;
	}

    /**
     * @brief Convert mxArray transformation matrix to pose quaternion
     *
     * @param[in] mxTform mxArray pointing to a single or double precision transformation matrix
     * @param[out] pose vector of double-precision quaternion pose
     */
    inline void mxTransformationMatrixToPoseQuat(const mxArray* mxTform, double* pose) {
        
        if (mxIsDouble(mxTform)) {
            // Extract double-precision transformation matrix
            double *tform = matrix::getTypedData<double>(mxTform);
            
            // Convert to quaternion pose
            vision::tform2quatpose(tform, pose);
        } else {
            // Extract single-precision transformation matrix
            float *float_tform = matrix::getTypedData<float>(mxTform);
            
            // Convert to double
            std::vector<double> double_tform(16, 0);
            double_tform.reserve(16);
            
            for (size_t i = 0; i < double_tform.size(); i++) {
                double_tform[i] = (double) float_tform[i];
            }
            
            // Convert to quaternion pose
            vision::tform2quatpose(&double_tform[0], pose);
        }
    }
} // end namespace vision

/**
 * @brief Gateway function to optimize a pose graph
 *
 * [optimPoses, solInfo, poseScale] = visionOptimizePoses(absPoses, relPoses, infoMats, edgeIds, maxIter, funcTol, verboseFlag, maxTime, edgeScales)
 * optimizes a pose graph with absolute node poses specified by absPoses and 
 * relative pose constraints specified by relPoses and associated edge IDs 
 * edgeIds. absPoses and relPoses must be an array of rigidtform3d objects. 
 * edgeIds is an M-by-2 array of edge ids. The optional input edgeScales is 
 * an M-by-1 array of edge scales. When edgeScales is specified, the pose
 * graph optimization is performed over 3-D similarity transformations. 
 * Otherwise, it is performed over 3-D rigid transformations. The output 
 * poseScale is available only when edgeScales is specified.
 *  
 * @mwBuiltinFunction visionOptimizePoses
 * @mwDominantClass   :all:
 * @mwNargin          9
 * @mwToolboxLocation vision/vision
 */
BUILTIN_IMPLEMENTATION void visionOptimizePoses(int nlhs, matrix::unique_mxarray_ptr plhs[], int nrhs, const mxArray* prhs[]) {
    
	const bool isSimilarityPoseGraph = vision::validatePoseGraph(nlhs, plhs, nrhs, prhs);
    
    const mxArray *mxAbsPoses = prhs[0];
    const mxArray *mxRelPoses = prhs[1];
    const mxArray *mxInfoMats = prhs[2];
    const mxArray *mxEdgeIds  = prhs[3];
    
    const size_t numNodes = mxGetNumberOfElements(prhs[0]);
    const size_t numEdges = mxGetNumberOfElements(prhs[1]);
    
    const double maxIter   = mxGetScalar(prhs[4]);
    const double funcTol   = mxGetScalar(prhs[5]);
    const bool verboseFlag = mxIsLogicalScalarTrue(prhs[6]);
    const double maxTime   = mxGetScalar(prhs[7]);
    
    const mxArray *mxScales = isSimilarityPoseGraph ? prhs[8] : nullptr;

    const size_t   poseDim = isSimilarityPoseGraph ? 8 : 7;
    const size_t   edgeDim = poseDim + 2;
    constexpr size_t infoMatDim = 21;
    constexpr char rigidtform3dClassName[] = "rigidtform3d";
    constexpr char transformationMatrixPropertyName[] = "A";
    
    double	 blockSolverType = isSimilarityPoseGraph ? 2 : 1; // SIM3 solver : SE3 solver
    
    std::vector<std::vector<double>> Nodes, Edges, InfoMats, OptimNodes;
    
    Nodes.reserve(numNodes);
    OptimNodes.reserve(numNodes);
    Edges.reserve(numEdges);
    InfoMats.reserve(numEdges);
    
    // TODO Possible performance improvement by directly constructing g2o data
    //      structures like g2o::VertexSE3.
    for (size_t n = 0; n < numNodes; n++) {
        // Allocate vector of length 7 or 8 to store
        // [x, y, z, q1, q2, q3, q4] or
        // [x, y, z, q1, q2, q3, q4, scale]
        std::vector<double> pose(poseDim);
        pose.reserve(poseDim);
        
        // Extract transformation matrix from rigidtform3d object and convert to
        // quaternion pose
        mxArray* mxAbsPose_n = mxGetPropertyShared(mxAbsPoses, n, transformationMatrixPropertyName);
        
        vision::mxTransformationMatrixToPoseQuat(mxAbsPose_n, &pose[0]);
        
        // Scales of view poses are set to 1
        if (isSimilarityPoseGraph){
            pose.back() = 1.0;
        }

        Nodes.push_back(pose);
        
        mxDestroyArray(mxAbsPose_n);
    }
    
    double* edgeIds     = matrix::getTypedData<double>(mxEdgeIds);
	double* edgeScales = isSimilarityPoseGraph ? matrix::getTypedData<double>(mxScales) : nullptr;
    
    for (size_t e = 0; e < numEdges; e++) {
        // Allocate vector of length 9 or 10 to store
        // [edgeId1, edgeId2, x, y, z, q1, q2, q3, q4] or
        // [edgeId1, edgeId2, x, y, z, q1, q2, q3, q4, scale]
        std::vector<double> edge(edgeDim);
        edge.reserve(edgeDim);
        
        // Add edge ids (convert to 0-based indices)
        edge[0] = edgeIds[e] - 1;
        edge[1] = edgeIds[e + numEdges] -1;
        
        // Extract transformation matrix from rigidtform3d object and convert to
        // quaternion pose
        mxArray* mxRelPose_n = mxGetPropertyShared(mxRelPoses, e, transformationMatrixPropertyName);

        vision::mxTransformationMatrixToPoseQuat(mxRelPose_n, &edge[2]);
        
        if (isSimilarityPoseGraph){
            // Convert [x/scale, y/scale, z/scale] to [x, y, z] to follow 
            // the format in shared backend
            edge[2] *= edgeScales[e];
            edge[3] *= edgeScales[e];
            edge[4] *= edgeScales[e];
            edge[edgeDim-1] = edgeScales[e];
        }
        
        Edges.push_back(edge);
        
        std::vector<double> infoMatCompact = isSimilarityPoseGraph ? 
			vision::compactifySimilarityInformationMatrix(mxGetCell(mxInfoMats, e)) :
			vision::compactifyRigidInformationMatrix(mxGetCell(mxInfoMats, e));

        InfoMats.push_back(infoMatCompact);
        
        mxDestroyArray(mxRelPose_n);
    }
    
    // Create a pose graph optimizer
    vision::PoseGraphOptimizer optimizer(blockSolverType, maxIter, funcTol, verboseFlag, maxTime);
    
    // Optimize pose graph
    optimizer.optimize(Nodes, Edges, InfoMats, OptimNodes);
    
    // Note: PKG_ADOPT is a tag added as part of the MATLAB packages transition project. 
    // It will eventually be removed automatically. See the following 
    // for more information:
    // https://confluence.mathworks.com/pages/viewpage.action?pageId=467761996
    plhs[0] = matrix::from_matlab(omCreateMCOSMatrix(numNodes, 1, rigidtform3dClassName)); // PKG_ADOPT
    
    for (size_t n = 0; n < numNodes; n++) {
        mxArray* mxTform = mxCreateUninitNumericMatrix(4, 4, mxDOUBLE_CLASS, mxREAL);
        double * tform = matrix::getTypedData<double>(mxTform);
        vision::quatpose2tform(&OptimNodes[n][0], tform);

        const mxArray* inputs[] = {mxTform};
        mxArray* mxTformObj = omConstructObjectWithClient(rigidtform3dClassName, 1, inputs, COSGetPublicClient()); // PKG_ADOPT
        mcos::COSInterfacePtr cosI = omGetArrayElement(mxTformObj, 0);
        omSetArrayElement(plhs[0].get(), n, cosI);
        
        mxDestroyArray(mxTform);
        mxDestroyArray(mxTformObj);
    }
    
    if (nlhs>1) {
        double finalChi = optimizer.finalChi();
        double numIter  = optimizer.numIterations();
        double exitFlag = optimizer.exitFlag();
        
        // Package these into a MATLAB struct
        const char* fields[] = {"FinalChi", "NumIterations", "ExitFlag"};
        plhs[1] = matrix::create_struct(1, 1, 3, fields);
		mxArray *mxOptimInfo = plhs[1].get();
        
        mxSetFieldByNumber(mxOptimInfo, 0, 0, mxCreateDoubleScalar(finalChi));
        mxSetFieldByNumber(mxOptimInfo, 0, 1, mxCreateDoubleScalar(numIter));
        mxSetFieldByNumber(mxOptimInfo, 0, 2, mxCreateDoubleScalar(exitFlag));
        
        // Return optimized scales associated with view poses. These values
        // are used to update the map after similarity pose graph optimization
        // in the monocular visual SLAM workflow
        if (isSimilarityPoseGraph){
            plhs[2] = matrix::create_uninit(numNodes, 1, mxDOUBLE_CLASS, mxREAL);
            double* nodeScales = matrix::getTypedData<double>(plhs[2].get());
            
            for (size_t n = 0; n < numNodes; n++){
                nodeScales[n] = OptimNodes[n][poseDim-1];
            }
        }
    }
}
