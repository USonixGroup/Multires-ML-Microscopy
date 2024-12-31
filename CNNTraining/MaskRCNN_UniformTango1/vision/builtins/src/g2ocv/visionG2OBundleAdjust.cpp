/*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
This file contains the built-in function to perform bundle adjustment with g2o framework

The MATLAB API is:
- Full bundle Adjustment:
[xyzRefinedPoints, refinedPoses, reprojectionErrors] = visionG2OBundleAdjust(xyzPoints, measurements,...
	cameraMatrices, quaternionBases, visibility, intrinsics, options, fixedCameraIndex);

Inputs:
- xyzPoints:	   3 x M matrix,		double
- measurements:	   2 x N matrix,		double
- cameraMatrices:  6 x V matrix,		double
- quaternionBases: 4 x V matrix,		double
- visibility:      M x V sparse matrix, double
- intrinsics:
  - focalLength: 1 x 2 focal length, double
  - principalPoint: 1 x 2 principal point location in pixel units, double
  - radialDistortion: empty or 1 x 2 or 1 x 3 radial coefficients, double
  - tangentialDistortion: empty or 1 x 2 tangential coefficients, double
  - skew: empty or skew scalar, double
- options:
  - verbose: verbose mode, bool
  - absTol: absolute tolerance, double
  - relTol: relative tolerance, double
  - numIter: maximum iteration number, int
  (optional)
- fixedCameraIndices: a vector to specify the index (1-based) of the fixed camera, normally the first one, double
					  If this parameter is not provided, then all cameras are not fixed.

Outputs:
- xyzRefinedPoints: M x 3 matrix, double
- refinedPoses: 12 x V matrix, double
- reprojectionErrors: 2 x N matrix, double

References:
[1] Grisetti, G., Kümmerle, R., Strasdat, H., & Konolige, K. (2011). g2o: A general framework for (hyper) graph
optimization. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), Shanghai, China (pp. 9-13).

*////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include <stdint.h>
#include <cstdlib>
#include <string>
#include <cmath> //sqrt

#include "g2ocv/libmwg2ocv_util.hpp"
#include "g2ocv/visionG2OBundleAdjustTypes.hpp"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"

#include "g2o/solve/g2oSolve.h"
#include <matrix/matrix_developer_api.hpp>

#include <fl/except/MsgIDException.hpp>
#include <resources/vision/ocvShared.hpp>
#include <i18n/MessageCatalog.hpp>
#include <i18n/ustring_conversions.hpp>
#include "resources/vision/sfm.hpp"

// Unicode streams for output to the command window
#include "services/io/unicode_stream.hpp"

namespace {
	enum InputArgs {
		XYZPOINTS,
		MEASUREMENTS,
		CAMERAPOSES,
		QUATERNIONBASES,
		VISIBILITY,
		CAMERAPARAMS,
		OPTPARAMS,
		FIXEDCAMERAINDICES
	};

	void checkBAVariableInputs(int nlhs,
		matrix::unique_mxarray_ptr plhs[],
		int nrhs,
		const mxArray* prhs[]) {

		const int maxlhs(3), minrhs(8), maxrhs(8);
		mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);
		// Validate xyzPoints
		if (mxGetNumberOfDimensions(prhs[XYZPOINTS]) != 2 || mxGetM(prhs[XYZPOINTS]) != 3 ||
			!mxIsDouble(prhs[XYZPOINTS])) {
			throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
		}

		// Validate measurements
		if (mxGetNumberOfDimensions(prhs[MEASUREMENTS]) != 2 || mxGetM(prhs[MEASUREMENTS]) != 2 ||
			!mxIsDouble(prhs[MEASUREMENTS])) {
			throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
		}

		// Validate cameraPoses
		if (mxGetNumberOfDimensions(prhs[CAMERAPOSES]) != 2 || mxGetM(prhs[CAMERAPOSES]) != 6 ||
			!mxIsDouble(prhs[CAMERAPOSES])) {
			throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
		}

		// Validate quaternionBases
		if (mxGetNumberOfDimensions(prhs[QUATERNIONBASES]) != 2 || mxGetM(prhs[QUATERNIONBASES]) != 4 ||
			!mxIsDouble(prhs[QUATERNIONBASES]) ||
			mxGetN(prhs[QUATERNIONBASES]) != mxGetN(prhs[CAMERAPOSES])) {
			throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
		}

		// Validate visibility
		if (mxGetNumberOfDimensions(prhs[VISIBILITY]) != 2 ||
			mxGetM(prhs[VISIBILITY]) != mxGetN(prhs[XYZPOINTS]) || !mxIsDouble(prhs[VISIBILITY]) ||
			mxGetN(prhs[VISIBILITY]) != mxGetN(prhs[QUATERNIONBASES]) ||
			!mxIsSparse(prhs[VISIBILITY])) {
			throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
		}

		// Validate camera parameter structure
		if (mxGetClassID(prhs[CAMERAPARAMS]) != mxSTRUCT_CLASS) {
			throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
		}

		int nFields = mxGetNumberOfFields(prhs[CAMERAPARAMS]);
		if (nFields != 5) {
			throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
		}

		mxArray* pm;
		size_t nStructElems = mxGetNumberOfElements(prhs[CAMERAPARAMS]);
		for (size_t n = 0; n < nStructElems; ++n) {

			// Validate focal length
			pm = mxGetField(prhs[CAMERAPARAMS], n, "focalLength");
			if (pm) {
				if (mxGetNumberOfElements(pm) != 2 || !mxIsDouble(pm)) {
					throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
				}
			}
			else {
				throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
			}

			// Validate principal point
			pm = mxGetField(prhs[CAMERAPARAMS], n, "principalPoint");
			if (pm) {
				if (mxGetNumberOfElements(pm) != 2 || !mxIsDouble(pm)) {
					throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
				}
			}
			else {
				throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
			}

			// Validate skew
			pm = mxGetField(prhs[CAMERAPARAMS], n, "skew");
			if (pm) {
				if (!mxIsScalar(pm) || !mxIsDouble(pm)) {
					throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
				}
			}
			else {
				throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
			}

			// Validate radialDistortion
			int fieldNum = mxGetFieldNumber(prhs[CAMERAPARAMS], "radialDistortion");
			if (fieldNum < 0) {
				throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
			}

			pm = mxGetFieldByNumber(prhs[CAMERAPARAMS], n, fieldNum);

			if (pm) {
				size_t num = mxGetNumberOfElements(pm);
				if ((num != 2 && num != 3 && num != 0) || !mxIsDouble(pm)) {
					throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
				}
			}

			// Validate tangentialDistortion
			fieldNum = mxGetFieldNumber(prhs[CAMERAPARAMS], "tangentialDistortion");
			if (fieldNum < 0) {
				throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
			}

			pm = mxGetFieldByNumber(prhs[CAMERAPARAMS], n, fieldNum);

			if (pm) {
				size_t num = mxGetNumberOfElements(pm);
				if ((num != 2 && num != 0) || !mxIsDouble(pm)) {
					throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
				}
			}

			// Validate optimization parameters
			pm = mxGetField(prhs[OPTPARAMS], 0, "absoluteTolerance");
			if (!mxIsFullScalarDouble(pm)) {
				throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
			}

			pm = mxGetField(prhs[OPTPARAMS], 0, "relativeTolerance");
			if (!mxIsFullScalarDouble(pm)) {
				throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
			}

			pm = mxGetField(prhs[OPTPARAMS], 0, "maxIterations");
			if (!mxIsFullScalarDouble(pm)) {
				throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
			}

			pm = mxGetField(prhs[OPTPARAMS], 0, "verbose");
			if (!mxIsLogicalScalar(pm)) {
				throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
			}

			// Validate fixedCameraIndices
			if (mxGetNumberOfDimensions(prhs[FIXEDCAMERAINDICES]) != 2 ||
				!mxIsDouble(prhs[FIXEDCAMERAINDICES]) ||
				(mxGetM(prhs[FIXEDCAMERAINDICES]) > 1 && mxGetN(prhs[FIXEDCAMERAINDICES]) > 1)) {
				throw fl::except::MakeException(vision::ocvShared::invalidInputClass());
			}
		}
	}
} // end namespace 

/**
 * @mwBuiltinFunction visionG2OBundleAdjust
 * @mwDoesNotCallMATLAB true
 * @mwDoesNotRaiseWarnings true
 * @mwNargin  8
 * @mwNargout 3
 * @mwToolboxLocation vision/vision
 **/
BUILTIN_IMPLEMENTATION void visionG2OBundleAdjust(int nlhs, matrix::unique_mxarray_ptr plhs[], int nrhs, const mxArray* prhs[])
{
	checkBAVariableInputs(nlhs, plhs, nrhs, prhs);

	// Inputs
	const size_t      numPoints = mxGetN(prhs[XYZPOINTS]);
	const double*     xyzPoints = matrix::getTypedData<double>(prhs[XYZPOINTS]);
	const double*     measurements = matrix::getTypedData<double>(prhs[MEASUREMENTS]);
	const size_t      numMeasures = mxGetN(prhs[MEASUREMENTS]);
	const size_t      numViews = mxGetN(prhs[CAMERAPOSES]);
	double*           cameraPoses = matrix::getTypedData<double>(prhs[CAMERAPOSES]);
	double*           quaternionBases = matrix::getTypedData<double>(prhs[QUATERNIONBASES]);
	const bool        bSingleCamera = mxIsScalar(prhs[CAMERAPARAMS]);
	const mwIndex*    irs = mxGetIr(prhs[VISIBILITY]);
	const mwIndex*    jcs = mxGetJc(prhs[VISIBILITY]);

	// Optimization parameters
	const bool    verbose = mxGetScalar(mxGetField(prhs[OPTPARAMS], 0, "verbose"));
	const double  absTol = mxGetScalar(mxGetField(prhs[OPTPARAMS], 0, "absoluteTolerance"));
	const double  relTol = mxGetScalar(mxGetField(prhs[OPTPARAMS], 0, "relativeTolerance"));
	const size_t  numIter = static_cast<size_t>(mxGetScalar(mxGetField(prhs[OPTPARAMS], 0, "maxIterations")));

	double* fixedCameraIndex = matrix::getTypedData<double>(prhs[FIXEDCAMERAINDICES]);
	size_t  numCameraIndices = mxGetNumberOfElements(prhs[FIXEDCAMERAINDICES]);

	g2o::SparseOptimizer optimizer;
	optimizer.setVerbose(false); // Disable verbose output of g2o 

	// Assign solver
	auto solver_ptr = g2o::make_unique<g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>>();

	// Assign optimization algorithm
	auto algorithm_ptr = new g2o::OptimizationAlgorithmLevenberg(
		g2o::make_unique<g2o::BlockSolver_6_3>(std::move(solver_ptr))
	);
	optimizer.setAlgorithm(algorithm_ptr);

	// Assign camera positions and camera parameters
	const size_t firstCameraIndex = 0;
	double*  focal = matrix::getTypedData<double>(mxGetField(prhs[CAMERAPARAMS], firstCameraIndex, "focalLength"));
	double*  center = matrix::getTypedData<double>(mxGetField(prhs[CAMERAPARAMS], firstCameraIndex, "principalPoint"));
	double   skew = mxGetScalar(mxGetField(prhs[CAMERAPARAMS], firstCameraIndex, "skew"));
	double*  kr = nullptr;
	double*  kt = nullptr;
	size_t   numRadialCoefs = 0;
	mxArray* pm = nullptr;

	pm = mxGetField(prhs[CAMERAPARAMS], firstCameraIndex, "radialDistortion");
	if (pm) {
		kr = matrix::getTypedData<double>(pm);
		numRadialCoefs = mxGetNumberOfElements(mxGetField(prhs[CAMERAPARAMS], firstCameraIndex, "radialDistortion"));
	}
	pm = mxGetField(prhs[CAMERAPARAMS], firstCameraIndex, "tangentialDistortion");
	if (pm) {
		kt = matrix::getTypedData<double>(pm);
	}

	for (size_t j = 0; j < numViews; ++j) {

		if (!bSingleCamera && j > 0) {
			focal = matrix::getTypedData<double>(mxGetField(prhs[CAMERAPARAMS], j, "focalLength"));
			center = matrix::getTypedData<double>(mxGetField(prhs[CAMERAPARAMS], j, "principalPoint"));
			skew = mxGetScalar(mxGetField(prhs[CAMERAPARAMS], j, "skew"));

			pm = mxGetField(prhs[CAMERAPARAMS], j, "radialDistortion");
			if (pm) {
				kr = matrix::getTypedData<double>(pm);
				numRadialCoefs = mxGetNumberOfElements(mxGetField(prhs[CAMERAPARAMS], j, "radialDistortion"));
			}
			pm = mxGetField(prhs[CAMERAPARAMS], j, "tangentialDistortion");
			if (pm) {
				kt = matrix::getTypedData<double>(pm);
			}
		}
		
		auto v_se3 = new vision::bundleAdjust::VertexCamera(focal, center, skew, numRadialCoefs, kr, kt);

		Eigen::Vector3d trans(cameraPoses[j * 6 + 3], cameraPoses[j * 6 + 4], cameraPoses[j * 6 + 5]);
		Eigen::Quaterniond q(quaternionBases[j * 4], quaternionBases[j * 4 + 1],
			quaternionBases[j * 4 + 2], quaternionBases[j * 4 + 3]);
		g2o::SE3Quat pose(q, trans);

		v_se3->setId(static_cast<int>(j));
		for (size_t jj = 0; jj < numCameraIndices; ++jj) {
			if (fixedCameraIndex[jj] == j + 1) {
				/*Note: G2O does not allowed all camera fixed */
				v_se3->setFixed(true); // fix the camera based on fixedCameraIndices
				break;
			}
		}

		v_se3->setEstimate(pose);
		optimizer.addVertex(v_se3);
	} // end of camera view loop

	// Assign xyzPoints vertex starting from numViews
	size_t point_id = numViews + 1;;
	for (size_t i = 0; i < numPoints; ++i) {
		auto v_p = new g2o::VertexSBAPointXYZ();
		v_p->setId(static_cast<int>(point_id));
		v_p->setMarginalized(true);
		v_p->setEstimate(Eigen::Vector3d(xyzPoints[3 * i], xyzPoints[3 * i + 1], xyzPoints[3 * i + 2]));
		optimizer.addVertex(v_p);
		point_id += 1;
	}

	double initialChi2 = 0; // Squared reprojection error
	double initialChi = 0; // Reprojection error

	// Assign edges
	for (size_t j = 0; j < numViews; ++j) {
		size_t k1 = jcs[j];
		size_t k2 = jcs[j + 1];
		for (size_t k = k1; k < k2; ++k) {
			size_t i = irs[k];
			point_id = i + numViews + 1;
			Eigen::Vector2d z(measurements[2 * k], measurements[2 * k + 1]);
			auto e = new vision::bundleAdjust::EdgeProjection();

			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(static_cast<int>(point_id)))); // xyzPoints
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(static_cast<int>(j)))); //camera view
			e->setMeasurement(z);
			e->information() = Eigen::Matrix2d::Identity();
			e->setParameterId(0, 0);
			optimizer.addEdge(e);

			// Calculate initial error 
			Eigen::Vector2d err = e->returnError();
			initialChi2 += err.squaredNorm();
			initialChi += err.norm();
		} // end of xyzPoint loop
	} // end of camera view loop

	optimizer.initializeOptimization();

	double lastChi2 = initialChi2;
	double curChi2 = 0.0;

	// Verbose output
	fl::ustring s;
	services::io::uout myout;

	for (size_t ii = 0; ii < numIter; ++ii) {
		optimizer.optimize(1);
		curChi2 = optimizer.activeChi2();

		// Modified Verbose print format to MATLAB built in format     
		if (verbose) {
			s = fl::i18n::MessageCatalog::get_message(vision::sfm::sbaIteration(std::to_string(ii + 1)));
			myout << s;

			s = fl::i18n::MessageCatalog::get_message(vision::sfm::sbaMeanSquareError(std::to_string(lastChi2 / numMeasures)));
			myout << s << std::endl;
		}

		if (curChi2 / numMeasures < absTol) {
			if (verbose) {
				s = fl::i18n::MessageCatalog::get_message(vision::sfm::sbaStopCondSmallAbsFunVal());
				myout << s << std::endl;
			}
			break;
		}

		if ((lastChi2 + curChi2 - 2 * std::sqrt(lastChi2*curChi2)) < relTol*lastChi2) {  // ((sqrt(e1) - sqrt(e2))^2 < relTol*e1
			if (verbose) {
				s = fl::i18n::MessageCatalog::get_message(vision::sfm::sbaStopCondSmallRelChangeOfFunVal());
				myout << s << std::endl;
			}
			break;
		}

		if (ii == (numIter - 1) && verbose) {
			s = fl::i18n::MessageCatalog::get_message(vision::sfm::sbaStopCondMaxIteration());
			myout << s << std::endl;
		}
		lastChi2 = curChi2;
	}

	if (verbose) {
		s = fl::i18n::MessageCatalog::get_message(vision::sfm::sbaReportInitialError(std::to_string(initialChi / numMeasures)));
		myout << s << std::endl;

		// Calculate final mean reprojection error
		double finalChi = 0.0;
		for (auto &it : optimizer.activeEdges()) {
			finalChi += std::sqrt((it)->chi2());
		}

		s = fl::i18n::MessageCatalog::get_message(vision::sfm::sbaReportFinalError(std::to_string(finalChi / numMeasures)));
		myout << s << std::endl;
	}

	// Extract refindedPoints, refinedPoses, reprojectionErrors
	plhs[0] = matrix::create(numPoints, 3, mxDOUBLE_CLASS, mxREAL);
	plhs[1] = matrix::create(12, numViews, mxDOUBLE_CLASS, mxREAL);
	plhs[2] = matrix::create(2, numMeasures, mxDOUBLE_CLASS, mxREAL);

	// Get raw pointers to data for use in computations
	double* xyzRefinedPoints = matrix::getTypedData<double>(plhs[0].get());
	double* cameraPoseTforms = matrix::getTypedData<double>(plhs[1].get());
	double* reprojErrors = matrix::getTypedData<double>(plhs[2].get());

	for (size_t j = 0; j < numViews; ++j) {

		auto vj = dynamic_cast<vision::bundleAdjust::VertexCamera*>(optimizer.vertex(static_cast<int>(j))); // camera view
		Eigen::Isometry3d p = vj->estimate();

		// Convert from Eigen matrix to rotation and translation 
		cameraPoseTforms[12 * j] = p(0, 0);
		cameraPoseTforms[12 * j + 1] = p(1, 0);
		cameraPoseTforms[12 * j + 2] = p(2, 0);
		cameraPoseTforms[12 * j + 3] = p(0, 1);
		cameraPoseTforms[12 * j + 4] = p(1, 1);
		cameraPoseTforms[12 * j + 5] = p(2, 1);
		cameraPoseTforms[12 * j + 6] = p(0, 2);
		cameraPoseTforms[12 * j + 7] = p(1, 2);
		cameraPoseTforms[12 * j + 8] = p(2, 2);

		cameraPoseTforms[12 * j + 9] = -(p(0, 3)*p(0, 0) + p(1, 3)*p(1, 0) + p(2, 3)*p(2, 0));
		cameraPoseTforms[12 * j + 10] = -(p(0, 3)*p(0, 1) + p(1, 3)*p(1, 1) + p(2, 3)*p(2, 1));
		cameraPoseTforms[12 * j + 11] = -(p(0, 3)*p(0, 2) + p(1, 3)*p(1, 2) + p(2, 3)*p(2, 2));

		g2o::SE3Quat pose = vj->estimate();

		size_t k1 = jcs[j];
		size_t k2 = jcs[j + 1];
		for (size_t k = k1; k < k2; ++k) {
			size_t i = irs[k];

			g2o::VertexSBAPointXYZ* vi;
			vi = dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(static_cast<int>(i + numViews + 1))); // xyzPoint       

			Eigen::Vector3d xyz = vi->estimate();

			xyzRefinedPoints[i] = xyz(0);
			xyzRefinedPoints[i + numPoints] = xyz(1);
			xyzRefinedPoints[i + numPoints * 2] = xyz(2);

			Eigen::Vector2d reprojection = vj->cam_map(pose.map(xyz));

			reprojErrors[2 * k] = reprojection(0) - measurements[2 * k];
			reprojErrors[2 * k + 1] = reprojection(1) - measurements[2 * k + 1];

		} // end of xyzPoint loop
	} // end of camera view loop
} // end of BUILTIN_IMPLEMENTATION