///////////////////////////////////////////////////////////////////////////
// Copyright 2022-2023 The MathWorks, Inc.
///////////////////////////////////////////////////////////////////////////
#define _SILENCE_CXX17_NEGATORS_DEPRECATION_WARNING

#include "registerICP.hpp"

///////////////////////////////////////////////////////////////////////////
// @mwBuiltinFunction       registerICP
// @mwNargin                1
// @mwNargout               2
// @mwDoesNotCallMATLAB     true
// @mwDoesNotRaiseWarnings  true
BUILTIN_IMPLEMENTATION void registerICP(int nlhs,       mxArray* plhs[],
                                        int nrhs, const mxArray* prhs[]) {

    // Validate inputs.
    checkInputs(nlhs, nrhs, prhs);

    // Construct registration object with the input parameters.
    ICPRegistrationImpl icpRegistrationObj(prhs[0]);

    // Perform registration.
    auto regResult = icpRegistrationObj.registerImpl();

    // Send the results back to MATLAB.
    packageOutputs(regResult, nlhs, plhs);
}

//////////////////////////////////////////////////////////////////////////////
// ICPRegistrationImpl Constructor.
//////////////////////////////////////////////////////////////////////////////
ICPRegistrationImpl::ICPRegistrationImpl(const mxArray* prhs){
    
    // Parse inputs and initialize class members.
    parseInputs(prhs);

    // Configure OpenMP for multi-threading.
    #ifdef _OPENMP
        // Get the upper bound of the possible number of threads.
        int numThreads = omp_get_max_threads();
        omp_set_num_threads(numThreads);
    #endif
}


//////////////////////////////////////////////////////////////////////////////
// Helper function to perform registration based on the input method.
//////////////////////////////////////////////////////////////////////////////
o3dRegistration::RegistrationResult 
    ICPRegistrationImpl::registerImpl(){
    
    if(method=="pointToPoint") {
        return registerUsingPointToPointICP();
    } else if(method=="planeToPlane") {
        return registerUsingPlaneToPlaneICP();
    } else if(method=="pointToPlane") {
        return registerUsingPointToPlaneICP();
    } else if(method=="pointToPlaneWithColor") {
        return registerUsingPointToPlaneWithColorICP();
    } else {
        return registerUsingPlaneToPlaneWithColorICP();
    }
}

//////////////////////////////////////////////////////////////////////////////
// Helper function to call Open3D's API for point-to-point ICP.
//////////////////////////////////////////////////////////////////////////////
o3dRegistration::RegistrationResult 
    ICPRegistrationImpl::registerUsingPointToPointICP(){
    
    bool doEstimateScale = false;
    auto estimation = o3dRegistration::TransformationEstimationPointToPoint(doEstimateScale);

    return o3dRegistration::RegistrationICP(moving, fixed, metric,
                initTform, estimation, convergenceCriteria, useInlierRatio, verbose);
}

//////////////////////////////////////////////////////////////////////////////
// Helper function to call Open3D's API for point-to-plane ICP.
//////////////////////////////////////////////////////////////////////////////
o3dRegistration::RegistrationResult 
    ICPRegistrationImpl::registerUsingPointToPlaneICP(){
    
    auto estimation = o3dRegistration::TransformationEstimationPointToPlane();

    return o3dRegistration::RegistrationICP(moving, fixed, metric, 
                initTform, estimation, convergenceCriteria, useInlierRatio, verbose);
}

////////////////////////////////////////////////////////////////////////////////
// Helper function to call Open3D's API for color variant of point-to-plane ICP.
// This algorithm is inspired from "Open3D v0.17.0 cpp/open3d/pipelines/
// registration/ColoredICP.cpp". The hyperparameters have been modified after
// performing experiments on a set of chosen datasets.
////////////////////////////////////////////////////////////////////////////////
o3dRegistration::RegistrationResult 
    ICPRegistrationImpl::registerUsingPointToPlaneWithColorICP(){
    
    int prevMaxIteration = 0;
    Eigen::Matrix4d trans = initTform;
    o3dRegistration::RegistrationResult finalResult;    

    // Multi-Scale Registration (Coarser registration to finer registration).
    // The result of a coarse registration step is fed to the next finer 
    // registration step as initial transformation.
    for (int i = 0; i < 3; ++i) {
        double voxelSize = voxelSizes[i];

        auto movingDownsampled = gridAverageDownsampling(moving, voxelSize);
        auto fixedDownsampled = gridAverageDownsampling(fixed, voxelSize);

        auto estimation = o3dRegistration::TransformationEstimationForColoredICP();

        auto result = o3dRegistration::RegistrationColoredICP(*movingDownsampled, *fixedDownsampled, metric,
        trans, estimation, o3dRegistration::ICPConvergenceCriteria(useTransformationTolerance, 
        relativeTranslation, relativeRotation, iterations[i], prevMaxIteration),
        useInlierRatio, verbose);
        prevMaxIteration = result.previousMaxIteration;
        trans = result.transformation;
        finalResult = result;
    }

    return finalResult;
}

//////////////////////////////////////////////////////////////////////////////
// Helper function to call Open3D's API for plane-to-plane ICP (Generalized-ICP).
//////////////////////////////////////////////////////////////////////////////
o3dRegistration::RegistrationResult 
    ICPRegistrationImpl::registerUsingPlaneToPlaneICP(){
    
    auto estimation = o3dRegistration::TransformationEstimationForGeneralizedICP();

    return o3dRegistration::RegistrationGeneralizedICP(moving, fixed, metric,
                initTform, estimation, convergenceCriteria, useInlierRatio, verbose);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper function to call Open3D based implementation for color variant of plane-to-plane ICP (Generalized-ICP).
// This algorithm is inspired from "PCL v1.13.0 /registration/src/gicp6d.cpp ". The hyperparameters have been 
// modified after performing experiments on a set of chosen datasets. This implementation is based on Open3D's
// template for kd tree which can suffice the need of six dimenstional kd-tree.
//
//  M. Korn, M. Holzkothen, and J. Pauli. Color supported Generalized-ICP. In VISAPP, 2014.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
o3dRegistration::RegistrationResult 
    ICPRegistrationImpl::registerUsingPlaneToPlaneWithColorICP(){

    int prevMaxIteration = 0;
    auto estimation = o3dRegistration::TransformationEstimationForGeneralizedICP();

    return o3dRegistration::RegistrationGeneralizedICP(moving, fixed, metric,
    initTform, estimation, o3dRegistration::ICPConvergenceCriteria(useTransformationTolerance, 
    relativeTranslation, relativeRotation, maxIteration, prevMaxIteration, true), useInlierRatio, verbose);
}

//////////////////////////////////////////////////////////////////////////////
// Grid average downsampling and estimation of normals
//////////////////////////////////////////////////////////////////////////////
std::shared_ptr<open3d::geometry::PointCloud> gridAverageDownsampling(open3d::geometry::PointCloud ptCloud,double voxelSize)
{
    auto ptCloudDownsampled = ptCloud.VoxelDownSample(voxelSize);
    ptCloudDownsampled->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(
    voxelSize * 2.0, 30));
    return ptCloudDownsampled;
}

//////////////////////////////////////////////////////////////////////////////
// Read normals from an mxArray.
//////////////////////////////////////////////////////////////////////////////
std::vector<Eigen::Vector3d> 
    ICPRegistrationImpl::readNormals(const mxArray* mxNormals){

    std::vector<Eigen::Vector3d> normals;

    size_t count = mxGetM(mxNormals);
    size_t xOffset(0);
    size_t yOffset(xOffset + count);
    size_t zOffset(yOffset + count);
    if (mxIsSingle(mxNormals)) {
        real32_T* normalsData = static_cast<real32_T*>(mxGetData(mxNormals));
        for (size_t i = 0; i < count; i++) {
            auto normal = Eigen::Vector3d(normalsData[i + xOffset],
                                          normalsData[i + yOffset],
                                          normalsData[i + zOffset]);
            normals.push_back(normal);
        }
    } else {
        real64_T* normalsData = static_cast<real64_T*>(mxGetData(mxNormals));
        for (size_t i = 0; i < count; i++) {
            auto normal = Eigen::Vector3d(normalsData[i + xOffset],
                                          normalsData[i + yOffset],
                                          normalsData[i + zOffset]);
            normals.push_back(normal);
        }
    }

    return normals;
}

//////////////////////////////////////////////////////////////////////////////
// Read normalized colors from an mxArray.
// Input must be of type double
//////////////////////////////////////////////////////////////////////////////
std::vector<Eigen::Vector3d>
    ICPRegistrationImpl::readNormalizedColors(const mxArray* mxColors){

    std::vector<Eigen::Vector3d> colors;

    size_t count = mxGetM(mxColors);
    size_t xOffset(0);
    size_t yOffset(xOffset + count);
    size_t zOffset(yOffset + count);
    
    real64_T* colorsData = static_cast<real64_T*>(mxGetData(mxColors));
    for (size_t i = 0; i < count; i++) {
        auto color = Eigen::Vector3d(colorsData[i + xOffset],
                                     colorsData[i + yOffset],
                                     colorsData[i + zOffset]);
        colors.push_back(color);
    }
    return colors;
}

//////////////////////////////////////////////////////////////////////////////
// Read voxel sizes from an mxArray.
//////////////////////////////////////////////////////////////////////////////
std::vector<double>
    ICPRegistrationImpl::readVoxelSizes(const mxArray* mxVoxelSizes){

    std::vector<double> voxels;
    double* voxelPtr = static_cast<double*>(mxGetData(mxVoxelSizes));

    for (size_t i = 0; i < 3; i++) {
        voxels.push_back((double)voxelPtr[i]);
    }
    return voxels;
}

//////////////////////////////////////////////////////////////////////////////
// Read the max iterations distribution for multi-scale registration from an mxArray.
//////////////////////////////////////////////////////////////////////////////
std::vector<int>
    ICPRegistrationImpl::readMaxIterations(const mxArray* mxIterations){

    std::vector<int> iterations;
    double* iterationPtr = static_cast<double*>(mxGetData(mxIterations));

    for (size_t i = 0; i < 3; i++) {
        iterations.push_back((int)iterationPtr[i]);
    }
    return iterations;
}

//////////////////////////////////////////////////////////////////////////////
// Check inputs.
//////////////////////////////////////////////////////////////////////////////
static void checkInputs(int nlhs, int nrhs, const mxArray *prhs[]){

    int minrhs(1);
    int maxrhs(1);
    int maxlhs(2);
    mxCheckNumArgs(nlhs, maxlhs, nrhs, minrhs, maxrhs);
}

//////////////////////////////////////////////////////////////////////////////
// Package outputs to send back to MATLAB.
//////////////////////////////////////////////////////////////////////////////
static void packageOutputs(o3dRegistration::RegistrationResult regResult,
                           int nlhs, mxArray *plhs[]){

    // Package transformation output.
    Eigen::Matrix4d_u T = regResult.transformation;
    auto mxTransform = matrix::create(4, 4, mxDOUBLE_CLASS, mxREAL);
    double* tform = (double*) mxGetData(mxTransform.get());
    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++)
            tform[i+4*j] = T(j,i); // (j,i) instead of (i,j) to transpose
    }

    plhs[0] = matrix::to_matlab(std::move(mxTransform));
}

//////////////////////////////////////////////////////////////////////////////
// Parse inputs from the structure in mxArray.
//////////////////////////////////////////////////////////////////////////////
void ICPRegistrationImpl::parseInputs(const mxArray* prhs){

    // Parse input point clouds.
    moving = readPointCloud(mxGetField(prhs, 0, "moving"));
    fixed = readPointCloud(mxGetField(prhs, 0, "fixed"));

    // Parse input normals if present.
    bool hasNormalsA = static_cast<bool>(mxGetScalar(mxGetField(prhs, 0, "HasNormalsA")));
    if(hasNormalsA){
        moving.normals_ = readNormals(mxGetField(prhs, 0, "movingNormals"));
    }

    bool hasNormalsB = static_cast<bool>(mxGetScalar(mxGetField(prhs, 0, "HasNormalsB")));
    if(hasNormalsB){
        fixed.normals_ = readNormals(mxGetField(prhs, 0, "fixedNormals"));
    }

    // Parse inital transformation and convergence criteria.
    initTform = readInitTform(prhs);
    convergenceCriteria = readConvergenceCriteria(prhs);

    // Parse registration options.
    parseRegistrationOptions(prhs);

    if(useColor) {
        moving.colors_ = readNormalizedColors(mxGetField(prhs, 0, "movingColors"));
        fixed.colors_ = readNormalizedColors(mxGetField(prhs, 0, "fixedColors"));
        voxelSizes = readVoxelSizes(mxGetField(prhs, 0, "VoxelSizes"));
        iterations = readMaxIterations(mxGetField(prhs, 0, "MaxIterationsVector"));
    }
}

//////////////////////////////////////////////////////////////////////////////
// Read input point cloud locations from an mxArray.
//////////////////////////////////////////////////////////////////////////////
o3dGeometry::PointCloud ICPRegistrationImpl::readPointCloud(const mxArray* location){

    o3dGeometry::PointCloud ptCloud;

    size_t count = mxGetM(location);
    size_t xOffset(0);
    size_t yOffset(xOffset + count);
    size_t zOffset(yOffset + count);
    if (mxIsSingle(location)) {
        real32_T* locationPoints = static_cast<real32_T*>(mxGetData(location));
        for (size_t i = 0; i < count; i++) {
            auto point = Eigen::Vector3d(locationPoints[i + xOffset],
                                         locationPoints[i + yOffset],
                                         locationPoints[i + zOffset]);
            ptCloud.points_.push_back(point);
        }
    } else {
        real64_T* locationPoints = static_cast<real64_T*>(mxGetData(location));
        for (size_t i = 0; i < count; i++) {
            auto point = Eigen::Vector3d(locationPoints[i + xOffset],
                                         locationPoints[i + yOffset],
                                         locationPoints[i + zOffset]);
            ptCloud.points_.push_back(point);
        }
    }

    return ptCloud;
}

//////////////////////////////////////////////////////////////////////////////
// Read initial transformation from an mxArray.
//////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d ICPRegistrationImpl::readInitTform(const mxArray *prhs){

    mxArray* mxInitTform = mxGetField(prhs, 0, "InitialTransform");
    Eigen::Matrix4d initTform;
    if(mxIsSingle(mxInitTform)){
        float* tform = static_cast<float*>(mxGetData(mxInitTform));
        for (size_t i = 0; i < 4; i++) {
            for (size_t j = 0; j < 4; j++)
                initTform(j,i) = static_cast<double>(tform[i+4*j]); // (j,i) instead of (i,j) to transpose
        }
    } else {
        double* tform = static_cast<double*>(mxGetData(mxInitTform));
        for (size_t i = 0; i < 4; i++) {
            for (size_t j = 0; j < 4; j++)
                initTform(j,i) = tform[i+4*j]; // (j,i) instead of (i,j) to transpose
        }
    }

    return initTform;
}

//////////////////////////////////////////////////////////////////////////////
// Read tolerance parameters from mxArray and construct ICPConvergenceCriteria.
//////////////////////////////////////////////////////////////////////////////
o3dRegistration::ICPConvergenceCriteria
       ICPRegistrationImpl::readConvergenceCriteria(const mxArray *prhs){
    
    maxIteration = static_cast<int>(mxGetScalar(
        mxGetField(prhs, 0, "MaxIteration")));
    
    relativeRotation = static_cast<double>(mxGetScalar(
        mxGetField(prhs, 0, "RelativeRotation")));

    relativeTranslation = static_cast<double>(mxGetScalar(
        mxGetField(prhs, 0, "RelativeTranslation")));

    // This is a flag to inform ICPConvergenceCriteria present in Registration.cpp 
    // to measure convergence based on transformation (R & t) instead of default 
    // metrics like relative fitness and relative RMSE.
    useTransformationTolerance = true;

    // This would hold the previous maximum iteration utilised in the Colored
    // variants of ICP registration. Default is set to '0' as other variants
    // do not follow multi-scale ICP registation.
    int previousMaxIterations = 0;

    return o3dRegistration::ICPConvergenceCriteria(
        useTransformationTolerance, relativeTranslation, 
        relativeRotation, maxIteration, previousMaxIterations);
}

//////////////////////////////////////////////////////////////////////////////
// Parse registration options.
//////////////////////////////////////////////////////////////////////////////
void ICPRegistrationImpl::parseRegistrationOptions(const mxArray* prhs){

    verbose = static_cast<bool>(mxGetScalar(
        mxGetField(prhs, 0, "Verbose")));

    useInlierRatio = static_cast<bool>(mxGetScalar(
        mxGetField(prhs, 0, "UseInlierRatio")));

    if(useInlierRatio){
        metric = static_cast<double>(mxGetScalar(
            mxGetField(prhs, 0, "InlierRatio"))); 
    } else {
        metric = static_cast<double>(mxGetScalar(
            mxGetField(prhs, 0, "MaxInlierDistance")));
    }

    method = matrix::get_string(mxGetField(prhs, 0, "Method"));

    // Parse check flag for input colors and using color values.
    useColor = static_cast<bool>(mxGetScalar(mxGetField(prhs, 0, "UseColors")));
}