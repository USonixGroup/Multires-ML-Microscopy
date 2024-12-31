///////////////////////////////////////////////////////////////////////////
//
//  optimizePOsesCore contains
//  implementation of optimizePoses_published_c_api.
//
///////////////////////////////////////////////////////////////////////////
#ifdef COMPILE_FOR_VISION_BUILTINS
#include <optimizePoses/vision_defines.h>
#include <optimizePoses/optimizePoses_published_c_api.hpp>
#include "g2o/solve/g2oSolve.h"
#else
#include "optimizePoses_published_c_api.hpp"
#include "g2o/config.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/optimization_algorithm.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"
#include "g2o/core/sparse_optimizer.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/sim3/sim3.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"
#endif
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)
#endif //_MSC_VER

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <algorithm> // for std::max
#include <cmath>
#include <vector>
static bool hasToStop = false;
///////////////////////////////////////////////////////////////////
// Function to convert transformation matrix to pose containing
// translation and normalized quaternion
///////////////////////////////////////////////////////////////////
void tform2quatpose(void* mTform, double* pose) {
    const double* tform = static_cast<const double*>(mTform);
    double Qxx = tform[0];
    double Qxy = tform[1];
    double Qxz = tform[2];
    double Qyx = tform[4];
    double Qyy = tform[5];
    double Qyz = tform[6];
    double Qzx = tform[8];
    double Qzy = tform[9];
    double Qzz = tform[10];

    double t = Qxx + Qyy + Qzz;

    double r, s, w, x, y, z;
    if (t >= 0) {
        r = sqrt(1 + t);
        s = 0.5 / r;
        w = 0.5 * r;
        x = (Qzy - Qyz) * s;
        y = (Qxz - Qzx) * s;
        z = (Qyx - Qxy) * s;
    } else {
        double maxVal = std::max(Qxx, std::max(Qyy, Qzz));

        if (maxVal == Qxx) {
            r = sqrt(1 + Qxx - Qyy - Qzz);
            s = 0.5 / r;
            w = (Qzy - Qyz) * s;
            x = 0.5 * r;
            y = (Qyx + Qxy) * s;
            z = (Qxz + Qzx) * s;
        } else if (maxVal == Qyy) {
            r = sqrt(1 + Qyy - Qxx - Qzz);
            s = 0.5 / r;
            w = (Qxz - Qzx) * s;
            x = (Qyx + Qxy) * s;
            y = 0.5 * r;
            z = (Qzy + Qyz) * s;
        } else {
            r = sqrt(1 + Qzz - Qxx - Qyy);
            s = 0.5 / r;
            w = (Qyx - Qxy) * s;
            x = (Qxz + Qzx) * s;
            y = (Qzy + Qyz) * s;
            z = 0.5 * r;
        }
    }
    pose[0] = tform[3];
    pose[1] = tform[7];
    pose[2] = tform[11];
    pose[3] = w;
    pose[4] = x;
    pose[5] = y;
    pose[6] = z;
}

///////////////////////////////////////////////////////////////////
// Function to convert pose containing translation and normalized
// quaternion to transformation matrix
///////////////////////////////////////////////////////////////////
void quatpose2tform(const double* pose, double* tform) {

    tform[3] = pose[0];
    tform[7] = pose[1];
    tform[11] = pose[2];
    tform[12] = double(0.0);
    tform[13] = double(0.0);
    tform[14] = double(0.0);
    tform[15] = double(1.0);

    double q0 = pose[3];
    double qx = pose[4];
    double qy = pose[5];
    double qz = pose[6];

    double q02(q0 * q0), qx2(qx * qx), qy2(qy * qy), qz2(qz * qz);
    double qxy(qx * qy), q0z(q0 * qz), qxz(qx * qz), q0y(q0 * qy);
    double qyz(qy * qz), q0x(q0 * qx);
    tform[0] = q02 + qx2 - qy2 - qz2;
    tform[1] = 2 * qxy - 2 * q0z;
    tform[2] = 2 * qxz + 2 * q0y;
    tform[4] = 2 * qxy + 2 * q0z;
    tform[5] = q02 - qx2 + qy2 - qz2;
    tform[6] = 2 * qyz - 2 * q0x;
    tform[8] = 2 * qxz - 2 * q0y;
    tform[9] = 2 * qyz + 2 * q0x;
    tform[10] = q02 - qx2 - qy2 + qz2;
}

///////////////////////////////////////////////////////////////////
// Function converts 2D matrix nodes to 2-D vector
///////////////////////////////////////////////////////////////////
void initializeVectors(const int numNodes, const int poseDim, void* mNodes, void** nodesVec) {
    double* nodesTab = static_cast<double*>(mNodes);
    std::vector<std::vector<double>>* nodes = new std::vector<std::vector<double>>();
    *nodesVec = (void*)nodes;

    std::vector<std::vector<double>>& nodesOut = *nodes;
    for (int i = 0; i < numNodes; i++) {
        std::vector<double> v1;
        for (int j = 0; j < poseDim; j++) {
            v1.push_back(nodesTab[i + (j * numNodes)]);
        }
        nodesOut.push_back(v1);
    }
}

///////////////////////////////////////////////////////////////////
// Function optimizes the absolute Poses of nodes and stores them
// in mOptimNodes
///////////////////////////////////////////////////////////////////
void poseOptimizer(const double blockSolverType,
                   const double maxIterations,
                   const double functionTolerance,
                   const double verbose,
                   const double maxTime,
                   void* mNodes,
                   void* mEdges,
                   void* mInfoMats,
                   double* mOptimNodes,
                   double* structValues) {
    std::vector<std::vector<double>>& nodes = *((std::vector<std::vector<double>>*)mNodes);
    std::vector<std::vector<double>>& relPoses = *((std::vector<std::vector<double>>*)mEdges);
    std::vector<std::vector<double>>& infoMats = *((std::vector<std::vector<double>>*)mInfoMats);
    std::vector<std::vector<double>> solvedNodes;
    solvedNodes.reserve(nodes.size());

    double finalChi = 0;
    double numIterations = 0;
    double exitFlag = 0;
    double solverId = 0; // Levenberg-Marquardt Eigen Sparse solver
    bool computeGuess = false;
    #ifdef COMPILE_FOR_VISION_BUILTINS
    g2oSolve(nodes, relPoses, infoMats, solvedNodes, finalChi, numIterations, exitFlag, solverId,
             blockSolverType, maxIterations, maxTime, functionTolerance, verbose, computeGuess);
    #else
    std::string strSolver;
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(verbose);
    optimizer.setForceStopFlag(&hasToStop);

    g2o::SparseOptimizerTerminateAction* terminateAction = 0;
    terminateAction = new g2o::SparseOptimizerTerminateAction;
    terminateAction->setGainThreshold(functionTolerance);
    terminateAction->setMaxIterations(maxIterations);
    optimizer.addPostIterationAction(terminateAction);
    double ts = g2o::get_monotonic_time();

    std::vector<double> firstVec = nodes[1];
    int VertexType = (int)blockSolverType;
    if (blockSolverType == 1) {
        // SE3 optimization using 6X3 fixed block solver
        std::unique_ptr<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>> linearSolver
            (new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>());

        std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr(new g2o::BlockSolver_6_3(std::move(linearSolver)));

        g2o::OptimizationAlgorithmLevenberg* solver =
            new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));

        optimizer.setAlgorithm(solver);
    } else{
        // SIM3 optimization using 7X3 fixed block solver
        std::unique_ptr<g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>> linearSolver
            (new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>());

        std::unique_ptr<g2o::BlockSolver_7_3> solver_ptr(new g2o::BlockSolver_7_3(std::move(linearSolver)));

        g2o::OptimizationAlgorithmLevenberg* solver =
            new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));

        optimizer.setAlgorithm(solver);
    }
    switch (VertexType) {
        case 1:
            // add SE3 Vertices to optimizer
            for (unsigned int i = 0; i < nodes.size(); i++) {
                g2o::VertexSE3* vSE3 = new g2o::VertexSE3;

                g2o::SE3Quat pose;

                std::vector<double> node = nodes[i];

                /*g2o::Vector3D t_;
                t_(0) = node[0];
                t_(1) = node[1];
                t_(2) = node[2];


                Eigen::Quaterniond q(node[3], node[4], node[5], node[6]);
                pose.setTranslation(t_);
                pose.setRotation(q);*/
                g2o::Vector7 v7;
                v7 << node[0],node[1],node[2],node[4],node[5],node[6],node[3];
                vSE3->setEstimate(g2o::internal::fromVectorQT(v7));
                vSE3->setId(i);

                optimizer.addVertex(vSE3);
            }


                for (unsigned int i = 0; i < relPoses.size(); i++) {
                    std::vector<double> edge = relPoses[i];
                    g2o::EdgeSE3* odometry = new g2o::EdgeSE3;
                    odometry->vertices()[0] = optimizer.vertex(int(edge[0]));
                    odometry->vertices()[1] = optimizer.vertex(int(edge[1]));

                    g2o::Vector7 v7d;
                    v7d[0] = edge[2];
                    v7d[1] = edge[3];
                    v7d[2] = edge[4];
                    //g2o::Vector3D t_;
                    v7d[3] = edge[6];
                    v7d[4] = edge[7];
                    v7d[5] = edge[8];
                    v7d[6] = edge[5];

                    odometry->setMeasurement(g2o::internal::fromVectorQT(v7d));


                    typedef Eigen::Matrix<double, 6, 6> Matrix6d;

                    Matrix6d information;
                    information.fill(0.);

                    std::vector<double> infomat = infoMats[i];
                    int ind = 0;
                    for (int k = 0; k < 6; ++k) {
                        for (int l = k; l < 6; ++l) {
                            information(k, l) = infomat[ind];
                            ind = ind + 1;
                            if (k != l) {
                                information(l, k) = information(k, l);
                            }
                        }
                    }


                    odometry->setInformation(information);
                    optimizer.addEdge(odometry);
                }
                    break;
        default:
            // add SIM3 Vertices to optimizer
            for (unsigned int i = 0; i < nodes.size(); i++) {
                std::vector<double> node = nodes[i];
                g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();
                Eigen::Quaterniond q(node[3], node[4], node[5], node[6]);
                g2o::Vector3 t_(node[0], node[1], node[2]);
                g2o::Sim3 pose(q, t_, 1.0);
                if (firstVec.size() > 7) {
                    pose = g2o::Sim3(q, t_, node[7]);
                }
                // Sim3 solver expectes that vertex stores world to camera coordinate transformation matrix which is the inverse of robot world pose
                VSim3->setEstimate(pose.inverse());
                VSim3->setId(i);

                VSim3->_fix_scale = false;
                optimizer.addVertex(VSim3);
            }


                for (unsigned int i = 0; i < relPoses.size(); i++) {
                    std::vector<double> edge = relPoses[i];
                    g2o::EdgeSim3* odometry = new g2o::EdgeSim3();
                    odometry->vertices()[0] = optimizer.vertex(int(edge[0]));
                    odometry->vertices()[1] = optimizer.vertex(int(edge[1]));

                    Eigen::Quaterniond q(edge[5], edge[6], edge[7], edge[8]);
                    g2o::Vector3 t_(edge[2], edge[3], edge[4]);
                    g2o::Sim3 pose(q, t_, 1.0);
                    if (firstVec.size() > 7) {
                        pose = g2o::Sim3(q, t_, edge[9]);
                    }
                    // Sim3 solver expectes that edge stores transformation matrix to convert 3D points from first frame to second frame coordinate system which is the inverse of relative transformation between the robot poses
                    odometry->setMeasurement(pose.inverse());


                    typedef Eigen::Matrix<double, 7, 7> Matrix7d;

                    Matrix7d information;
                    information.fill(0.);

                    std::vector<double> infomat = infoMats[i];
                    int ind = 0;
                    for (int k = 0; k < 7; ++k) {
                        for (int l = k; l < 7; ++l) {
                            information(k, l) = infomat[ind];
                            ind = ind + 1;
                            if (k != l) {
                                information(l, k) = information(k, l);
                            }
                        }
                    }

                    odometry->setInformation(information);
                    optimizer.addEdge(odometry);
                }
    }

    g2o::OptimizableGraph::Vertex* v=optimizer.vertex(0);
    v->setFixed(1);

    // Initialize the Optimization
    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    if (computeGuess){
        optimizer.computeInitialGuess();
    }
    double dts = g2o::get_monotonic_time() - ts;
    double optimizationTimeLimit = maxTime - dts;

    // Optimize the pose graph
    int ite = optimizer.optimize(maxIterations, false);

    switch (VertexType) {
        case 1:
            // Fill Solved SE3 nodes
            for (unsigned int i = 0; i < nodes.size(); i++) {
                g2o::VertexSE3* vse3 = static_cast<g2o::VertexSE3*>(optimizer.vertex(i));
                g2o::Isometry3 pose = vse3->estimate();
                g2o::Vector7 v7d;
                v7d = g2o::internal::toVectorQT(pose);
                std::vector<double> solvedNode;
                solvedNode.clear();
                solvedNode.push_back(v7d[0]);
                solvedNode.push_back(v7d[1]);
                solvedNode.push_back(v7d[2]);
                solvedNode.push_back(v7d[6]);
                solvedNode.push_back(v7d[3]);
                solvedNode.push_back(v7d[4]);
                solvedNode.push_back(v7d[5]);
                solvedNodes.push_back(solvedNode);
            }
                break;
        default:
            // Fill Solved SIM3 nodes
            for (unsigned int i = 0; i < nodes.size(); i++) {
                g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(i));
                // Sim3 vertices store transformation to convert 3D points in world frame to camera frame. Inverse of this transformation represents camera world pose. Returning the world pose of the camera as the solved node.
                g2o::Sim3 pose = VSim3->estimate().inverse();
                g2o::Vector3 v3d = pose.translation();
                Eigen::Quaterniond q = pose.rotation();
                double s = pose.scale();
                std::vector<double> solvedNode;
                solvedNode.clear();
                // Returning the solved nodes in [tx,ty,tx,qw,qx,qy,qz,scale] format
                solvedNode.push_back(v3d[0]);
                solvedNode.push_back(v3d[1]);
                solvedNode.push_back(v3d[2]);
                solvedNode.push_back(q.w());
                solvedNode.push_back(q.x());
                solvedNode.push_back(q.y());
                solvedNode.push_back(q.z());
                solvedNode.push_back(s);
                solvedNodes.push_back(solvedNode);
            }
    }

    finalChi = optimizer.activeRobustChi2();
    #endif
    int k = 0;
    for (size_t i = 0; i < solvedNodes.size(); i++) {
        for (size_t j = 0; j < solvedNodes[0].size(); j++) {
            mOptimNodes[k++] = solvedNodes[i][j];
        }
    }
    if (numIterations == maxIterations) {
        exitFlag = 2;
    }
    if (exitFlag == 5) {
        numIterations++;
    }
    structValues[0] = finalChi;
    structValues[1] = numIterations;
    structValues[2] = exitFlag;

    delete ((std::vector<std::vector<double>>*)mNodes);
    delete ((std::vector<std::vector<double>>*)mEdges);
    delete ((std::vector<std::vector<double>>*)mInfoMats);
}