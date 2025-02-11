/* Copyright 2019-2020 The MathWorks, Inc. */

/**
 * @file
 * Header for pose graph optimization
 */

#ifndef POSE_GRAPH_OPTIMIZER_HPP
#define POSE_GRAPH_OPTIMIZER_HPP

#include "g2ocv/libmwg2ocv_util.hpp"

#include <vector>

#include <mfl_scalar/constants.hpp>         // for getInf

#include "g2o/solve/g2oSolve.h"

namespace vision {
    /**
     * @brief Object that wraps g2o-based pose graph optimization
     */
    class LIBMWG2OCV_API PoseGraphOptimizer {
    public:
        /**
         * @brief Create a PoseGraphOptimizer object
         *
         * @param[in] blockSolverType Block solver type, specified as a double value
         *      1 - Fixed Block Solver with block dimensions 6,3 (SE3 Solver)
         *      2 - Fixed Block Solver with block dimensions 7,3 (SIM3 Solver)
         * @param[in] maxIter Maximum number of iterations, specified as a double value
         * @param[in] funcTol Function tolerance, specified as a double value
         * @param[in] verboseFlag Verbose flag, specified as a bool value
         * @param[in] maxTime Maximum time, specified as a double value
         *
         */
        PoseGraphOptimizer(const double blockSolverType, const double maxIter, const double funcTol, const bool verboseFlag, const double maxTime)
        : BlockSolverType(blockSolverType), MaxIterations(maxIter), FunctionTolerance(funcTol), Verbose(verboseFlag), MaxTime(maxTime) {}
        
        /**
         * @brief Optimize pose graph
         *
         * @param[in] Nodes Absolute node poses to be optimized, specified 
         *      as a vector of double vectors. Each double vector holds 
         *      - a 7-element SE3 pose or 
         *      - an 8-element SIM3 pose.
         * @param[in] Edges Relative edge constraints, specified as a vector 
         *      of double vectors. Each double vector holds 
         *      - 9 doubles: a 2-element edge id and 7-element SE3 pose or
         *      - 10 doubles: a 2-element edge id and 8-element SIM3 pose
         * @param[in] InfoMats Information matrices, specified as a vector 
         *      of double vectors. Each double vector holds a 21-element 
         *      compact information matrix.
         * @param[out] OptimNodes Optimized absolute node poses, specified 
         *      as a vector of double vectors. Each double vector holds 
         *      - a 7-element SE3 pose or
         *      - an 8-element SIM3 pose
         *
         */
        void optimize(const std::vector<std::vector<double>>& Nodes,
                      const std::vector<std::vector<double>>& Edges,
                      const std::vector<std::vector<double>>& InfoMats,
                      std::vector<std::vector<double>>& OptimNodes) {
            
            g2oSolve(Nodes, Edges, InfoMats, OptimNodes, FinalChi, NumIterations,
                     ExitFlag, SolverId, BlockSolverType, MaxIterations, MaxTime,
                     FunctionTolerance, Verbose, ComputeGuess);
            
            if (NumIterations == MaxIterations) {
                ExitFlag = 2;
            }
            if (ExitFlag == 5) {
                NumIterations++;
            }
        }
        
        /**
         * @brief Get final chi
         */
        double finalChi() const {
            return FinalChi;
        }
        
        /**
         * @brief Get number of iterations
         */
        double numIterations() const {
            return NumIterations;
        }
        
        /**
         * @brief Get exit flag
         */
        double exitFlag() const {
            return ExitFlag;
        }
        
    private:
        // Modifiable parameters
        double BlockSolverType;
        double MaxIterations;
        double FunctionTolerance;
        bool   Verbose;
        double MaxTime;
        
        // Constant parameters
        const double SolverId = 0;                              // Levenberg-Marquardt Eigen Sparse solver
        const bool ComputeGuess = false;
        
        // Filled by solve
        double FinalChi;
        double NumIterations;
        double ExitFlag;
    };
} // end namespace vision

#endif /* POSE_GRAPH_OPTIMIZER_HPP */
