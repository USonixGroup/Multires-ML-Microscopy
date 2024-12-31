////////////////////////////////////////////////////////////////////////////////
//
//  3x3 Real Symmetric Eigen solver with Householder transform and QL method
//
// John Mathews, "Numerical methods for Mathematics, Science, and Engineering, 2nd Edition"
// http://en.wikipedia.org/wiki/Householder_transformation
// http://beige.ucs.indiana.edu/B673/node38.html
////////////////////////////////////////////////////////////////////////////////

// Copyright 2021 The MathWorks, Inc.

#ifndef TMW_REALSYMMETRICEIGENSOLVER_HPP
#define TMW_REALSYMMETRICEIGENSOLVER_HPP

#include <matrix.h>
#ifndef COMPILE_FOR_VISION_BUILTINS
#include "realSymmetricEigenSolver_core.hpp"
#else
#include <vision/realSymmetricEigenSolver_core.hpp>
#include <mex.h>
#endif

#endif