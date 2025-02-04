function [grad, hess] = gradientHessian(qryPtMeanSubtracted, iCov, d1, d2, Jp, Hp)
% Computes the gradient and Hessian
% The gradient is the vector of first partial derivative of a scalar field.
% The Hessian a square matrix of second-order partial 
% derivatives of a scalar-valued function, or scalar field.

% [1] Martin Magnusson, The Three-Dimensional Normal-Distributions
% Transform - an Efficient Representation for Registration, Surface
% Analysis, and Loop Detection, Thesis, 2013

% $matlabroot/toolbox/vision/builtins/src/vision/include/ndtUtils.hpp

% Copyright 2021 The MathWorks, Inc.

%#codegen

% Evaluate gradient at the point Eq: 6.12 [1]

% Compute qt*C*q
qC = transpose(qryPtMeanSubtracted) * iCov;
qCq = zeros(1, 'like', qryPtMeanSubtracted);
for i = 1:3
    qCq = qCq + qC(i) .* qryPtMeanSubtracted(i);
end

% Compute qt*C*(dx/dp)
qCJ = qC * Jp;
c = d1 * d2 * exp((-d2/2) * qCq);
grad = c .* qCJ;

% Evaluate Hessian at the point Eq: 6.13 [1]
JpT = transpose(Jp);
JJ = JpT * iCov * Jp;
qCJ_sq = transpose(qCJ) * qCJ;
h = qC(1) * Hp(:,:,1) + qC(2) * Hp(:,:,2) + qC(3) * Hp(:,:,3);

hess = c * (-d2 * qCJ_sq + h + JJ);
end

