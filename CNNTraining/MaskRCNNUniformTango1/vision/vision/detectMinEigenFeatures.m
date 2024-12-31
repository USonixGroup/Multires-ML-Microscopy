function pts = detectMinEigenFeatures(I,varargin)

% Copyright 2012-2023 The MathWorks, Inc.

%#codegen
pts = vision.internal.detector.harrisMinEigen('MinEigen', I, varargin{:});
