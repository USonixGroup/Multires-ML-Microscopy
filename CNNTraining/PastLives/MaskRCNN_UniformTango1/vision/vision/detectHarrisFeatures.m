function pts = detectHarrisFeatures(I,varargin)

% Copyright 2012-2023 The MathWorks, Inc.

%#codegen
pts = vision.internal.detector.harrisMinEigen('Harris', I, varargin{:});
