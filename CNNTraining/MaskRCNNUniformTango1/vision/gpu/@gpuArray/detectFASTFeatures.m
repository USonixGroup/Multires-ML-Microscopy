function pts = detectFASTFeatures(I, varargin)
% detectFASTFeatures Find corners using the FAST algorithm on the GPU
%
% GPU support for detectFASTFeatures is removed.
% For GPU support, use detectHarrisFeatures instead.

% Copyright 2015-2021 The MathWorks, Inc.

error(message('vision:gpu:supportRemoval', mfilename, 'detectHarrisFeatures'));
