function disparityMap = disparityBM(I1, I2, varargin)
% disparityBM Computes disparity map using block matching.
%
% GPU support for disparityBM is removed. For GPU
% support, use disparitySGM instead.
%
% Copyright 2017-2021 The MathWorks, Inc.

error(message('vision:gpu:supportRemoval', mfilename,'disparitySGM'));
