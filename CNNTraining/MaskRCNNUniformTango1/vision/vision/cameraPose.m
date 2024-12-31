%cameraPose Compute relative rotation and translation between camera poses
%   -----------------------------------------------------------------------
%   cameraPose was renamed to relativeCameraPose. Please use the new 
%   function in place of cameraPose.
%   -----------------------------------------------------------------------
%
%  See also relativeCameraPose

%#codegen

function [orientation, location, validPointsFraction] = ...
    cameraPose(F, varargin)

%   Copyright 2015-2020 The MathWorks, Inc.

[orientation, location, validPointsFraction] = relativeCameraPose(F, varargin{:});
