function [tform, movingReg, rmse] = pcregistericp(moving, fixed, varargin)

% Copyright 2014-2024 The MathWorks, Inc.

%#codegen

% Validate inputs
narginchk(2, 16)
[unorgMoving, unorgFixed, configOptions, doExtrapolate, metric] = vision.internal.pc.parseICPInputs( ...
    moving, fixed, mfilename, varargin{:});

useAcceleration = strcmpi(metric, "planeToPlane") || ...
    strcmpi(metric, "pointToPlaneWithColor") || ...
    strcmpi(metric, "planeToPlaneWithColor") || ...
    ~doExtrapolate;

if coder.gpu.internal.isGpuEnabled
    % Check that only supported metrics are given for CUDA code generation
    coder.internal.errorIf(strcmpi(metric, "pointToPlaneWithColor") || strcmpi(metric, "planeToPlaneWithColor"), ...
        'vision:pointcloud:unsupportedGPUCodegen');
    icpReg = vision.internal.pc.ICPRegistrationGPU(unorgMoving, unorgFixed, configOptions);
elseif useAcceleration
    icpReg = vision.internal.pc.ICPRegistration(unorgMoving, unorgFixed, configOptions);
else
    icpReg = vision.internal.pc.ICPRegistrationGPU(unorgMoving, unorgFixed, configOptions);
end
icpReg.initializeRegistration;
tform = icpReg.registerPointClouds;
if nargout > 1
    movingReg = pctransform(moving, tform);
    if nargout == 3
        rmse = cast(vision.internal.pc.rmse(removeInvalidPoints(movingReg), ...
            icpReg.Fixed), 'like', moving.Location);
    end
end
end
