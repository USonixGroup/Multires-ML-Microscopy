classdef fisheyeParameters < vision.internal.calibration.FisheyeParametersImpl
% This fisheyeParameters class contains Code generation implementation of fisheyeParameters
%
%   This class is for internal use only and may be removed in the future.
%
%   This class is used to implement code generation support for the
%   fisheyeParameters object.

% Copyright 2021 The MathWorks, Inc.

%#codegen
    methods
        %----------------------------------------------------------------------
        function this = fisheyeParameters(varargin)
            this@vision.internal.calibration.FisheyeParametersImpl(varargin{:});
        end
    end
end
