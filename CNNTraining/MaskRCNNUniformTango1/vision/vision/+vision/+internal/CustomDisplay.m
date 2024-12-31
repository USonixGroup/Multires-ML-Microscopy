%vision.interal.CustomDisplay Codegen compatible custom display mixin
%
%   matlab.mixin.CustomDisplay does not support code generation, so classes
%   inheriting from it will not be able to generate code. This class uses a
%   codegen redirect to a stub class in order to support code generation.

% Copyright 2019-2021 The MathWorks, Inc.
classdef CustomDisplay < matlab.mixin.CustomDisplay & handle
    
    methods (Access = public, Static, Hidden)
        %------------------------------------------------------------------
        function name = matlabCodegenRedirect(~)
            name = 'vision.internal.codegen.CustomDisplay';
        end
    end
end