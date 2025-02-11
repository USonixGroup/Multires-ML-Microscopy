function validateNotObject(data, fcnName, varName)
% Validate that some data is not an object. This should be used in
% conjunction with validateattributes to exclude numeric-ish objects that
% pass validation but must not be passed to builtins or MEX.

% Copyright 2018 The MathWorks, Inc.

%#codegen

if coder.target('MATLAB')
    % For now we require fcnName and varName, but not varIdx.
    narginchk(3, 3);
    
    % If we get here then the data matches class and attributes, but may still
    % be an object. We use a hidden built-in to check isobject from C++ so that
    % no MCOS object can overload it.
    if feval('_visionIsObject', data)
        % We try to be helpful and tell the user how to fix the problem for objects
        % we know about.
        if ismethod(data, 'gather')
            msg = message("vision:validation:noObjectsGather", varName);
        else
            msg = message("vision:validation:noObjects", varName);
        end
        % Throw using a customized ID, as per validateattributes.
        if strlength(fcnName)==0
            msgId = "MATLAB:invalidObject";
        else
            msgId = "MATLAB:"+fcnName+":invalidObject";
        end
        throwAsCaller(MException(msgId, getString(msg)));
    end
end