function [R, t, pts, outputClass, varargout] = parseProjectionInputs(intrinsics,...
    rotationMatrix, translationVector, points, isCameraParamsSupported, filename, varargin)
% parseProjectionInputs Function to validate the camera projection inputs.

% Copyright 2019-2022 The MathWorks, Inc.

%#codegen

nargoutchk(0,6);

% if any of the inputs is double, internal math is done in
% doubles.  Otherwise, internal math is  done in singles.

if isa(rotationMatrix, 'double') || isa(translationVector, 'double')...
        || isa(points, 'double')
    R = double(rotationMatrix);
    tTemp = double(translationVector);
    pts = double(points);
    if isa(intrinsics, 'cameraIntrinsics') || ...
      (isCameraParamsSupported && isa(intrinsics, 'cameraParameters'))
        K = double(intrinsics.K);
    elseif isa(intrinsics, 'cameraIntrinsicsKB') || ...
      (isCameraParamsSupported && isa(intrinsics, 'cameraParametersKB'))
        K = double(intrinsics.K);
    elseif isa(intrinsics, 'cameraParameters')
        % cameraParameters are only supported for pointsToWorld and worldToImage
        coder.internal.error('vision:calibrate:cameraParametersUnsupported');
    end
else
    R = single(rotationMatrix);
    tTemp = single(translationVector);
    pts = single(points);
    if isa(intrinsics, 'cameraIntrinsics') || ...
      (isCameraParamsSupported && isa(intrinsics, 'cameraParameters'))
        K = single(intrinsics.K);
    elseif isa(intrinsics, 'cameraParameters')
        % cameraParameters are only supported for pointsToWorld and worldToImage
        coder.internal.error('vision:calibrate:cameraParametersUnsupported');
    end
end

% Force t to be a row vector.
t = tTemp(:)';

% if points is double, then the output is double. Else the output is single
if isa(points, 'double')
    outputClass = 'double';
else
    outputClass = 'single';
end

% cameraIntrinsics
if isempty(coder.target) %matlab
    parser = inputParser;
    parser.addParameter('ApplyDistortion', false, @checkApplyDistortion);
    parser.parse(varargin{:});
    applyDistortion = parser.Results.ApplyDistortion;
else % codegen
    params = struct('ApplyDistortion', uint32(0));
    popt = struct(...
        'CaseSensitivity', false, ...
        'StructExpand',    true, ...
        'PartialMatching', true);
    pstruct = coder.internal.parseParameterInputs(params, popt, varargin{:});   
    applyDistortion = coder.internal.getParameterValue(pstruct.ApplyDistortion, ...
        false, varargin{:});
    checkApplyDistortion(applyDistortion);
end

% Additional outputs for cameraIntrinsics
if nargout > 4
    varargout{1} = K;
    varargout{2} = applyDistortion;
end
end

%--------------------------------------------------------------------------
function tf = checkApplyDistortion(val)                
vision.internal.inputValidation.validateLogical(val, 'ApplyDistortion');
tf = true;
end
