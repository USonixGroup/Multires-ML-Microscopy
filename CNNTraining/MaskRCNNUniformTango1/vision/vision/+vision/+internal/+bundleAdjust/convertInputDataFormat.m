function [cameraMatrices, quaternionBases, intrinsicsStruct, returnPoseType] = convertInputDataFormat(...
    cameraPoses, intrinsics, isUndistorted, optimType)
%convertInputDataFormat Convert inputs to internal data structure.
%   All returned values are double.
%
%   cameraMatrices: 6-by-V, rotation vector + translation vector
%   quaternionBases: 4-by-V, quaternions for initial rotations
%   intrinsicsStruct: a structure of camera intrinsics

% Copyright 2019-2022 The MathWorks, Inc.

%#codegen

if strcmp(optimType, 'motion')
    
    % Convert the intrinsics object to a simple structure
    % Note, the internal reprojection function uses a different definition
    % of skew factor, i.e., s = S / fc(1)
    intrinsicsStruct = struct('focalLength',double(intrinsics.FocalLength), ...
        'principalPoint', double(intrinsics.PrincipalPoint), ...
        'radialDistortion', zeros(coder.ignoreConst(1),coder.ignoreConst(3)), ...
        'tangentialDistortion', zeros(coder.ignoreConst(1),coder.ignoreConst(2)), ...
        'skew', double(intrinsics.Skew / intrinsics.FocalLength(1)));
    coder.varsize('intrinsicsStruct.radialDistortion');
    if ~isUndistorted
        % Skip if the distortion coefficients are all zeros
        if any(intrinsics.RadialDistortion) || any(intrinsics.TangentialDistortion)
            intrinsicsStruct.radialDistortion = double(intrinsics.RadialDistortion);
            intrinsicsStruct.tangentialDistortion = double(intrinsics.TangentialDistortion);
        end
    end

else % structure or full

    % Convert cameraParameters to cameraIntrinsics to allow parentheses-style indexing
    if isa(intrinsics, 'cameraParameters')
        camIntrinsics = convertToIntrinsics(intrinsics);
    else 
        camIntrinsics = intrinsics;
    end
    
    numCameras = numel(camIntrinsics);
    
    st = struct('focalLength',zeros(coder.ignoreConst(1),coder.ignoreConst(2)), ...
        'principalPoint', zeros(coder.ignoreConst(1),coder.ignoreConst(2)), ...
        'radialDistortion', zeros(coder.ignoreConst(1),coder.ignoreConst(3)), ...
        'tangentialDistortion', zeros(coder.ignoreConst(1),coder.ignoreConst(2)), ...
        'skew', zeros(coder.ignoreConst(1),coder.ignoreConst(1)));
    coder.varsize('st.radialDistortion');
    intrinsicsStruct = repmat(st, 1, numCameras);
    
    for n = 1:numCameras
        % Convert the intrinsics object to a simple structure
        intrinsicsStruct(n).focalLength = double(camIntrinsics(n).FocalLength);
        intrinsicsStruct(n).principalPoint = double(camIntrinsics(n).PrincipalPoint);
        if ~isUndistorted
            % Skip if the distortion coefficients are all zeros
            if any(camIntrinsics(n).RadialDistortion(:)) || any(camIntrinsics(n).TangentialDistortion(:))
                intrinsicsStruct(n).radialDistortion = double(camIntrinsics(n).RadialDistortion);
                intrinsicsStruct(n).tangentialDistortion = double(camIntrinsics(n).TangentialDistortion);
             end
        end
        % Note, the internal reprojection function uses a different definition
        % of skew factor, i.e., s = S / fc(1)
        intrinsicsStruct(n).skew = double(camIntrinsics(n).Skew / camIntrinsics(n).FocalLength(1));
    end
 end

[cameraMatrices, quaternionBases, returnPoseType] = convertToProjectionMatrices(cameraPoses);

end
%==========================================================================
% Convert cameraParameters to cameraIntrinsics
%==========================================================================
function intrinsics = convertToIntrinsics(camParam)
% Image size is not used in bundleAdjustment, but required in
% constructing the cameraIntrinsics object. When it is not available in
% the cameraParameters-type input, set it to a default value.
imSize = camParam.ImageSize;
if isempty(imSize)
    imageSize = [1 1];
else
    imageSize = imSize;
end

intrinsics = cameraIntrinsics(camParam.FocalLength, camParam.PrincipalPoint, imageSize);
end

%==========================================================================
% Convert cameraPoses to a compact form
%==========================================================================
function [cameraMatrices, quaternionBases, returnPoseType] = convertToProjectionMatrices(cameraPoses)
% Convert camera poses to a compact form of camera projection matrices
% Use quaternion for numerical stability

if isa(cameraPoses, 'rigidtform3d') || isa(cameraPoses, 'rigid3d') % In motion mode, cameraPoses is a rigidtform3d or rigid3d object
    cameraMatrices = zeros(6,1);
    t = double(cameraPoses.T(4,1:3));
    R = double(cameraPoses.T(1:3,1:3));
    cameraMatrices(4:6, 1) = -t*R';
    quaternionBases = vision.internal.quaternion.rotationToQuaternion(R);
    
    returnPoseType = class(t);
else

    if isSimMode()
        hasAbsolutePose = ismember('AbsolutePose', cameraPoses.Properties.VariableNames);
    else
        if isstruct(cameraPoses)
            hasAbsolutePose = isfield(cameraPoses, 'AbsolutePose');
        else
            hasAbsolutePose = any(strcmp('AbsolutePose', cameraPoses.VariableNames));
        end
    end

    if hasAbsolutePose
        absPoses       = cameraPoses.AbsolutePose;
        
        ts = zeros(1, 3, numel(absPoses));
        Rs = zeros(3, 3, numel(absPoses));

        coder.varsize("ts");
        coder.varsize("Rs");
        ts(:,:,1) = absPoses(1).Translation;
        Rs(:,:,1) = absPoses(1).Rotation;

        for i = 2:numel(absPoses)
            ts(:,:,i) = absPoses(i).Translation;
            Rs(:,:,i) = absPoses(i).Rotation;
        end
        
        returnPoseType = class(absPoses(1).Translation);
    else
        loc = cameraPoses.Location;
        or = cameraPoses.Orientation;
        ts = zeros(1, 3, size(loc,1));
        Rs = zeros(3, 3, size(loc,1));
        coder.varsize("ts");
        coder.varsize("Rs");

        ts(:,:,1) = loc{1};
        Rs(:,:,1) = or{1};

        for i = 2:size(loc,1)
            t = loc{i};
            r = or{i};
            ts(:,:,i) = t;
            Rs(:,:,i) = r;
        end
        returnPoseType = class(cameraPoses.Location{1});       
    end

    if isSimMode()
        cameraMatrices          = zeros(6, height(cameraPoses)); % double
        cameraMatrices(4:6, :)  = squeeze(pagemtimes(-ts, pagetranspose(Rs)));
    else
        cameraMatrices = zeros(6, size(cameraPoses.ViewId, 1));
        rsTranspose = transposePage(Rs);
        tsRs = mtimesPage(-ts, rsTranspose);
        cameraMatrices(4:6, :) = squeeze(tsRs);
    end

    quaternionBases = vision.internal.quaternion.rotationToQuaternion(double(Rs));
end
end

function trans = transposePage(mat)
    trans = zeros(size(mat));
    for i = 1:size(mat,3)
        trans(:,:,i) = mat(:,:,i).';
    end
end

function mtimes = mtimesPage(x, y)
    mtimes = zeros(size(x, 1), size(y, 2), size(x, 3));

    for i = 1:size(x,3)
        mtimes(:,:,i) = x(:,:,i)*y(:,:,i);
    end
end

function out = isSimMode()
    out = isempty(coder.target);
end