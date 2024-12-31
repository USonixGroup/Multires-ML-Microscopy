function varargout = readAprilTag(I, varargin)

% Copyright 2020-2024 The MathWorks, Inc.

%#codegen

narginchk(1,8);

% Parse positional inputs and name-value arguments.
[imgUint8, tagFamily, doEstimatePose, intrinsics, tagSize, detectorParams] = ...
    vision.internal.inputValidation.parseReadAprilTagInputs(I, varargin{:});

if ~doEstimatePose
    nargoutchk(0,3);
    [varargout{1:nargout}] = readAprilTagID(imgUint8, tagFamily, detectorParams);
else
    nargoutchk(0,4);
    [varargout{1:nargout}] = readAprilTagIDPose(imgUint8, tagFamily, intrinsics, tagSize, detectorParams);
end

end

%--------------------------------------------------------------------------
% isSimMode - check if simulation mode or codegen mode
%--------------------------------------------------------------------------
function out = isSimMode()
    out = isempty(coder.target);
end

%--------------------------------------------------------------------------
% readAprilTagID - Call AprilTag tag ID decoding
%--------------------------------------------------------------------------
function [id, loc, detectedFamily] = readAprilTagID(Img, tagFamily, detectorParams)


if isSimMode()
    [id, loc, detectedFamily] = vision.internal.aprilTagReader(Img.', tagFamily, false, detectorParams);
else
    [id, loc, detectedFamily] = vision.internal.buildable.readAprilTagBuildable.readAprilTagID(Img.', ...
        tagFamily, false, detectorParams);
end

loc = loc + 1; % AprilTag uses zero-indexing for images

if isSimMode()
    %output string array for detected families
    detectedFamily = string(detectedFamily);
end

end

%--------------------------------------------------------------------------
% readAprilTagIDPose - Call AprilTag tag ID decoding and pose estimation
%--------------------------------------------------------------------------
function [id, loc, pose, detectedFamily] = readAprilTagIDPose(I, tagFamily, intrinsics, tagSize, params)

focalLength = intrinsics.FocalLength;
principalPoint = intrinsics.PrincipalPoint;

if isSimMode()
    [id, loc, detectedFamily, rotMatrices, transVectors] = vision.internal.aprilTagReader(I.', tagFamily, ...
        true, params, focalLength, principalPoint, tagSize);
else
    [id, loc, detectedFamily, rotMatrices, transVectors] = vision.internal.buildable.readAprilTagBuildable.readAprilTagID(I.', tagFamily, ...
        true, params, focalLength, principalPoint, tagSize);
end

count = 1;
invalidIdx = [];

if isSimMode()
    pose = rigidtform3d.empty;
    for idx = 1:size(rotMatrices, 3)

        % Only accept tags with valid poses. An invalid rotation matrix points
        % to an incorrect detection. This is more prominent in the smaller tag
        % families like tag16h5 and tag25h9.
        try %#ok
            vision.internal.inputValidation.validateRotationMatrix(...
                rotMatrices(:,:,idx), 'readAprilTag', 'rotationMatrix');
            pose(count) = rigidtform3d(rotMatrices(:,:,idx)', transVectors(:,:,idx)');
            count = count + 1;
        catch
            invalidIdx = [invalidIdx idx]; %#ok<AGROW>
        end
    end
    
else
    
    % allocating memory for rigidtform3d object array
    dataType = 'double';
    T = eye(4, 4, dataType);
    
    % Create a dummy object
    pose = rigidtform3d(T);
    
    countValid = 1;
    for idx = 1:size(rotMatrices, 3)
        if checkRotationMatrix(rotMatrices(:,:,idx))
            
            if countValid == 1
                % Update pose
                pose = rigidtform3d(rotMatrices(:,:,idx)',transVectors(:,:,idx)');
            else
                % Update the array
                pose(countValid) = rigidtform3d(rotMatrices(:,:,idx)',transVectors(:,:,idx)');
            end
            countValid = countValid+1;
        else
            invalidIdx = [invalidIdx idx]; %#ok<AGROW>
        end
    end
    
end
id(invalidIdx) = [];
loc(:,:,invalidIdx) = [];
detectedFamily(invalidIdx) = [];

loc = loc + 1; % AprilTag uses zero-indexing for images

if isSimMode()
    detectedFamily = string(detectedFamily);
end

end

%--------------------------------------------------------------------------
% checkRotationMatrix - Validate rotation matrix
%--------------------------------------------------------------------------
function validFlag = checkRotationMatrix(M)

validateattributes(M, {'numeric'}, ...
    {'finite', '2d', 'real', 'nonsparse', 'size', [3,3]});

if abs(det(double(M))-1) > 1e-3
    validFlag = false;
else
    M = double(M);
    MM = M*M';
    I = eye(3);
    validFlag = true;
    if max(abs(MM(:)-I(:))) > 1e-3
        validFlag = false;
    end
end
end