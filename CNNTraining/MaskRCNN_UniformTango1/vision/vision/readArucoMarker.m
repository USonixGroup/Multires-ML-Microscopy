function varargout = readArucoMarker(I, varargin)

%   Copyright 2023-2024 The MathWorks, Inc.
%#codegen

% Validate number of inputs.
narginchk(1,24)

% Parse positional inputs and name-value arguments.
[I, markerFamily, doEstimatePose, intrinsics, markerSize, params] = ...
    vision.internal.inputValidation.parseReadArucoInputs(I, varargin{:});

if ~doEstimatePose
    % Validate number of outputs.
    nargoutchk(0,4);

    % Detect ArUco markers.
    [varargout{1:nargout}] = readArucoID(I, markerFamily, params);
else
    % Validate number of outputs in pose estimation syntax.
    nargoutchk(0,5);

    % Verify image size consistency between intrinsics and input image.
    vision.internal.inputValidation.checkImageSize(I, intrinsics.ImageSize);

    % Detect ArUco markers and estimate poses.
    [varargout{1:nargout}] = readArucoIDPose(I, markerFamily, intrinsics, markerSize, params);
end
end

%--------------------------------------------------------------------------
% readArucoID - Call ArUco marker ID decoding
%--------------------------------------------------------------------------
function [ids, locs, detectedFamily, rejections] = readArucoID(I, markerFamily, detectorParams)

if isSimMode
    [ids, locs, detectedFamily, rejectionsCell] = readAruco(I, markerFamily, detectorParams);
     % output string array for detected families
    detectedFamily = string(detectedFamily);
else
    [ids, locs, detectedFamily, rejectionsCell] = vision.internal.buildable.readArucoMarkerBuildable.readAruco(I, markerFamily, detectorParams);
end

if nargout == 4
    % Post process rejections.
    if isSimMode()
        allRejections = mergeRejectedCandidates(rejectionsCell);
    else
        allRejections = mergeRejectedCandidatesCodegen(rejectionsCell);
    end
    if ~isempty(allRejections)
        rejections = removeValidDetections(allRejections, locs);
    else
        rejections = zeros(4,2,0);
    end
end
end

%--------------------------------------------------------------------------
% readArucoIDPose - Call ArUco marker ID decoding and pose estimation
%--------------------------------------------------------------------------
function [ids, locs, pose, detectedFamily, rejections] = readArucoIDPose(I, markerFamily, intrinsics, markerSize, detectorParams)

% Convert CVT intrinsics to OpenCV.
[K, D] = cameraIntrinsicsToOpenCV(intrinsics);

% Convert K to row major layout to pass it to OpenCV directly.
Krowmajor = K';

% Detect ArUco markers and estimate poses.
if isSimMode()
    [ids, locs, detectedFamily, rejectionsCell, rMats, tvecs] = ...
        readAruco(I, markerFamily, detectorParams, markerSize, Krowmajor, D);

    % output string array for detected families
    detectedFamily = string(detectedFamily);
else
    [ids, locs, detectedFamily, rejectionsCell, rMats, tvecs] = vision.internal. ...
        buildable.readArucoMarkerBuildable.readAruco(I, markerFamily, detectorParams,...
        markerSize, Krowmajor, D);
end

% Construct rigidtform3d poses.
pose = constructPoses(rMats, tvecs);

% Post process rejections.
if nargout == 5
    if isSimMode()
        allRejections = mergeRejectedCandidates(rejectionsCell);
    else
        allRejections = mergeRejectedCandidatesCodegen(rejectionsCell);
    end
    rejections = removeValidDetections(allRejections, locs);
end
end

%--------------------------------------------------------------------------
function poses = constructPoses(rMats, tvecs)

numPoses = coder.internal.indexInt(size(tvecs,2));
poses =  rigidtform3d(rMats(:,:,1), tvecs(:,1));
if numPoses >= 2
    for i = 2:numPoses
        % Construct pose.
        tvec = tvecs(:,i);
        rMat = rMats(:,:,i);
        poses(i) = rigidtform3d(rMat, tvec);
    end
end
end

%--------------------------------------------------------------------------
% mergeRejectedCandidates combines rejected candidates from all marker
% families keeping only one copy of a candidate in the returned result.
%
% Each candidate is a 4-by-2 polygon represented by its 4 corner locations.
%
% Input: inRejections - numFamilies-by-1 cell array of 4-by-2-by-N matrices
% Output: allRejections - 4-by-2-by-M matrix
%--------------------------------------------------------------------------
function allRejections = mergeRejectedCandidates(inRejections)
% Initialize an empty matrix for all polygons

% Concatenate all rejected candidates.
allPolygons = cat(3,inRejections{:});

% Reshape and transpose the 4-by-2-by-N matrix into a N-by-8 matrix.
% Now each row will respresent a candidate polygon.
allPolygons = reshape(allPolygons, 8, [])';

% Initialize an empty matrix for unique polygons
uniquePolygons = [];

% Loop through each row (polygon) in the allPolygons matrix
for i = 1:size(allPolygons, 1)
    % Check if the current polygon is already in the uniquePolygons matrix
    isUnique = true;
    for j = 1:size(uniquePolygons, 1)
        if isequal(allPolygons(i, :), uniquePolygons(j, :))
            isUnique = false;
            break;
        end
    end

    % If the current polygon is unique, add it to the uniquePolygons matrix
    if isUnique
        uniquePolygons = [uniquePolygons; allPolygons(i, :)];
    end
end

% Reshape the uniquePolygons matrix to the desired 4-by-2-by-M format
allRejections = reshape(uniquePolygons', 4, 2, []);
end

%--------------------------------------------------------------------------
% mergeRejectedCandidatesCodegen is the codegen version of
% mergeRejectedCandidates
%--------------------------------------------------------------------------
function allRejections = mergeRejectedCandidatesCodegen(inRejections)

if ~isempty(inRejections)
    % To extract only the unique formats mentioned in the cell array.
    allPolygons = reshape(inRejections, 8, [])';

    num = size(allPolygons,1);
    uniqueFormats = allPolygons;
    flagInd = zeros(1,num);
    c = 1;

    % Count number of times each format appears and their indexes
    for i = 1:num

        item = allPolygons(i,:);
        ind = NaN(1,num);
        count = 1;
        for k = 1:num
            if isequal(item, allPolygons(k,:))
                ind(count) = k;
                count = count + 1;
            end
        end
        validmask = ~isnan(ind);

        % Sort the indices
        ind = sort(ind(validmask));

        if length(ind) > 1
            % If the format occurs more than once
            for j = 2:length(ind)
                if ~(any(flagInd(:) == ind(j)))
                    flagInd(c) = ind(j);
                    c = c+1;
                end
            end
        end
    end
    count = 0;
    for i = 1:num
        if ~(any(flagInd(:) == i))
            count = count + 1;
            uniqueFormats(count,:) = allPolygons(i,:);
        end
    end

    uniquePolygons = coder.nullcopy(zeros(count,8));
    for i = 1:count
        uniquePolygons(i,:) = uniqueFormats(i,:);
    end
    % Reshape the uniquePolygons matrix to the desired 4-by-2-by-M format
    allRejections = reshape(uniquePolygons', 4, 2, []);
else
    allRejections = zeros(0,0,'like',inRejections);
end
end

%--------------------------------------------------------------------------
% removeValidDetections removes any valid detection from the rejected
% candidate. This is especially required when searching across marker
% families as there may be markers rejected in one family but detected in
% another family.
%
% Input: rejections - 4-by-2-by-M matrix, detections - 4-by-2-by-N matrix
% Output: validRejections - 4-by-2-by-P matrix
%--------------------------------------------------------------------------
function rejections = removeValidDetections(rejections, detections)

invalidIndices = [];

% Loop through each rejection
for i = 1:size(rejections,3)
    rejectionCenter = mean(rejections(:, :, i));

    % Check if the current rejection is in detections
    for j = 1:size(detections,3)
        detectionCenter = mean(detections(:, :, j));

        % Compare the current rejection with each detection
        if isequal(rejectionCenter, detectionCenter)
            % This can be equal only if the polygons are true
            % duplicates as the vertex coordinates are floating point
            % numbers. This cannot be equal for concentric polygons due
            % to floating point tolerances.
            invalidIndices(end+1) = i;
            break;
        end
    end
end

rejections(:,:,invalidIndices) = [];
end

%--------------------------------------------------------------------------
% isSimMode - check if simulation mode or codegen mode
%--------------------------------------------------------------------------
function out = isSimMode()
out = isempty(coder.target);
end
