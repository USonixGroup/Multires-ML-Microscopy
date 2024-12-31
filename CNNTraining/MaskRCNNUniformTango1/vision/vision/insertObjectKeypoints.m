function RGB = insertObjectKeypoints(I, keypoints, namedArgs)
%

% Copyright 2023 The MathWorks, Inc.
%#codegen
arguments
    I (:,:,:) {mustBeNonsparse, mustBeNonempty, mustBeReal,checkImage,...
        validateNotObject(I,'I')}
    keypoints (:,:,:) {mustBeNonsparse,mustBeNumeric,mustBeReal,...
        mustBePositive,mustBeNonempty,mustBeFinite,validateNotObject(keypoints,'keypoints')}
end

arguments

    namedArgs.Connections {mustBeNumeric,mustBeReal,mustBeInteger,...
        mustBePositive,mustBeNonsparse, ivalidateConnections}= [];
    namedArgs.KeypointVisibility ...
        {ivalidatedKeypointVisibilityFlag(namedArgs.KeypointVisibility,keypoints)}= true;
    namedArgs.KeypointColor ...
        {ivalidatedKeypointColor(namedArgs.KeypointColor,keypoints)}= [0.850 0.325 0.098];
    namedArgs.KeypointLabel = [];
    namedArgs.KeypointSize{mustBeNumeric,mustBeInteger,mustBeReal,...
    mustBeNonsparse,mustBePositive,mustBeFinite,ivalidatedKeypointSize} = 2;
    namedArgs.Opacity (1,1) {mustBeNumeric, mustBeReal, mustBeNonempty,...
        mustBeFinite,mustBeGreaterThanOrEqual(namedArgs.Opacity,0),...
        mustBeLessThanOrEqual(namedArgs.Opacity,1)} = 1;
    namedArgs.ConnectionColor = [0 0.447 0.741];

    namedArgs.LineWidth (1,1) {mustBeNumeric, mustBeFinite, mustBeInteger,...
        mustBeNonsparse, mustBePositive, ...
        vision.internal.inputValidation.validateNotObject(namedArgs.LineWidth,'vision','LineWidth')} = 1;
    namedArgs.TextBoxColor = [1 1 0];
    namedArgs.TextBoxOpacity(1,1){mustBeNumeric, mustBeReal, mustBeNonempty,...
        mustBeNonsparse, mustBeFinite,mustBeGreaterThanOrEqual(namedArgs.TextBoxOpacity,0),...
        mustBeLessThanOrEqual(namedArgs.TextBoxOpacity,1)} = 0.6;
    namedArgs.FontColor = [0 0 0];
    namedArgs.Font =  "LucidaSansRegular";
    namedArgs.FontSize{validateFontSize} = 12;
end

numObjects = size(keypoints,3);

if size(namedArgs.KeypointVisibility(:),1)==1
 numKeypoins = size(keypoints,1) * numObjects;
else
 numKeypoins = nnz(namedArgs.KeypointVisibility);
end

if ~isempty(namedArgs.KeypointLabel)
    validateAndManageLabels(namedArgs.KeypointLabel,keypoints);
    if isSimMode()
        if isstring(namedArgs.KeypointLabel) || iscategorical(namedArgs.KeypointLabel)
            labels = convertStringsToChars(string(namedArgs.KeypointLabel));
            finalLabels = cell(numKeypoins,1);
        end
    else
        % Convert string and categorical types to cell array type
        % for code generation.
        labels = cellstr(namedArgs.KeypointLabel);
        if coder.const(isstring(namedArgs.KeypointLabel) || iscategorical(namedArgs.KeypointLabel))
            finalLabels = cellstr(repmat(labels,numObjects,1));
        end
    end
end

finalKeypoints = coder.nullcopy(zeros(numKeypoins,3));
out = cell(1,numObjects);
connectionVisFlag = cell(1,numObjects);
connectionVisibilityFlag = cell(1,numObjects);
numOfSkeletonLines = 0;
if ~isempty(namedArgs.Connections)
    for id = 1:numObjects
        if size(namedArgs.KeypointVisibility(:),1)==1
            numOfSkeletonLines = size(namedArgs.Connections,1)*numObjects;
            break;
        else
            logicalFilter = namedArgs.KeypointVisibility(:,id);
            connectionVisFlag = true(size(namedArgs.Connections(:,1),1),1);
            keypointIndex = find(~logicalFilter);
            jointConnectionsX = namedArgs.Connections(:,1);
            jointConnectionsY = namedArgs.Connections(:,2);
            idxx = find(ismember(jointConnectionsX,keypointIndex));
            idxy = find(ismember(jointConnectionsY,keypointIndex));
            out{id} = union(idxx',idxy');
            connectionVisFlag(out{id}) = false;
            connectionVisibilityFlag{id} = connectionVisFlag;
            numOfSkeletonLines = numOfSkeletonLines + size(jointConnectionsX,1)-size(out{id},2);
        end
    end
end
visibleObjCount = 1;
visibleSkeletonLineCount = 1;
if ~isempty(namedArgs.Connections)
    skeletonLines = zeros(numOfSkeletonLines,4);
    if coder.const(~isstring(namedArgs.ConnectionColor) && size(namedArgs.ConnectionColor,1)~=1)
        connectionColors = coder.nullcopy(zeros(numOfSkeletonLines,3));
    end
end

if coder.const(~isstring(namedArgs.KeypointColor) && size(namedArgs.KeypointColor,1)~=1)
    keypointColors = coder.nullcopy(zeros(numKeypoins,3));
end

if size(namedArgs.KeypointVisibility(:),1)==1
    if coder.const(~isstring(namedArgs.KeypointColor) && size(namedArgs.KeypointColor,1)~=1)
        keypointColors  = repmat(namedArgs.KeypointColor,numObjects,1);
    end
    if ~isempty(namedArgs.Connections)
        if coder.const(~isstring(namedArgs.ConnectionColor) && size(namedArgs.ConnectionColor,1)~=1)
            connectionColors = repmat(namedArgs.ConnectionColor,numObjects,1);
        end
        skeletonLines = [keypoints(namedArgs.Connections(:,1),:,:)...
            keypoints(namedArgs.Connections(:,2),:,:)];
        skeletonLines = permute(skeletonLines,[1,3,2]);
        skeletonLines = reshape(skeletonLines,[],4);
    end
    keypoints = permute(keypoints,[1,3,2]);
    finalKeypoints = reshape(keypoints,[],2);
    radius = repmat(namedArgs.KeypointSize,numKeypoins,1);
    finalKeypoints = [finalKeypoints,radius];
    if ~isempty(namedArgs.KeypointLabel)
        finalLabels= repmat(labels,numObjects,1);
    end
else
    % Iterate over number of objects present in an image.
    for idx = 1:numObjects
        currentKeypoints = keypoints(:,:,idx);
        if size(namedArgs.KeypointVisibility(:),1)~=1
            % Using KeypointVisibibility flag to extract the visible
            % skeleton connection lines.
            logicalFilter = namedArgs.KeypointVisibility(:,idx);
            visibleJointConnection = namedArgs.Connections;
            visibleJointConnection(out{idx},:) = [];
        end
    
        currentObjVisibleCount = nnz(logicalFilter);
    
        if ~isempty(namedArgs.Connections)
            % visible skeleton connection lines.
            currentSkeletonLines = [currentKeypoints(visibleJointConnection(:,1),:)...
                currentKeypoints(visibleJointConnection(:,2),:)];
            numCurrentSkeletonLines = size(currentSkeletonLines,1);
            if coder.const(~isstring(namedArgs.ConnectionColor) && size(namedArgs.ConnectionColor,1)~=1)
                connectionColors(visibleSkeletonLineCount:visibleSkeletonLineCount+numCurrentSkeletonLines-1,:) = namedArgs.ConnectionColor(connectionVisibilityFlag{idx},:);
            end
            skeletonLines(visibleSkeletonLineCount:visibleSkeletonLineCount+numCurrentSkeletonLines-1,:) = currentSkeletonLines;
            visibleSkeletonLineCount = visibleSkeletonLineCount+numCurrentSkeletonLines;
        end
    
        % Extract visible keypoints
        if coder.const(~isstring(namedArgs.KeypointColor) && size(namedArgs.KeypointColor,1)~=1)
            keypointColors(visibleObjCount:visibleObjCount+currentObjVisibleCount-1,:) = namedArgs.KeypointColor(logicalFilter,:);
        end
        finalKeypoints(visibleObjCount:visibleObjCount+currentObjVisibleCount-1,:) = [currentKeypoints(logicalFilter,:),...
            namedArgs.KeypointSize*ones(currentObjVisibleCount,1)];
    
        if ~isempty(namedArgs.KeypointLabel)
            if isSimMode()
                finalLabels(visibleObjCount:visibleObjCount+currentObjVisibleCount-1) = labels(logicalFilter);
            else
                fLabels = labels{logicalFilter};
                for lc=1:size(logicalFilter,1)
                    finalLabels{visibleObjCount+lc-1} = fLabels;
                end
            end
        end
        visibleObjCount = visibleObjCount+currentObjVisibleCount;
    
    end
end

if ~isempty(namedArgs.Connections)
    % Circumference point: This code reduce the skeleton line length by
    % substracting the sin and cos factor of the radius of the filled
    % circle. This calculation is helpful to calculate the circumference
    % point of circle where skeleton line touch the circle.
    for j= 1:size(skeletonLines,1)
        angleDgree = atand(abs(skeletonLines(j,4)-skeletonLines(j,2)+eps)...
            /abs(skeletonLines(j,3)-skeletonLines(j,1))+eps);
        sinFactor = namedArgs.KeypointSize*sind(angleDgree);
        cosFactor = namedArgs.KeypointSize*cosd(angleDgree);
        if skeletonLines(j,2)<skeletonLines(j,4)
            skeletonLines(j,2) = skeletonLines(j,2) + sinFactor;
            skeletonLines(j,4) = skeletonLines(j,4) - sinFactor;
        else
            skeletonLines(j,2) = skeletonLines(j,2) - sinFactor;
            skeletonLines(j,4) = skeletonLines(j,4) + sinFactor;
        end
        if skeletonLines(j,3)<skeletonLines(j,1)
            skeletonLines(j,1) = skeletonLines(j,1) - cosFactor;
            skeletonLines(j,3) = skeletonLines(j,3) + cosFactor;
        else
            skeletonLines(j,1) = skeletonLines(j,1) + cosFactor;
            skeletonLines(j,3) = skeletonLines(j,3) - cosFactor;
        end
    end
end

% Draw the keypoints using filled circle.
if coder.const(isstring(namedArgs.KeypointColor) || size(namedArgs.KeypointColor,1)==1)
    RGB = insertShape(I,"filled-circle",finalKeypoints,...
        ShapeColor=namedArgs.KeypointColor, ...
        Opacity=namedArgs.Opacity);
else
    RGB = insertShape(I,"filled-circle",finalKeypoints, ...
        ShapeColor=keypointColors, ...
        Opacity=namedArgs.Opacity);
end

% Draw the skeleton connection line.
if ~isempty(namedArgs.Connections)
    if coder.const(isstring(namedArgs.ConnectionColor) || size(namedArgs.ConnectionColor,1)==1)
        RGB = insertShape(RGB,"line",skeletonLines, ...
            ShapeColor=namedArgs.ConnectionColor, ...
            LineWidth=namedArgs.LineWidth);
    else
        RGB = insertShape(RGB,"line",skeletonLines, ...
            ShapeColor=connectionColors, ...
            LineWidth=namedArgs.LineWidth );
    end
end

% Insert keypoint labels
if ~isempty(namedArgs.KeypointLabel)
    % Calculate the text box position so that it will not overlap with
    % keypoint circle.
    textLocAndWidth = getTextLocAndWidth(finalKeypoints, namedArgs.LineWidth);
    textPosition    = textLocAndWidth(:,1:2);
    shapeWidth  = textLocAndWidth(:,3);
    shapeHeight = textLocAndWidth(:,4);
    RGB = insertText(RGB,textPosition,finalLabels, ...
        BoxOpacity=namedArgs.TextBoxOpacity, ...
        FontSize=namedArgs.FontSize ,...
        Font=namedArgs.Font,...
        FontColor=namedArgs.FontColor,...
        TextBoxColor=namedArgs.TextBoxColor, ...
        AnchorPoint="RightBottom",...
        ShapeWidth=shapeWidth, ...
        ShapeHeight=shapeHeight);
end
end
%--------------------------------------------------------------------------
function textLocAndWidth = getTextLocAndWidth(position, lineWidth)
% This function computes the text location and the width of the keypoint
% circle. Text location:
%   * It is the bottom-right corner (x,y) of the label text box
%   * Since label text box is placed above the shape (i.e., bottom border
%     of the label text box touches the top-most point of the shape), (x,
%     y) is computed as follows: For keypoint circle, (x, y) is the
%     bottom-right corner of the rectangle that encloses the shape (circle)
%   * For 'circle' shape, Width of label text box = diameter of circle

position = round(position);
halfLineWidth = floor(lineWidth/2);
lineWidthAdj = 2*halfLineWidth; % adjusted line width

% [x y width] = [center_x-radius center_y-radius-1 2*radius+1]
textLocAndWidth = [position(:,1)-position(:,3) - halfLineWidth...
    position(:,2)-position(:,3) - 1 - halfLineWidth...
    2*position(:,3) + (lineWidthAdj+1), ...
    2*position(:,3) + (lineWidthAdj+1)];
end
%--------------------------------------------------------------------------
function checkImage(I)
% Validate input image

% No objects allowed.
vision.internal.inputValidation.validateNotObject(I, 'vision', 'I');

validateattributes(I,{'uint8', 'uint16', 'int16', 'double', 'single'}, ...
    {'real','nonsparse'}, mfilename, 'I', 1)
% input image must be 2d or 3d (with 3 planes)
coder.internal.errorIf(((ndims(I) > 3) || ((size(I,3) ~= 1) && (size(I,3) ~= 3))),...
          "vision:dims:imageNot2DorRGB");

coder.internal.assert(coder.internal.isConst(size(I,3)), ...
    'vision:insertObjectKeypoints:image3rdDimFixed');
end
%--------------------------------------------------------------------------
function ivalidateConnections(connections)
if ~isempty(connections)
    coder.internal.errorIf((size(connections,2)~=2 || numel(size(connections))~=2),...
                   'vision:hrnetObjectKeypoint:invalidConnections');
end
end
%--------------------------------------------------------------------------
function ivalidatedKeypointSize(keypointSize)

coder.internal.errorIf((~(size(keypointSize,1)==1 && size(keypointSize,2)==1)),...
  'vision:insertObjectKeypoints:keypointSizeMustBeSingleNumber');
end
%--------------------------------------------------------------------------
function ivalidatedKeypointVisibilityFlag(flag,keypoints)

coder.internal.errorIf((~islogical(flag)),...
    'vision:insertObjectKeypoints:mustBeLogical');

if numel(flag)~=1
coder.internal.errorIf((numel(size(flag))~=2 || (size(keypoints,1)~= size(flag,1) ...
    || size(keypoints,3)~= size(flag,2))),...
               'vision:insertObjectKeypoints:mustBeSameDimOfKeypoints');
end

end
%--------------------------------------------------------------------------
function validateAndManageLabels(label,keypoints)

coder.internal.errorIf(~(isstring(label) || iscategorical(label)),...
        'vision:insertObjectKeypoints:labelsMustBeCategoricalAndString');

coder.internal.errorIf((size(label,1)~=size(keypoints,1)||size(label,2)~=1),...
    'vision:insertObjectKeypoints:labelsAndKeypointsMustBeSameSpatialDim');

end
%--------------------------------------------------------------------------
function validateFontSize(FontSize)
% Validate 'FontSize'
vision.internal.inputValidation.validateNotObject(FontSize,'vision','FontSize');
validateattributes(FontSize, {'numeric'}, ...
    {'nonempty', 'integer', 'nonsparse', 'scalar', '>=', 8, '<=', 72}, ...
    mfilename, 'FontSize');
end
%--------------------------------------------------------------------------
function TF = validateNotObject(x,name)
vision.internal.inputValidation.validateNotObject(x, 'vision', name);
TF = true;
end
%--------------------------------------------------------------------------
function ivalidatedKeypointColor(keypointColor, keypoints)
if ~isstring(keypointColor) && size(keypointColor,1)~=1
coder.internal.errorIf((size(keypointColor,1)~=size(keypoints,1)&&size(keypointColor,2)==3),...
    'vision:insertObjectKeypoints:keypointColorAndKeypointsMustBeSameSpatialDim');
end
end
%--------------------------------------------------------------------------
function flag = isSimMode()
flag = coder.target('MATLAB');
end
