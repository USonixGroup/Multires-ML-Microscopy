function bboxB = bboxresize(bboxA,scale)
%BBOXRESIZE Resize bounding boxes.
%   bboxB = BBOXRESIZE(bboxA,scale) returns bounding boxes that are scale
%   times the size of bounding boxes in bboxA. bboxA is a M1-by-N matrix 
%   defining M1 bounding boxes. scale is a scalar or vector specifying the 
%   resize scale factors. When scale is a scalar, the same scale factor is 
%   applied to all dimensions of the bounding boxes in bboxA. When scale is 
%   a vector, scale(1) resizes the box height and scale(2) resizes the box 
%   width.
%
%   This function supports 2-D and 3-D bounding boxes which includes
%   axis-aligned rectangle, rotated rectangle and cuboid.
%
%   Class Support
%   -------------
%   bboxA can be any nonsparse numeric type. The class of bboxB is the same
%   as the class of bboxA when bboxB contains floating point data.
%   Otherwise, the class of bboxB is single. scale can be any numeric
%   class.
%
%   Example: Resize image and associated bounding boxes.
%   ----------------------------------------------------
%   % Read image.
%   I = imread('peppers.png');
%
%   % Define bounding boxes and labels.
%   bboxA = [
%       410 230 100 90
%       186 78  80  60
%       ];
%
%   labelsA = [
%       "garlic"
%       "onion"
%       ];
%
%   % Resize image and bounding boxes.
%   scale = 1.5;
%   J = imresize(I,scale);
%   bboxB = bboxresize(bboxA,scale);
%
%   % Display the results.
%   figure
%   I = insertObjectAnnotation(I,'Rectangle',bboxA,labelsA);
%   J = insertObjectAnnotation(J,'Rectangle',bboxB,labelsA);
%   imshowpair(I,J,'montage')
%
%   See also bboxwarp, bboxcrop, imresize.

% Copyright 2019-2023 The MathWorks, Inc.

params = iParseInputs(bboxA,scale);

scale = params.Scale;

bboxA = iManageEmptyInputs(bboxA);

% Cast boxes to floating point for processing.
bboxA = iCastToFloat(bboxA);

bboxB = resizeCoordinates(bboxA,scale);

end

%--------------------------------------------------------------------------
function bboxB = resizeCoordinates(bboxA,scale)
    if any(size(bboxA,2) == 4)
        bboxB = bboxA;
        bboxB(:,[1 3]) = scale(2) * bboxA(:,[1 3]);
        bboxB(:,[2 4]) = scale(1) * bboxA(:,[2 4]);
    elseif any(size(bboxA,2) == 5)
        if ~isempty(bboxA)
            % Convert rotated bbox to vertices for scaling.
            bboxPointsTemp = bbox2points(bboxA);

            % Remap bboxPointsTemp to be an M-by-8 matrix, where M is the number of
            % bounding boxes and each row is specified as [x1 x2 x3 x4 y1 y2 y3 y4].
            bboxPoints = reshape(bboxPointsTemp,[8,size(bboxA,1)])';

            % Scale the vertices based on scaling factor input.
            bboxPoints(:,1:4) = scale(2) * bboxPoints(:,1:4);
            bboxPoints(:,5:8) = scale(1) * bboxPoints(:,5:8);

            % Convert scaled vertices back to bbox format.
            w = max(hypot(bboxPoints(:,2:3) - bboxPoints(:,[1 4]), ...
                       bboxPoints(:,6:7) - bboxPoints(:,[5 8])),[],2);
            h = max(hypot(bboxPoints(:,3:4) - bboxPoints(:,[2 1]), ...
                       bboxPoints(:,7:8) - bboxPoints(:,[6 5])),[],2);

            % Rescale center bbox coordinates.
            xCtr = scale(2) * bboxA(:,1);
            yCtr = scale(1) * bboxA(:,2);

            bboxB = [xCtr yCtr w h bboxA(:,5)];
        else
            bboxB = bboxA;
        end
    else
        bboxB = bboxA;
        bboxB(:,[1 4]) = scale(2) * bboxA(:,[1 4]);
        bboxB(:,[2 5]) = scale(1) * bboxA(:,[2 5]);
        bboxB(:,[3 6]) = scale(3) * bboxA(:,[3 6]);
    end
end

%--------------------------------------------------------------------------
function params = iParseInputs(bboxA,scale)

p = inputParser;
p.addRequired('bboxA',@(x)vision.internal.inputValidation.checkBBox(x,mfilename,'bboxA',1));
p.addRequired('scale',@iCheckScale);

p.parse(bboxA,scale);

params.InputPrecision = class(bboxA);
params.Scale = iManageScalarScale(bboxA,scale);

end

%--------------------------------------------------------------------------
function scaleOut = iManageScalarScale(bboxA,scale)

if isscalar(scale)
    if size(bboxA,2) == 5
        scaleOut = repelem(scale,1,2);
    else
        scaleOut = repelem(scale,1,3);
    end
else
    scaleOut = scale;
end
end

%--------------------------------------------------------------------------
function iCheckScale(s)

validateattributes(s,{'numeric'},...
    {'nonempty','row','real','finite','positive','nonsparse'},...
    mfilename,'scale',2);

% Numeric objects are not supported.
vision.internal.inputValidation.validateNotObject(s,'vision','scale');

end


%--------------------------------------------------------------------------
function x = iCastToFloat(x)
if ~isfloat(x)
    % Cast non-floats to single for processing.
    x = single(x);
end
end

%--------------------------------------------------------------------------
function X = iManageEmptyInputs(X)
if isempty(X) && (size(X,2) == 0)
    X = reshape(X,0,4);
end
end
