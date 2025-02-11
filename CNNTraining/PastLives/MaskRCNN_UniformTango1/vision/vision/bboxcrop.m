function [bboxB,indices] = bboxcrop(bboxA,win,varargin)
%BBOXCROP Crop bounding boxes.
%   bboxB = BBOXCROP(bboxA,win) crops bounding boxes in bboxA using the
%   cropping window, win. bboxA is an M1-by-N matrix defining M1 bounding
%   boxes. bboxB is an M2-by-N matrix defining M2 bounding boxes, where M2
%   <= M1. For rectangular bboxA inputs, the cropping window, win, must be
%   a four-element vector, [x y width height], or an
%   images.spatialref.Rectangle object. For cuboid bboxA inputs, the
%   cropping window, win, must be a six-element vector, [x y z width height
%   depth] or an images.spatialref.Cuboid object. For a rectangular
%   cropping window, the output bounding box positions are relative to
%   the location of the cropping window. 
% 
%   This function supports 2-D and 3-D bounding boxes which includes
%   axis-aligned rectangle, rotated rectangle and cuboid.
%
%   [...,indices] = BBOXCROP(bboxA,win) also returns a vector of indices
%   indicating which bounding boxes in bboxA are within the cropping
%   window.
%
%   [...] = BBOXCROP(...,Name,Value) specifies additional name-value pair
%   arguments described below:
%
%   'OverlapThreshold'   Specify a positive overlap threshold that is less
%                        than or equal to 1. The amount of overlap between
%                        bboxA and the cropping window is defined as
%                        area(bboxA intersect win) / area(bboxA). If the
%                        overlap is above the threshold, boxes in bboxA are
%                        clipped to the cropping window border. Otherwise,
%                        they are discarded. Note that lowering the
%                        threshold may discard parts of objects.
%
%                        Default: 1
%
%   Class Support
%   -------------
%   bboxA can be any nonsparse numeric type. The class of bboxB is the same
%   as the class of bboxA. win can be any numeric type, an
%   images.spatialref.Rectangle object, or an images.spatialref.Cuboid
%   object.
%
%   Example: Center crop image and associated bounding boxes.
%   ---------------------------------------------------------
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
%   % Create center cropping window.
%   targetSize = [256 256];
%   win = centerCropWindow2d(size(I),targetSize);
%   
%   % Center crop image.
%   [r,c] = deal(win.YLimits(1):win.YLimits(2),win.XLimits(1):win.XLimits(2));
%   J = I(r,c,:);
%   
%   % Center crop boxes and labels. Boxes outside the cropping window are
%   % removed.
%   [bboxB,indices] = bboxcrop(bboxA,win);
%   labelsB = labelsA(indices);
%   
%   % Display the results.
%   figure
%   I = insertObjectAnnotation(I,'Rectangle',bboxA,labelsA);
%   J = insertObjectAnnotation(J,'Rectangle',bboxB,labelsB);
%   imshowpair(I,J,'montage')   
%   
%   See also bboxresize, bboxwarp, imcrop, centerCropWindow2d, 
%            randomCropWindow2d, images.spatialref.Rectangle.

% Copyright 2019-2023 The MathWorks, Inc.

params = iParseInputs(bboxA, win, varargin{:});

bboxA = iManageEmptyInputs(bboxA);

[bboxB,valid] = iCropWorldCoordinates(bboxA,win,params);

% Return indices bounding boxes from bboxA that are retained in bboxB.
indices = reshape(find(valid),[],1);

end

%--------------------------------------------------------------------------
function [bboxB,valid] = iCropWorldCoordinates(bboxA,win,params)

numCoords = size(bboxA,2);
if numCoords == 4   

    % Convert xywh to minmax format for cropping.
    bboxCorners = vision.internal.bbox.xywh2minmax(bboxA);

    win = iConvertXYWHToMinMax(win);

    [bboxB, valid] = vision.internal.bbox.crop(...
        bboxCorners, win, params.OverlapThreshold);

    % Convert minmax to xywh
    bboxB = [bboxB(:,1:2) bboxB(:,3:4) - bboxB(:,[1 2])];

    bboxB = iPositionBoxRelativeToCroppingWindow(bboxB, win);

elseif numCoords == 5
    [X,Y] = vision.internal.bbox.bbox2poly(bboxA);
    
    % Valid boxes are those where all of the vertices are completely in
    % bounds.
    win = iManageWindowFormat(win);
    valid = all(win.contains(X,Y),1);
    bboxB = bboxA(valid,:);

    bboxB = iPositionBoxRelativeToCroppingWindow(bboxB, win);

elseif numCoords == 9
    [X,Y,Z] = vision.internal.bbox.bbox2poly(bboxA);
    
    % Valid boxes are boxes where all of the vertices are completely in
    % bounds.
    win = iManageWindowFormat(win);
    valid = all(win.contains(X,Y,Z),1);
    bboxB = bboxA(valid,:);

    % N.B. cuboids are not repositioned w.r.t to the cropping window.
    % Cuboids are frequently used with point cloud data and it is not
    % common to change the location of each point during a "cropping"
    % operation. 
else
    assert(false,'Unexpected box format');
end


end

%--------------------------------------------------------------------------
function bbox = iManageWindowFormat(bbox)

if isnumeric(bbox) 
    if numel(bbox) == 4
        xLimits = [bbox(1),bbox(1)+bbox(3)];
        yLimits = [bbox(2),bbox(2)+bbox(4)];
        bbox = images.spatialref.Rectangle(xLimits,yLimits);
    elseif numel(bbox) == 6
        xLimits = [bbox(1),bbox(1)+bbox(4)];
        yLimits = [bbox(2),bbox(2)+bbox(5)];
        zLimits = [bbox(3),bbox(3)+bbox(6)];
        bbox = images.spatialref.Cuboid(xLimits,yLimits,zLimits);
    end
end

end

%--------------------------------------------------------------------------
function bbox = iConvertXYWHToMinMax(bbox)
if isa(bbox,'images.spatialref.Rectangle')
    bbox = [bbox.XLimits(1) bbox.YLimits(1) bbox.XLimits(2) bbox.YLimits(2)];
else
    bbox(:,3:4) = bbox(:,1:2) + bbox(:,3:4);
end
end

%--------------------------------------------------------------------------
function params = iParseInputs(bboxA,win,varargin)

p = inputParser;
p.addRequired('bboxA',@(x)vision.internal.inputValidation.checkBBox(x,mfilename,'bboxA',1));
p.addRequired('win',@iCheckCropWindow);
p.addParameter('OverlapThreshold',1,@iCheckOverlapThreshold);

p.parse(bboxA,win,varargin{:})

userInput = p.Results;

params.OverlapThreshold = double(userInput.OverlapThreshold);

iCheckDimensionalityAgreement(bboxA,win);

if size(bboxA,2) == 5 || size(bboxA,2) == 9
    % Verify that OverlapThreshold is not less than 1 for rotated
    % rectangles and cuboids.
    if params.OverlapThreshold < 1
        error(message('vision:bbox:rotatedBoxOverlapThreshold'));
    end
end



end

%--------------------------------------------------------------------------
function iCheckDimensionalityAgreement(bboxA,win)

isRectangle = (size(bboxA,2) == 4) || (size(bboxA,2) == 5);
isCuboid = size(bboxA,2) == 9;

if isRectangle
    if ~( (numel(win) == 4) || isa(win,'images.spatialref.Rectangle') )
        error(message('vision:bbox:dimensionaltiyMustAgree'));
    end
elseif isCuboid
    if ~( (numel(win) == 6) || isa(win,'images.spatialref.Cuboid') )
        error(message('vision:bbox:dimensionaltiyMustAgree'));
    end
elseif isempty(bboxA)
    % No-op
else
   assert(false,'Unexpected box format'); 
end


end

%--------------------------------------------------------------------------
function iCheckCropWindow(win)

validateattributes(win,{'numeric','images.spatialref.Rectangle','images.spatialref.Cuboid'},...
    {},mfilename,'win',2);

if isnumeric(win)

    validateattributes(win,{'numeric'},...
        {'row','real','finite','nonsparse'},...
        mfilename,'win',2);
    
    % Numeric objects are not supported.
    vision.internal.inputValidation.validateNotObject(win,'vision','win');
    
    if (length(win) ~= 4) && (length(win) ~= 6)
        error(message('vision:bbox:invalidCroppingWindowSize'));
    end
    
    
else
    validateattributes(win,{'images.spatialref.Rectangle','images.spatialref.Cuboid'},...
        {'scalar'},mfilename,'win',2); 
    
    iAssertLimitsAreFinite(win.XLimits);
    iAssertLimitsAreFinite(win.YLimits);
    if isa(win,'images.spatialref.Cuboid')
        iAssertLimitsAreFinite(win.ZLimits);
    end
    
end
end

%--------------------------------------------------------------------------
function bbox = iPositionBoxRelativeToCroppingWindow(bbox,win)

% Position bounding boxes relative to the cropping window origin. The
% origin of the cropping window is the same as the intrinsic image
% coordinate system: (0.5,0.5).
if ~isnumeric(win)
    win = [win.XLimits(1) win.YLimits(1)];
end

bbox(:,1:2) = bbox(:,1:2) - cast(win(1:2),'like',bbox) + 0.5;
end

%--------------------------------------------------------------------------
function iCheckOverlapThreshold(x)

validateattributes(x,{'numeric'},...
    {'scalar','positive','<=',1,'real','finite','nonsparse'},...
    mfilename,'OverlapThreshold');

% Numeric objects are not supported.
vision.internal.inputValidation.validateNotObject(x,'vision','OverlapThreshold');

end

%--------------------------------------------------------------------------
function iAssertLimitsAreFinite(limits)
if ~all(isfinite(limits))
    error(message('vision:bbox:RectangleLimitsMustBeFinite'));
end
end

%--------------------------------------------------------------------------
function X = iManageEmptyInputs(X)
if isempty(X) && (size(X,2) == 0)
    X = reshape(X,0,4); 
end
end