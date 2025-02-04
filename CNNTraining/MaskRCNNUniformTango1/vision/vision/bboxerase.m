function [bboxB,indices] = bboxerase(bboxA,win,params)
%bboxerase Remove bounding boxes within a region of interest.
%   bboxB = bboxerase(bboxA,win) removes bounding box in bboxA using the
%   erasing window, win. bboxA is an M1-by-4 matrix defining M1 bounding 
%   boxes. bboxB is an M2-by-4 matrix defining M2 bounding boxes, where
%   M2 <= M1. The erasing window, win, must be a four-element vector,
%   [x y width height], or an images.spatialref.Rectangle object.
%
%   [..., indices] = bboxerase(bboxA,win) also returns a vector of indices
%   indicating bounding boxes in bboxA which are partially or completely
%   outside the erasing window which are not removed.
%
%   [...] = bboxerase(...,Name,Value) specifies additional name-value pair
%   arguments as described below:
%
%   'EraseThreshold'    Specify a positive overlap ratio threshold that is 
%                       less than or equal to 1. The amount of overlap between
%                       bboxA and erasing window, win, is defined as the 
%                       area(bboxA intersect win) / area(bboxA). If the
%                       overlap is at or above the threshold, the function 
%                       removes bounding boxes in bboxA which are around 
%                       the referencing window.
%                     
%                       Default: 0.8
%
%   Example: Apply random erasing augmentation and update corresponding bounding boxes.
%   ----------------------------------------------------------------------------------
%
%   % Read an image.
%   I = imread('peppers.png');
%
%   % Define bounding boxes and labels.
%   bboxA = [
%       410 230 100 90
%       186 78  80  60
%       ];
% 
%   labelsA = [
%        "garlic"
%        "onion"
%      ];
%
%   % Select a region whose area is between 2% and 13% of the area of the 
%   % input image, with an aspect ratio between 1:10 and 35:100.
%   scale = [0.02,0.13];
%   dimensionRatio = [1,10;35,100];
%
%   % Randomly select a 2D window region from the input image.
%   win = randomWindow2d(size(I),'Scale',scale,'DimensionRatio',dimensionRatio);
%
%   % Apply random erase on Image.
%   J = imerase(I,win);
%
%   % Erase bounding boxes above a threshold of 0.8.
%   [bboxB,indices] = bboxerase(bboxA,win,'EraseThreshold',0.8);
%   labelsB = labelsA(indices);
%  
%   % Display the original and augmented image.
%   annotatedI = insertObjectAnnotation(I,'Rectangle',bboxA,labelsA);
%   annotatedJ = insertObjectAnnotation(J,'Rectangle',bboxB,labelsB);
%
%   figure
%   montage({annotatedI,annotatedJ})
%
%   See also bboxcrop, imerase, randomWindow2d, imref2d, 
%   affineOutputView, insertObjectAnnotation.
  
%   Copyright 2020-2021 The MathWorks, Inc.

    arguments
        bboxA {mustBeNumeric,mustBeFinite,mustBeReal,mustBeValidSize(bboxA)}
        win {mustBeNumericOrObject(win),mustBeValidDatatype(win),mustBeValidWinSize(win)}
        params.EraseThreshold (1,1) double {mustBeNumeric(params.EraseThreshold),mustBeFinite(params.EraseThreshold),...
            mustBeNonNan(params.EraseThreshold),mustBePositive(params.EraseThreshold),mustBeReal(params.EraseThreshold),mustBeInRange(params.EraseThreshold,0,1)} = 0.8    
    end
   
    % Handle bounding box.
    [bboxB,indices] = iApplyCutOutBbox(bboxA,win,params);
end

%% Supporting function.

function [bboxes,indices] = iApplyCutOutBbox(bboxes,coord,params) 
% Apply randomErase bbox function handling bounding boxes.
 
    % Removing bbox when erased area contains detection box and overlap ratio
    % is more than Threshold.
    if(isa(coord,'images.spatialref.Rectangle'))
       coord = [coord.XLimits(1) coord.YLimits(1) diff(coord.XLimits) diff(coord.YLimits)];
    end
    
    indices = 1:size(bboxes,1);
    if ~isempty(coord)
        coord = vision.internal.bbox.xywh2minmax(coord);
        bboxes = vision.internal.bbox.xywh2minmax(bboxes);
        coord = cast(coord,'like',bboxes);
        [~,ind] = vision.internal.bbox.crop(bboxes,coord,params.EraseThreshold);
        
        % Constructing indices array.
        indices = 1:size(bboxes,1);
        % Retaining indices which are not erased.
        indices = indices(~ind);
        bboxes = bboxes(indices,:);
        bboxes(:,3:4) = bboxes(:,3:4) - bboxes(:,1:2);
    end
end


function mustBeNumericOrObject(args)
    validateattributes(args,{'numeric','images.spatialref.Rectangle'},...
    {},mfilename,'win',2);
    if(isa(args,'images.spatialref.Rectangle'))
        validateattributes(args,{'images.spatialref.Rectangle'},...
        {'size',[1,1]},mfilename,'win',2);
    end
end

function mustBeValidDatatype(args)
    if(isa(args,'numeric'))
        mustBeFinite(args);
        mustBeReal(args);
    end
end

function mustBeValidSize(args)
    if(isa(args,'numeric') && ~isequal(size(args,2),4))
        error(message('vision:bbox:InvalidSize'))
    end

    % Width and height must be positive.
    dims = args(:,[3 4]);
    if any(dims(:) <= 0)
        error(message('vision:visionlib:invalidBboxHeightWidth'));
    end
end

function mustBeValidWinSize(args)
    if isa(args,'numeric')
        if ~isequal(size(args),[1,4])
            error(message('vision:bbox:InvalidSize'))
        end
    
        % Width and height must be positive.
        dims = args(:,[3 4]);
        if any(dims(:) <= 0)
            error(message('vision:visionlib:invalidBboxHeightWidth'));
        end
    end
end
