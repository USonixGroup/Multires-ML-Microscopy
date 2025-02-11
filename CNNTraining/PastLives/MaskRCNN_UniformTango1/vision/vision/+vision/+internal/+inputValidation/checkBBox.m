function checkBBox(bbox,fname,name,pos)
% Check input bounding box formats. Support formats and validation are:
%
%   * axis-aligned rectangles must be M-by-4. The width and height values
%     must be positive.
%
%   * rectangles must be M-by-5 and all dimensions (width and
%     height) must be positive.
%
%   * cuboids bounding boxes must be M-by-9 and all dimensions (width,
%     height, depth) must be positive.
%
%
% N.B. This functions treats empty boxes ([]) as valid.

% Copyright 2019-2021 The MathWorks, Inc.

    
if ~isempty(bbox)
    if (size(bbox,2) ~= 4) && (size(bbox,2) ~=5) && (size(bbox,2) ~=9)
        error(message('vision:bbox:invalidBoxFormat'));
    end
    
    validateattributes(bbox,{'numeric'},...
        {'real','finite','nonsparse'},...
        fname,name,pos);
    
    % Do not allow numeric objects (e.g. gpuArray).
    vision.internal.inputValidation.validateNotObject(bbox,'vision',name);

    % Check specific types of bounding box formats.
    rectangle        = size(bbox,2) == 4;
    rotatedRectangle = size(bbox,2) == 5;
    cuboid           = size(bbox,2) == 9;
           
    if rectangle || rotatedRectangle
        % Width and height must be positive.
        dims = bbox(:,[3 4]);
        if anyNonPositive(dims(:))
            error(message('vision:visionlib:invalidBboxHeightWidth'));
        end
        
    elseif cuboid
        % Width, height, and depth must be positive. The cuboid center and
        % rotation angles can be any value.
        dims = bbox(:,[4 5 6]);
        if anyNonPositive(dims(:))
            error(message('vision:visionlib:invalidCuboidDims'));
        end
    end
end

%--------------------------------------------------------------------------
function tf = anyNonPositive(x)
tf = any(x <= 0);




