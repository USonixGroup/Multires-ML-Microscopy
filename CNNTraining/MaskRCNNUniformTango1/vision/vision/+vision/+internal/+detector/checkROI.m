function r = checkROI(roi, imageSize, varargin)
% checkROI Check the attributes and values of ROI
% r = checkROI(roi, imageSize) returns true if ROI has integer values and
% is inside the image with size specified in imageSize. Also checks whether
% the width, height and depth is >= zero. ROI must be a 1-by-4 or a 1-by-6
% element vector

%#codegen
%#ok<*EMCA>

% Copyright 2018-2019 The MathWorks, Inc.

if isempty(varargin)
    is3D = false;
else
    is3D = coder.const(varargin{1});
end

if ~isempty(roi)
    
    if is3D
        
        % roi must be 1-by-4 numeric vector or 1-by-6 numeric vector
        validateattributes(roi, {'numeric'}, ...
            {'real', 'nonsparse', 'nonnan', 'finite', 'numel', 6, 'vector'},...
            'checkROI', 'ROI');

        % rounds floats and casts to int32 to avoid saturation of smaller integer types.
        roi = vision.internal.detector.roundAndCastToInt32(roi);

        numDims = 3;
        ROIStartLocation = cell(1,numDims);
        ROIEndLocation = cell(1,numDims);
        ROIExtent = cell(1,numDims);

        for idx = coder.unroll(1:numDims)
            ROIStartLocation{idx} = roi(idx);
            ROIExtent{idx} = roi(numDims+idx);
            ROIEndLocation{idx} = ROIStartLocation{idx} + ROIExtent{idx}-1;
        end

        for idx = coder.unroll(1:numDims)
            coder.internal.errorIf(ROIExtent{idx} < 0, ...
                'vision:validation:invalidROIWidthHeightDepth');
        end

        % Flip the first two dimensions so that width is followed
        % by height and rest of the dimensions as-is
        imageSize(1:2) = flip(imageSize(1:2));
        % roi must be fully contained within I
        for idx = coder.unroll(1:numDims)
            coder.internal.errorIf(ROIStartLocation{idx} < 1  ...
                || ROIEndLocation{idx} > imageSize(idx), ...
                'vision:validation:invalidROIValue');
        end
        
    else
       
        % roi must be 1-by-4 numeric vector
        validateattributes(roi, {'numeric'}, ...
            {'real', 'nonsparse', 'nonnan', 'finite', 'numel', 4, 'vector'},...
            'checkROI', 'ROI');

        % rounds floats and casts to int32 to avoid saturation of smaller integer types.
        roi = vision.internal.detector.roundAndCastToInt32(roi);    

        % width and height must be >= 0
        coder.internal.errorIf(roi(3) < 0 || roi(4) < 0, ...
            'vision:validation:invalidROIWidthHeight');

        % roi must be fully contained within I
        coder.internal.errorIf(roi(1) < 1 || roi(2) < 1 ...
            || roi(1)+roi(3) > imageSize(2)+1 ...
            || roi(2)+roi(4) > imageSize(1)+1, ...
            'vision:validation:invalidROIValue');
        
    end
end
r = true;
