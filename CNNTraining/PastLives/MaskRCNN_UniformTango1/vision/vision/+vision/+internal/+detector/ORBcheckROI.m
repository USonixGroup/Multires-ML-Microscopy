function r = ORBcheckROI(roi, imageSize)
% ORBcheckROI Check the attributes and values of ROI
% r = checkROI(roi, imageSize) returns true if ROI has integer values and
% is inside the image with size specified in imageSize. Also checks whether
% the width and height is >= 63.

% Copyright 2018-2019 The MathWorks, Inc.

%#codegen
%#ok<*EMCA>

% roi must be 1-by-4 numeric vector
validateattributes(roi, {'numeric'}, ...
    {'real', 'nonsparse', 'nonnan', 'finite', 'numel',4,'vector'},...
    'checkROI', 'ROI');

if ~isempty(roi)
    
    % rounds floats and casts to int32 to avoid saturation of smaller integer types.
    roi = vision.internal.detector.roundAndCastToInt32(roi);
    
    % width and height must be >= 63
    coder.internal.errorIf(roi(3) < 63 || roi(4) < 63, ...
        'vision:validation:invalidROIWidthHeightORB');% 63 = 2*31 +1 (31 is the patchSize)
    
    % roi must be fully contained within I
    coder.internal.errorIf(roi(1) < 1 || roi(2) < 1 ...
        || roi(1)+roi(3) > imageSize(2)+1 ...
        || roi(2)+roi(4) > imageSize(1)+1, ...
        'vision:validation:invalidROIValue');
end
r = true;
