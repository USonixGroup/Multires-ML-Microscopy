%checkImagePoints Check validity of image points from calibration images
%
%  See also estimateFisheyeParameters, estimateCameraParameters

% Copyright 2017-2020 The MathWorks, Inc.
function checkImagePoints(imagePoints, mfilename, varargin)

allownan = false;
if nargin > 2
    allownan = varargin{1};
end

if allownan
    attributes = {'nonsparse', 'ncols', 2};
else
    attributes = {'nonnan', 'finite', 'nonsparse', 'ncols', 2};
end

validateattributes(imagePoints, {'single','double'}, attributes, ...
        mfilename, 'imagePoints');

if ndims(imagePoints) == 4
    validateattributes(imagePoints, {'single','double'}, {'size', [nan, nan, nan, 2]}, ...
        mfilename, 'imagePoints');
end

if ndims(imagePoints) > 4
    error(message('vision:calibrate:imagePoints3Dor4D'));
end

if size(imagePoints, 3) < 2
    error(message('vision:calibrate:minNumPatterns'));
end

minNumPoints = 4;
if size(imagePoints, 1) < minNumPoints
    error(message('vision:calibrate:minNumImagePoints', minNumPoints-1));
end
