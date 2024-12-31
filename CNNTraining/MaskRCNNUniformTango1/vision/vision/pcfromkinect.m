function ptCloud = pcfromkinect(varargin)
 
% Copyright 2015-2023 The MathWorks, Inc.

[depthDevice, depthImage, isDepthOnly, colorImage, isDepthCentric, isVersionOne] = ...
                                            validateAndParseInputs(varargin{:});

if isDepthOnly
    xyzPoints = vision.internal.visionKinectDepthToSkeleton(depthDevice, depthImage);    
    invalidIndex = find(depthImage(:)==0);
else    
    [xyzPoints, alignedFlippedImage] = ...
        vision.internal.visionKinectColorToSkeleton(depthDevice, ...
                        depthImage, colorImage, isDepthCentric, isVersionOne);
    if isVersionOne
        invalidIndex = find(xyzPoints(:, :, 3)==0);
    else
        invalidIndex = find(xyzPoints(:, :, 3)==0 | isinf(xyzPoints(:, :, 3)));
    end
end

szImg = size(xyzPoints, 1) * size(xyzPoints, 2);
xyzPoints(invalidIndex)         = NaN;
xyzPoints(invalidIndex+szImg)   = NaN;
xyzPoints(invalidIndex+szImg*2) = NaN;

% Flip along X and Y axis to match the CVST coordinate system conventions
xyzPoints        = fliplr(xyzPoints);
xyzPoints(:,:,1) = -xyzPoints(:,:,1);
xyzPoints(:,:,2) = -xyzPoints(:,:,2);

if isDepthOnly
    ptCloud = pointCloud(xyzPoints);
else
    ptCloud = pointCloud(xyzPoints, 'Color', alignedFlippedImage);
end

%==========================================================================
%   Parameter validation
%==========================================================================
function [depthDevice, depthImage, isDepthOnly, colorImage, ...
    isDepthCentric, isVersionOne] = validateAndParseInputs(varargin)
% Validate and parse inputs
persistent depthParser colorParser;

isDepthOnly = (length(varargin) < 3);
if isDepthOnly 
    if isempty(depthParser)
        depthParser = inputParser;
        depthParser.CaseSensitive = false;
        depthParser.addRequired('DepthDevice', @validateDepthDevice)
        depthParser.addRequired('DepthImage', @(x)validateattributes(x, {'uint16'}, ...
                                {'real','nonsparse','2d','nonempty'}));
        parser = depthParser;
    else
        parser = depthParser;
    end
else
    if isempty(colorParser)
        colorParser = inputParser;
        colorParser.CaseSensitive = false;
        colorParser.addRequired('DepthDevice', @validateDepthDevice)
        colorParser.addRequired('DepthImage', @(x)validateattributes(x, {'uint16'}, ...
                                {'real','nonsparse','2d','nonempty'}));
        colorParser.addRequired('ColorImage', @(x)validateattributes(x, {'uint8'}, ...
                            {'real','nonsparse','nonempty','size',[NaN,NaN,3]}));
        colorParser.addOptional('Alignment', 'colorCentric', @validateAlignmentString);
        parser = colorParser;
    else
        parser = colorParser;
    end
end

parser.parse(varargin{:});

depthDevice = parser.Results.DepthDevice;
depthImage  = parser.Results.DepthImage;

% Get object info
info = imaqhwinfo(depthDevice);
isVersionOne = true;
if ~isempty(strfind(info.DeviceName, 'V2'))
    isVersionOne = false;
end

if ~isDepthOnly
    colorImage = parser.Results.ColorImage;

    % Validate the resolution of the input
    % The size of two input images must be the same for V1
    % The resolutions for V2 are fixed, so we omit checking here.
    if isVersionOne
        if size(depthImage,1)~=size(colorImage,1)||size(depthImage,2)~=size(colorImage,2)
            error(message('vision:pointcloud:mismatchDepthToColor'));
        end
    end
    
    isDepthCentric = false;
    if strncmpi(parser.Results.Alignment,'d', 1)
        isDepthCentric = true;
    end
else
    colorImage = uint8.empty;
    isDepthCentric = true;
end

%==========================================================================
%   Validate Depth Device
%==========================================================================
function tf = validateDepthDevice(value)
% Validate the object class of the video device
if ~isa(value, 'videoinput') && ~isa(value, 'imaq.VideoDevice')
    error(message('vision:pointcloud:invalidDepthDevice'));
end

info = imaqhwinfo(value);
if isempty(strfind(info.DeviceName, 'Kinect')) || isempty(strfind(info.DeviceName, 'Depth'))
    error(message('vision:pointcloud:invalidDepthDevice'));
end

tf = true;

%==========================================================================
%   Validate Alignment String
%==========================================================================
function tf = validateAlignmentString(value)
% Validate the alignment string
validatestring(value, {'colorCentric','depthCentric'});
tf = true;