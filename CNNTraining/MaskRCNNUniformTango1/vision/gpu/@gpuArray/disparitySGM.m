function disparityMap = disparitySGM(I1, I2, varargin)
% disparitySGM Compute disparity map using Semi-Global Matching.
%   disparityMap = disparitySGM(I1, I2) returns the disparity map for a
%   pair of stereo images, I1 and I2. I1 and I2 must have the same size and
%   must be rectified such that the corresponding points are located on the
%   same rows. This rectification can be performed using the
%   rectifyStereoImages function. The returned disparity map has the same
%   size as I1 and I2. Unreliable disparity values are set to
%   nan('single').
%
%   disparityMap = disparitySGM(..., Name, Value) specifies additional
%   name-value pairs described below:
%
%   'DisparityRange'       A two-element vector, [MinDisparity
%                          MaxDisparity], defining the range of disparity.
%                          MinDisparity and MaxDisparity must be integers,
%                          within the range (-image width, image width), 
%                          their difference must be divisible by 8 and the
%                          maximum difference is 128.
%
%                          Default: [0 128]
%
%   'UniquenessThreshold'  A non-negative integer defining the minimum
%                          value of uniqueness. If a pixel is less unique,
%                          the disparity computed for it is less reliable.
%                          Increasing this parameter will result in marking
%                          more pixels unreliable. You can set this
%                          parameter to 0 to disable it. Typical range is
%                          from 5 to 15.
%
%                          Default: 15
%
%   Class Support
%   -------------
%   All inputs must be real, finite and nonsparse. I1 and I2 must be gray
%   scale gpuArrays, same class and must be uint8, uint16, int16, single or
%   double. The output will be gpuArray of class single.
%
%   Example
%   -------
%   % Load the rectified images.
%   I1 = imread('scene_left.png');
%   I2 = imread('scene_right.png');
%
%   % Show the stereo anaglyph. You can view the image in 3-D using
%   % red-cyan stereo glasses.
%   figure
%   imshow(stereoAnaglyph(I1, I2))
%   title('Red-cyan composite view of the stereo images')
%
%   % Convert images to grayscale.
%   I1 = rgb2gray(I1);
%   I2 = rgb2gray(I2);
%
%   % Move images to the GPU.
%   I1 = gpuArray(I1);
%   I2 = gpuArray(I2);
%
%   % Compute the disparity map.
%   disparityRange = [-6 10];
%   disparityMap = disparitySGM(I1, I2, 'DisparityRange', disparityRange);
%
%   % Show the disparity map. For better visualization use the disparity
%   % range as the display range for imshow.
%   figure
%   imshow(disparityMap, disparityRange)
%   title('Disparity Map')
%   colormap jet
%   colorbar
%
%   See also disparityBM, rectifyStereoImages, reconstructScene, gpuArray,
%            estimateCameraParameters, estimateUncalibratedRectification

%   Copyright 2018-2019 The MathWorks, Inc.
%
%   References:
%   -----------
%   [1] I. Ernst and H. Hirschmuller. Mutual Information Based Semi-Global
%       Stereo Matching on the GPU. In Advances in Visual Computing, pages
%       228-239. Springer, 2008.
%
%   [2] R. Spangenberg et al. Weighted Semi-Global Matching and
%       Center-Symmetric Census Transform for Robust Driver Assistance.
%       Computer Analysis of Images and Patterns, pages 34-41, 2013.

%--------------------------------------------------------------------------
% Handle the case if Name-Value pairs are gpuArray Objects
%--------------------------------------------------------------------------
if(nargin > 2)
    % Make sure all parameters are on the host.
    [varargin{:}] = gather(varargin{:});
    
    % If the data is on the CPU, dispatch to the CPU version. You don't
    % want to run on the GPU unless the data is on the GPU.
    if ~isa(I1, 'gpuArray') && ~isa(I2, 'gpuArray')
        disparityMap = disparitySGM(I1, I2, varargin{:});
        return
    end
end

%--------------------------------------------------------------------------
% Parse the inputs
%--------------------------------------------------------------------------
r = parseInputs(I1, I2, varargin{:});

% parameters
% ----------------------------
optSGM.MinDisparity         = int32(r.DisparityRange(1));
optSGM.NumberOfDisparities  = int32(r.DisparityRange(2) - r.DisparityRange(1));
optSGM.UniquenessThreshold  = int32(r.UniquenessThreshold);

%--------------------------------------------------------------------------
% Other parameters which are not exposed in the main interface
%--------------------------------------------------------------------------

% Number of directions used to traverse for calculating Smoothing costs
% Maximum value allowed is 8 and minimum value is 1
optSGM.Directions           = int32(5);
% Penalty for small disparity changes, intended to detect slanted and
% curved surfaces
optSGM.Penalty1             = int32(15);
% Penalty for larger disparity discontinuities, smooths the results and
% makes abrupt changes difficult
optSGM.Penalty2             = int32(200);

%--------------------------------------------------------------------------
% Compute disparity
%--------------------------------------------------------------------------

I1U8 = im2uint8(I1);
I2U8 = im2uint8(I2);

deviceSpec = gpuDevice;
freeMem = deviceSpec.AvailableMemory;
% Determine whether high or low GPU memory to be used based on
% free memory. High memory option is faster than low memory.
% Two input images require ImagesSize bytes = 2*ImageSize
% Census transform requires ImageSize * 4(bytes) * 2(Images)
% Each Direction requires ImageSize * NumberOfDisparities
% Initial cost requires ImageSize * NumberOfDisparities
% 100MB buffer memory required for GPU code calculations
ReqMaxMem = size(I1U8,1) * size(I1U8,2)*( 10 + (double(optSGM.Directions) + 1) * double(optSGM.NumberOfDisparities))+ 100*10^6;
if(freeMem > ReqMaxMem)
    optSGM.Memory = 1; % High GPU Memory used
else
    optSGM.Memory = 0; % Low GPU Memory used
end

disparityGpu = visiongpuDisparitySGM(transpose(I1U8), transpose(I2U8), optSGM);
disparityMap = transpose(disparityGpu);

%--------------------------------------------------------------------------
% Parse and check inputs
%--------------------------------------------------------------------------
function r = parseInputs(I1, I2, varargin)

if( islogical(I1)||islogical(I2) )
    error( message('vision:disparity:logicalImagesNotSupported') );
end

vision.internal.inputValidation.validateImagePairForDisparity(I1, I2, 'I1', 'I2', 'grayscale');

imageSize = size(I1);
r = parseOptionalInputs(imageSize, varargin{:});

%--------------------------------------------------------------------------
function r = parseOptionalInputs(imageSize, varargin)

r = parseOptionalInputs_sim(imageSize, getDefaultParameters(),...
    varargin{:});

%--------------------------------------------------------------------------
function r = parseOptionalInputs_sim(imageSize, defaults, varargin)

if(nargin == 2)
    r = defaults;
    %----------------------------------------------------------------------
    % Validate all PV pairs
    %----------------------------------------------------------------------
    checkDisparityRange(r.DisparityRange, imageSize);
    checkUniquenessThreshold(r.UniquenessThreshold);
else
    % Instantiate an input parser
    parser = inputParser;
    parser.FunctionName = 'disparitySGM';
    
    %----------------------------------------------------------------------
    % Specify the optional parameter value pairs
    %----------------------------------------------------------------------
    parser.addParameter('DisparityRange', defaults.DisparityRange, ...
        @(x)(checkDisparityRange(x, imageSize)));
    parser.addParameter('UniquenessThreshold', defaults.UniquenessThreshold, ...
        @(x)(checkUniquenessThreshold(x)));
    
    %----------------------------------------------------------------------
    % Parse and check the optional parameters
    %----------------------------------------------------------------------
    parser.parse(varargin{:});
    r = parser.Results;
end

%--------------------------------------------------------------------------
function defaults = getDefaultParameters()

defaults = struct(...
    'DisparityRange',   [0 128], ...
    'UniquenessThreshold',   15 );

%--------------------------------------------------------------------------
function r = checkDisparityRange(value, imageSize)

imgWidth = imageSize(2);
validateattributes(value, {'numeric'}, ...
    {'real', 'nonsparse', 'nonnan', 'finite', 'integer', 'size', [1,2]},...
    mfilename, 'DisparityRange');

if( value(2) <= value(1) )
    error( message('vision:disparity:invalidMaxDisparityRange') );
elseif( mod(value(2) - value(1), 8) ~= 0 )
    error( message('vision:disparity:invalidDisparityRangeSGM') );
elseif( (value(2) - value(1)) > 128 )
    error( message('vision:disparity:DisparityRangeExceedLimitSGM') );
end

if( value(1) < 0 && (value(2) - value(1)) >= imgWidth )
    error( message('vision:disparity:DisparityRangeExceedImageWidth', ...
        imgWidth) );
elseif( value(1) < 0 && abs(value(1)) >= imgWidth )
    error( message('vision:disparity:absoluteMinDisparityRangeExceedImageWidthSGM', ...
        imgWidth) );
elseif( value(2) >= imgWidth )
    error( message('vision:disparity:maxDisparityRangeExceedImageWidthSGM', ...
        imgWidth) );
end

r = 1;

%--------------------------------------------------------------------------
function r = checkUniquenessThreshold(value)

validateattributes(value, {'numeric'}, ...
    {'real', 'nonsparse', 'scalar', 'nonnan', 'finite', 'integer', 'nonnegative'},...
    mfilename, 'UniquenessThreshold');

r = 1;
