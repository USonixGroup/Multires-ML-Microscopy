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
%   This function supports processing on a GPU. See the <a
%   href="matlab:doc('disparitySGM')">documentation</a>
%   for more details.
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
%   All inputs must be real, finite and non-sparse. I1 and I2 must be gray
%   scale, same class and must be uint8, uint16, int16, single or double.
%   The output will be of class single.
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
%   See also disparityBM, rectifyStereoImages, reconstructScene,
%            estimateCameraParameters, estimateUncalibratedRectification

%   Copyright 2018-2023 The MathWorks, Inc.
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

%#codegen
%#ok<*EMCA>

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
% Indexing will ensure I1 and I2 is a 2d matrix to coder during compile time
I1U8 = im2uint8(I1(:,:));
I2U8 = im2uint8(I2(:,:));

if isSimMode()
    % Transpose the inputs and output as MATLAB takes column major
    disparity_Map = visiondisparitySGM(transpose(I1U8), ...
        transpose(I2U8), optSGM);
    disparityMap = transpose(disparity_Map);
else
    if coder.gpu.internal.isGpuEnabled
        disparityMap = gpudisparitySGMCompute(I1U8, I2U8, optSGM);
    else
        disparityMap = vision.internal.buildable.disparitySGMBuildable.disparitySGMCompute(...
            I1U8, I2U8, optSGM);
    end
end

%--------------------------------------------------------------------------
% Parse and check inputs
%--------------------------------------------------------------------------
function r = parseInputs(I1, I2, varargin)

% Error out if logical images are inputs
if isSimMode()
    if( islogical(I1)||islogical(I2) )
        error( message('vision:disparity:logicalImagesNotSupported') );
    end
else
    errIf(islogical(I1)||islogical(I2),...
        'vision:disparity:logicalImagesNotSupported');
end

% Validate images like same class & Same size
vision.internal.inputValidation.validateImagePairForDisparity(I1, I2, 'I1', 'I2', ... 
    'grayscale');

imageSize = size(I1);
r = parseOptionalInputs(imageSize, varargin{:});

%--------------------------------------------------------------------------
function r = parseOptionalInputs(imageSize, varargin)

if isSimMode()
    % Call to simulation PV pairs parser
    r = parseOptionalInputs_sim(imageSize, getDefaultParameters(),...
        varargin{:});
else
    % Call to Codegen PV pairs parser
    r = parseOptionalInputs_cg(imageSize, varargin{:});
end

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
function r = parseOptionalInputs_cg( imageSize, varargin)

% Optional Name-Value pair: 2 pairs (see help section)
defaults = getDefaultParameters();
defaultsNoVal = getDefaultParametersNoVal();
properties    = getEmlParserProperties();

if (nargin == 1) % only imageSize
    r = defaults;
else
    pvPairStartIdx = 1;
    optarg = eml_parse_parameter_inputs(defaultsNoVal, properties, ...
        varargin{pvPairStartIdx:end});
    
    DisparityRange = (eml_get_parameter_value( optarg.DisparityRange, ...
        defaults.DisparityRange, varargin{pvPairStartIdx:end}));
    UniquenessThreshold = (eml_get_parameter_value( optarg.UniquenessThreshold, ...
        defaults.UniquenessThreshold, varargin{pvPairStartIdx:end}));
    
    r.DisparityRange = DisparityRange;
    r.UniquenessThreshold = UniquenessThreshold;
end

checkDisparityRange(r.DisparityRange, imageSize);
checkUniquenessThreshold(r.UniquenessThreshold);

%--------------------------------------------------------------------------
function defaults = getDefaultParameters()

defaults = struct(...
    'DisparityRange', [0 128], ...
    'UniquenessThreshold', 15);

%--------------------------------------------------------------------------
function defaultsNoVal = getDefaultParametersNoVal()

defaultsNoVal = struct(...
    'DisparityRange', uint32(0),...
    'UniquenessThreshold', uint32(0));

%--------------------------------------------------------------------------
function properties = getEmlParserProperties()

properties = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand',    true, ...
    'PartialMatching', false);

%--------------------------------------------------------------------------
function r = checkDisparityRange(value, imageSize)

imgWidth = imageSize(2);
validateattributes(value, {'numeric'}, ...
    {'real', 'nonsparse', 'nonnan', 'finite', 'integer', 'size', [1,2]},...
    mfilename, 'DisparityRange');

if isSimMode()
    
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
else
    
    errIf(value(2) <= value(1), 'vision:disparity:invalidMaxDisparityRange');
    errIf(mod(value(2) - value(1), 8) ~= 0, 'vision:disparity:invalidDisparityRangeSGM');
    errIf((value(2) - value(1)) > 128, 'vision:disparity:DisparityRangeExceedLimitSGM');
    errIf(value(1) < 0 && (value(2) - value(1)) >= imgWidth, ...
        'vision:disparity:DisparityRangeExceedImageWidth', sprintf('%d', int32(imgWidth)));
    errIf(value(1) < 0 && abs(value(1)) >= imgWidth, ...
        'vision:disparity:absoluteMinDisparityRangeExceedImageWidthSGM', sprintf('%d', int32(imgWidth)));
    errIf(value(2) >= imgWidth, ...
        'vision:disparity:maxDisparityRangeExceedImageWidthSGM', sprintf('%d', int32(imgWidth)));
end
r = 1;

%--------------------------------------------------------------------------
function r = checkUniquenessThreshold(value)

validateattributes(value, {'numeric'}, ...
    {'real', 'nonsparse', 'scalar', 'nonnan', 'finite', 'integer', 'nonnegative'},...
    mfilename, 'UniquenessThreshold');

r = 1;

%--------------------------------------------------------------------------
function flag = isSimMode()

flag = isempty(coder.target);

%--------------------------------------------------------------------------
function errIf(condition, msgID, varargin)

coder.internal.errorIf(condition, msgID, varargin{:});
