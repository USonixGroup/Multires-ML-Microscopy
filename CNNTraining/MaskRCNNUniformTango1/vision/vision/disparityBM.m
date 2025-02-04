function disparityMap = disparityBM(I1, I2, varargin)
% disparityBM Computes disparity map using block matching.
%   disparityMap = disparityBM(I1, I2) returns the disparity map for a pair
%   of stereo images, I1 and I2, using block matching algorithm. I1 and I2
%   must have the same size and must be rectified such that the
%   corresponding points are located on the same rows. This rectification
%   can be performed using the rectifyStereoImages function. The returned
%   disparity map has the same size as I1 and I2. Unreliable disparity
%   values are set to nan('single').
%
%   disparityMap = disparityBM(..., Name, Value) specifies additional
%   name-value pairs described below:
%
%   'DisparityRange'       A two-element vector, [MinDisparity
%                          MaxDisparity], defining the range of disparity.
%                          MinDisparity and MaxDisparity must be integers,
%                          within the range (-image width, image width) and
%                          their difference must be divisible by 16.
%
%                          Default: [0 64]
%
%   'BlockSize'            A positive odd integer that specifies the width
%                          of each square block of pixels used for
%                          comparing I1 and I2. BlockSize must be in the
%                          range [5 255].
%
%                          Default: 15
%
%   'ContrastThreshold'    A scalar value, defining the acceptable range of
%                          contrast values. Increasing this parameter
%                          results in fewer pixels being marked as
%                          unreliable. The ContrastThreshold must be in the
%                          range (0 1].
%
%                          Default: 0.5
%
%   'UniquenessThreshold'  A positive integer defining the minimum
%                          value of uniqueness. If a pixel is less unique,
%                          the disparity computed for it is less reliable.
%                          Increasing this parameter will result in marking
%                          more pixels unreliable. Set this
%                          parameter to 0 to disable it.
%
%                          Default: 15
%
%   'DistanceThreshold'    A positive integer defining the maximum
%                          distance for left-right checking. Increasing
%                          this parameter results in fewer pixels being
%                          marked as unreliable. Set this
%                          parameter to an empty matrix [] to disable it.
%                          The DistanceThreshold must be less than image
%                          width.
%
%                          Default: [] (disabled)
%
%   'TextureThreshold'     A scalar value, defining the minimum texture. If
%                          a block of pixels is less textured, the computed
%                          disparity is less reliable. Increasing this
%                          parameter results in more pixels being marked as
%                          unreliable. Set this parameter to 0 to disable
%                          it. TextureThreshold must be in the range [0 1).
%
%                          Default: 0.0002
%
%   Class Support
%   -------------
%   All inputs must be real, finite and nonsparse. I1 and I2 must be gray
%   scale, same class and must be uint8, uint16, int16, single, or double.
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
%   range = [0 16];
%   disparityMap = disparityBM(I1, I2, 'DisparityRange', range);
%
%   % Show the disparity map. For better visualization use the disparity
%   % range as the display range for imshow.
%   figure
%   imshow(disparityMap, range)
%   title('Disparity Map')
%   colormap jet
%   colorbar
%
%   See also rectifyStereoImages, reconstructScene, disparitySGM,
%            estimateCameraParameters, estimateUncalibratedRectification

%   Copyright 2017-2018 The MathWorks, Inc.
%
%   References:
%   -----------
%   [1] K. Konolige, "Small Vision Systems: Hardware and Implementation,"
%       Proceedings of the 8th International Symposium in Robotic Research,
%       pages 203-212, 1997.
%
%   [2] G. Bradski and A. Kaehler, "Learning OpenCV : Computer Vision with
%       the OpenCV Library," O'Reilly, Sebastopol, CA, 2008.

%#codegen
%#ok<*EMCA>

%--------------------------------------------------------------------------
% Parse the inputs
%--------------------------------------------------------------------------
r = parseInputs(I1, I2, varargin{:});

% BlockMatching method parameters
% -------------------------------

% preFilterCap minimum value allowed in OpenCV
if (r.ContrastThreshold < 0.016)
    r.ContrastThreshold = 0.016;
end
optBM.preFilterCap        = int32(floor(63 * r.ContrastThreshold));
optBM.SADWindowSize       = int32(r.BlockSize);
optBM.minDisparity        = int32(r.DisparityRange(1));
optBM.numberOfDisparities = int32(r.DisparityRange(2) - r.DisparityRange(1));
optBM.uniquenessRatio     = int32(r.UniquenessThreshold);

% parameters unique to block matching
optBM.textureThreshold    = int32(255 * r.TextureThreshold * r.BlockSize^2);

% OpenCV parameters that are not exposed as optional parameters
optBM.preFilterType       = int32(0); % Fixed to CV_STEREO_BM_NORMALIZED_RESPONSE
optBM.preFilterSize       = int32(15);
optBM.trySmallerWindows   = int32(0);

% OpenCV parameters that are not exposed as optional parameters

if isempty(r.DistanceThreshold)
    optBM.disp12MaxDiff       = int32(-1);
else
    optBM.disp12MaxDiff       = int32(r.DistanceThreshold);
end
%--------------------------------------------------------------------------
% Other OpenCV parameters which are not exposed in the main interface
%--------------------------------------------------------------------------

optBM.speckleWindowSize   = int32(0);
optBM.speckleRange        = int32(0);

%--------------------------------------------------------------------------
% Disparity pixel invalidation value
%--------------------------------------------------------------------------
optBM.invalidDisparityValue = NaN('single');

%--------------------------------------------------------------------------
% Compute disparity
%--------------------------------------------------------------------------
I1_u8 = im2uint8(I1);
I2_u8 = im2uint8(I2);

if isSimMode()
    disparityMap = ocvDisparityBM(I1_u8, I2_u8, optBM);
    
else
    disparityMap = vision.internal.buildable.disparityBMBuildable.disparityBM_compute(...
        I1_u8, I2_u8, optBM);
end

%==========================================================================
% Parse and check inputs
%==========================================================================
function r = parseInputs(I1, I2, varargin)
% Error out if logical images are inputs
if isSimMode()
    if(islogical(I1) || islogical(I2))
        error( message('vision:disparity:logicalImagesNotSupported') );
    end
else
    errIf(islogical(I1) || islogical(I2),...
        'vision:disparity:logicalImagesNotSupported');
end
vision.internal.inputValidation.validateImagePairForDisparity(I1, I2, 'I1', 'I2', 'grayscale');
imageSize = size(I1);
r = parseOptionalInputs(imageSize, varargin{:});

%==========================================================================
function r = parseOptionalInputs(imageSize, varargin)
if isSimMode()
    r = parseOptionalInputs_sim(imageSize, getDefaultParameters(),...
        varargin{:});
else
    r = parseOptionalInputs_cg(imageSize, varargin{:});
end

%==========================================================================
function r = parseOptionalInputs_cg(imageSize, varargin)

% Optional Name-Value pair: 6 pairs (see help section)
defaults = getDefaultParameters();
defaultsNoVal = getDefaultParametersNoVal();
properties    = getEmlParserProperties();

if nargin==1 % only imageSize
    r = defaults;
else
    pvPairStartIdx = 1;
    optarg = eml_parse_parameter_inputs(defaultsNoVal, properties, varargin{pvPairStartIdx:end});
    
    ContrastThreshold = (eml_get_parameter_value( ...
        optarg.ContrastThreshold, defaults.ContrastThreshold, varargin{pvPairStartIdx:end}));
    BlockSize = (eml_get_parameter_value( ...
        optarg.BlockSize, defaults.BlockSize, varargin{pvPairStartIdx:end}));
    DisparityRange = (eml_get_parameter_value( ...
        optarg.DisparityRange, defaults.DisparityRange, varargin{pvPairStartIdx:end}));
    TextureThreshold = (eml_get_parameter_value( ...
        optarg.TextureThreshold, defaults.TextureThreshold, varargin{pvPairStartIdx:end}));
    UniquenessThreshold = (eml_get_parameter_value( ...
        optarg.UniquenessThreshold, defaults.UniquenessThreshold, varargin{pvPairStartIdx:end}));
    DistanceThreshold = (eml_get_parameter_value( ...
        optarg.DistanceThreshold, defaults.DistanceThreshold, varargin{pvPairStartIdx:end}));
    r.ContrastThreshold = ContrastThreshold;
    r.BlockSize = BlockSize;
    r.DisparityRange = DisparityRange;
    r.TextureThreshold = TextureThreshold;
    r.UniquenessThreshold = UniquenessThreshold;
    r.DistanceThreshold = DistanceThreshold;
end
checkContrastThreshold(r.ContrastThreshold);
checkBlockSize(r.BlockSize, imageSize);
checkDisparityRange(r.DisparityRange, imageSize);
checkTextureThreshold(r.TextureThreshold);
checkUniquenessThreshold(r.UniquenessThreshold);
checkDistanceThreshold(r.DistanceThreshold, imageSize);

function r = parseOptionalInputs_sim(imageSize, defaults, varargin)

if(nargin == 2)
    r = defaults;
    checkContrastThreshold(r.ContrastThreshold);
    checkBlockSize(r.BlockSize, imageSize);
    checkDisparityRange(r.DisparityRange, imageSize);
    checkTextureThreshold(r.TextureThreshold);
    checkUniquenessThreshold(r.UniquenessThreshold);
    checkDistanceThreshold(r.DistanceThreshold, imageSize);
else
    % Instantiate an input parser
    parser = inputParser;
    parser.FunctionName = mfilename;
    %--------------------------------------------------------------------------
    % Specify the optional parameter value pairs
    %--------------------------------------------------------------------------
    parser.addParameter('ContrastThreshold', defaults.ContrastThreshold, ...
        @(x)(checkContrastThreshold(x)));
    parser.addParameter('BlockSize', defaults.BlockSize, ...
        @(x)(checkBlockSize(x, imageSize)));
    parser.addParameter('DisparityRange', defaults.DisparityRange, ...
        @(x)(checkDisparityRange(x, imageSize)));
    parser.addParameter('TextureThreshold', defaults.TextureThreshold, ...
        @(x)(checkTextureThreshold(x)));
    parser.addParameter('UniquenessThreshold', defaults.UniquenessThreshold, ...
        @(x)(checkUniquenessThreshold(x)));
    parser.addParameter('DistanceThreshold', [], ...
        @(x)(checkDistanceThreshold(x, imageSize)));
    %--------------------------------------------------------------------------
    % Parse the optional parameters
    %--------------------------------------------------------------------------
    parser.parse(varargin{:});
    
    r = parser.Results;
end

%==========================================================================
function defaults = getDefaultParameters()

defaults = struct(...
    'ContrastThreshold', 0.5, ...
    'BlockSize',   15, ...
    'DisparityRange',   [0 64], ...
    'TextureThreshold',   0.0002, ...
    'UniquenessThreshold',   15, ...
    'DistanceThreshold',  []); % unusual value

%==========================================================================
function defaultsNoVal = getDefaultParametersNoVal()

defaultsNoVal = struct(...
    'ContrastThreshold', uint32(0), ...
    'BlockSize',   uint32(0), ...
    'DisparityRange',   uint32(0), ...
    'TextureThreshold',   uint32(0), ...
    'UniquenessThreshold',   uint32(0), ...
    'DistanceThreshold',  uint32(0));

%==========================================================================
function properties = getEmlParserProperties()

properties = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand',    true, ...
    'PartialMatching', false);


%==========================================================================
function r = checkContrastThreshold(value)
validateattributes(value, {'numeric'}, ...
    {'real', 'nonsparse', 'scalar','nonnan', 'finite', 'nonnegative','>', 0, '<=', 1},...
    mfilename, 'ContrastThreshold');
r = 1;

%==========================================================================
function r = checkBlockSize(value, imageSize)
maxBlockSize = min([imageSize, 255]);
validateattributes(value, {'numeric'}, ...
    {'real', 'nonsparse', 'scalar','nonnan', 'finite', 'integer', 'nonnegative',...
    'odd', '>=', 5, '<=', maxBlockSize}, mfilename, 'BlockSize');

r = 1;

%==========================================================================
function r = checkDisparityRange(value, imageSize)
imgWidth = imageSize(2);
validateattributes(value, {'numeric'}, ...
    {'real', 'nonsparse', 'nonnan', 'finite', 'integer', 'size', [1,2], ...
    '>', -imgWidth, '<', imgWidth},...
    mfilename, 'DisparityRange');
if isSimMode()
    if(value(2) <= value(1))
        error(message('vision:disparity:invalidMaxDisparityRange'));
    elseif( mod(value(2) - value(1), 16) ~= 0 )
        error(message('vision:disparity:invalidDisparityRangeBM'));
    end
else
    errIf(value(2) <= value(1) , 'vision:disparity:invalidMaxDisparityRange');
    errIf(mod(value(2) - value(1), 16 ) ~= 0, 'vision:disparity:invalidDisparityRangeBM');
end
r = 1;

%==========================================================================
function r = checkTextureThreshold(value)
validateattributes(value, {'numeric'}, ...
    {'real', 'nonsparse', 'scalar','nonnan', 'finite', 'nonnegative', '<', 1},...
    mfilename, 'TextureThreshold');
r = 1;

%==========================================================================
function r = checkUniquenessThreshold(value)
validateattributes(value, {'numeric'}, ...
    {'real', 'nonsparse', 'scalar','nonnan' 'finite', 'integer', 'nonnegative'},...
    mfilename, 'UniquenessThreshold');
r = 1;

%==========================================================================
function r = checkDistanceThreshold(value, imageSize)
imgWidth = min(imageSize);
if ~isempty(value)
    validateattributes(value, {'numeric'}, ...
        {'real', 'nonsparse', 'scalar','nonnan', 'finite', 'integer',...
        'nonnegative', '<', imgWidth}, mfilename, 'DistanceThreshold');
end
r = 1;

%==========================================================================
function flag = isSimMode()

flag = isempty(coder.target);

%==========================================================================
function errIf(condition, msgID, varargin)

coder.internal.errorIf(condition, msgID, varargin{:});

