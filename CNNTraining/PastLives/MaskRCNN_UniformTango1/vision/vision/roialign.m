function Y = roialign(X, boxes, outputSize, varargin)
% roialign Non-quantized ROI pooling operation for Mask-CNN.
%
%   Y = roialign(X, boxes, outputSize) performs a pooling operation along
%   the spatial dimensions of the input X for each bounding box in BOXES and
%   returns output Y of the same fixed spatial size as OUTPUTSIZE.
%   The fixedsized outputs are computed by pooling the ROI into fixed size bins
%   without quantizing the grid points to the nearest pixel. The value at
%   each grid point is inferred using bilinear interpolation.
%
%   Y = roialign(__, Name, Value) specifies additional name-value pair arguments.
%   ROIALIGN accepts the following Name/Value pairs:
%
%   'ROIScale'            Ratio of scale of input feature map to that of the
%                         ROI coordinates. This specifies the factor used
%                         to scale input ROIs to the input feature map size.
%                         This value is a scalar.
%
%                         Default: 1.0
%
%   'SamplingRatio'       Number of samples in each pooled bin along height
%                         and width expressed as a 1x2 vector. The average
%                         of the sampled values is returned as the output
%                         value of each pooled bin.
%
%                         Default: 'auto'. This calculates adaptive number
%                         of samples as ceil(roiWidth/outputWidth) along the
%                         X axis and similarly for samples along Y axis.
%
%   Given an input X of size [H W C N], where C is the number of channels and
%   N is the number of observations, the size of output Y is
%   [outHeight outWidth C sum(M)], where outHeight and outWidth is the spatial
%   size specified by outputSize. M is a vector of length N and M(i) is the
%   number of ROIs associated with the i-th input feature map. This operation
%   is used in Mask-RCNN object detection networks.
%
%   Inputs
%   ------
%   X         - A 4-D formatted dlarray with 2 spatial dimensions, having a
%               data format of 'SSCB'.
%   boxes     - A 5xN numeric matrix holding N bounding boxes. Each bounding
%               box is formatted as [start_x; start_y; end_x; end_y; batchIdx].
%               The default BOXES coordinates are in the same coordinates
%               space as input X.
%   outputSize- Pooled output size, specified as a vector of two positive
%               integers [h w], where h is the height and w is the width.
%
%   Example
%   -------
%
%   X = dlarray(rand(10,10,3,2),"SSCB");
%
%   startXY = [2; 2];
%   endXY   = [4; 4];
%   batchIdx= 1;
%
%   rois = [startXY; endXY; batchIdx];
%
%   % Call roialign on a dlarray input to perform roipooling with a output
%   % size of 3x3
%   Y = roialign(X, rois, [3 3],'ROIScale', 2, 'SamplingRatio', [2 2]);
%
%   See also roiAlignLayer, roiMaxPooling2dLayer, nnet.cnn.layer.ROIAlignLayer.

%   Copyright 2020-2024 The MathWorks, Inc.
%#codegen

if isSimMode
    nameValueArgs = parseSimInputs(X, boxes, outputSize, varargin{:});
else
    nameValueArgs = parseCodInputs(X, boxes, outputSize, varargin{:});
end

batchIdx = boxes(5, :);
validateBatchIdx(batchIdx, X);

roiScale = nameValueArgs.ROIScale;
samplingRatioNew = nameValueArgs.SamplingRatio;

if(isequal(samplingRatioNew, 'auto'))
    samplingRatio = -1;
else
    samplingRatio = samplingRatioNew;
end

% Scale the boxes
boxesScaled = scaleBoxesToFeatureSpace( boxes, roiScale);

if isSimMode
    % Create the dlarray extension method
    func = vision.internal.dlarray.method.roiAlignMethod(outputSize, samplingRatio, size(X));

    Y = deep.internal.dlarray.extension.applyExtensionMethod(func, X, boxesScaled');
else
    pooled = extractdata(X);
    dlPooled = vision.internal.buildable.roiAlignForwardBuildable.roiAlignForward(pooled, ...
        boxesScaled, double(outputSize), samplingRatio);
    Y = dlarray(dlPooled, 'SSCB');
end

end

%--------------------------------------------------------------------------
function bboxes = scaleBoxesToFeatureSpace(bboxes, roiScale)
% Scales boxes to feature space. Input bboxes and scaled boxes
% are in [x1 y1 x2 y2] format.
if(roiScale == 1)
    return;
end
% Translate the bboxes to 0-index coordinate space and scale them.
% Translate the boxes back to 1-index coordinate space.
bboxes(1:4, :) = ((bboxes(1:4,:)-1).*roiScale)+1;

end

%--------------------------------------------------------------------------
function tf = validateInputDlArray (input, funcName, varname)
% Validate dlarray input
coder.internal.errorIf(~isdlarray(input),'vision:roiAlign:invalidInput');
coder.internal.errorIf((dims(input)~="SSCB"),'vision:roiAlign:invalidInput');

if isSimMode
    validateattributes(input, {'dlarray'}, ...
        {'nonempty', 'ndims', 4, 'real', 'nonsparse'}, ...
        funcName, varname);
else
    validateattributes(input, {'dlarray'}, ...
        {'nonempty', 'real', 'nonsparse'}, ...
        funcName, varname);
end

tf = true;
end

%--------------------------------------------------------------------------
function nameValueArgs = parseSimInputs(X, boxes, outputSize, nameValueArgs)
% Parse inputs for simulation
arguments
    X {validateInputDlArray(X, 'roialign.m','X')}
    boxes (5,:) {mustBeNonempty, mustBeNonsparse, mustBeReal, mustBeNumeric, mustBePositive, mustBeFinite};
    outputSize (1,2) {nnet.cnn.layer.ROIAlignLayer.validateOutputSize(outputSize,'roialign.m','OutputSize')};
    nameValueArgs.ROIScale (1,1) double {nnet.cnn.layer.ROIAlignLayer.validateROIScale(nameValueArgs.ROIScale,'roialign.m','ROIScale')} = 1;
    nameValueArgs.SamplingRatio {nnet.cnn.layer.ROIAlignLayer.validateSamplingRatio(nameValueArgs.SamplingRatio,'roialign.m','SamplingRatio')} = 'auto';
end
end

%--------------------------------------------------------------------------
function nameValueArgs = parseCodInputs(X, boxes, outputSize, nameValueArgs)
% Parse inputs for codegen
arguments
    X {validateInputDlArray(X, 'roialign.m', 'X')}
    boxes (5,:) {mustBeNonempty, mustBeNonsparse, mustBeReal, mustBeNumeric, mustBePositive, mustBeFinite}
    outputSize (1,2) {validateOutputSize(outputSize,'roialign.m','OutputSize')}
    nameValueArgs.ROIScale (1,1) double {validateROIScale(nameValueArgs.ROIScale,'roialign.m','ROIScale')} = 1
    nameValueArgs.SamplingRatio {validateSamplingRatio(nameValueArgs.SamplingRatio,'roialign.m','SamplingRatio')} = 'auto';
end
end

%--------------------------------------------------------------------------
function validateOutputSize(sz, name, varname)
% Validate outputSize input
validateattributes(sz, {'numeric'}, ...
    {'row', 'nonempty', 'numel', 2, 'positive', 'real', 'nonsparse', 'integer'}, ...
    name, varname);
end

%--------------------------------------------------------------------------
function validateSamplingRatio(ratio, name, varname)
% Validate samplingRatio NV pair
if(isstring(ratio))
    ratio = char(ratio);
end
if(ischar(ratio))
    validatestring(ratio,{'auto'}, name, varname);
else
    validateattributes(ratio, {'numeric'}, ...
        {'real', 'nonempty', 'numel', 2, 'integer', 'positive', 'nonsparse','>=',1}, ...
        name, varname);
end
end

%--------------------------------------------------------------------------
function validateROIScale(scale, name, varname)
% Validate ROIScale NV pair
validateattributes(scale, {'numeric'}, ...
    {'real', 'nonempty','scalar', 'positive', 'nonsparse', '>', 0}, ...
    name, varname);
end

%--------------------------------------------------------------------------
function validateBatchIdx(batchIdx, X)
% Validate that batchIdx is within the valid range
vision.internal.errorIf(any(batchIdx < 1 | batchIdx > size(X, 4)),'vision:roiAlign:invalidBatchIdx');
end

%--------------------------------------------------------------------------
function flag = isSimMode()
flag = isempty(coder.target);
end
