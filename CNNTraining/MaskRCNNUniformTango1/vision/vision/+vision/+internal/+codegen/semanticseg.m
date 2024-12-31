function [C,scores,allScores] = semanticseg(I,net,varargin)%#codegen
% Contains the code generation implementation of semanticseg.

% Copyright 2024 The MathWorks, Inc.

coder.gpu.kernelfun;

narginchk(2, inf);

detectionInputIsDatastore = coder.const(~isnumeric(I) && ~islogical(I));

coder.internal.errorIf(detectionInputIsDatastore, 'vision:semanticseg:datastoreNotSupported');

% Make the compile time function calls extrinsic
coder.extrinsic('vision.internal.detector.checkROI')
coder.extrinsic('getNetworkProperties')

% Get the underlying Network properties
mxArraynet = dltargets.internal.sdk.getUnderlyingMxArrayNetwork(net);
[netSize,classes,numClasses,dataFormat,layerName] = coder.const(@getNetworkProperties, mxArraynet);

iCheckNetwork(net);
iCheckImage(I, netSize);
isDlNetwork = coder.const(isa(net, 'dlnetwork'));

isPruningNetwork = isa(net, 'deep.prune.TaylorPrunableNetwork');
if (isPruningNetwork)
    net = net.getUnderlyingNetwork;
end

useROI = false; % Default ROI not to be used

if( ~isempty(varargin) && (isa(varargin{1}, 'numeric')) )

    roi = varargin{1};

    % Error out if roi is not a constant
    coder.internal.assert(coder.internal.isConst(roi), 'vision:semanticseg:roiConstant')
    coder.internal.errorIf(~(isvector(roi) && (size(roi,2) == 4)), 'vision:semanticseg:roiIncorrectNumel')

    % Check whether ROI is fully contained in image
    coder.const(feval('vision.internal.detector.checkROI', roi,size(I)));

    useROI = true;

    params = iParseInputsCG(roi, varargin{2:end});
else
    % If roi is not provided initialize as zeros.
    roi = zeros(1,4);

    params = iParseInputsCG(roi, varargin{:});
end
OutputType = iCheckOutputType(params.OutputType, numClasses);

if isempty(varargin)
    Classes = "auto";
else
    for i = 1:numel(varargin)
        if isequal(varargin{i},"Classes")
            Classes = varargin{i+1};
            break;
        else
            Classes = "auto";
        end
    end
end

iCheckNumClasses(Classes, numClasses, isDlNetwork)

% Validate Classes
iAssertValidClasses(Classes)

nargoutchk(1,3);

roi = params.ROI;
useROI = coder.const(useROI);

Iroi = vision.internal.detector.cropImageIfRequested(I, roi, useROI);

% Convert image from RGB <-> grayscale as required by network.
CIroi = iConvertImageToMatchNumberOfNetworkImageChannels(Iroi, netSize);

if ~isa(Iroi,'uint8')
    % convert data to single if not uint8. Network processes data in
    % single. casting to single preserves user data ranges.
    X = single(CIroi);
else
    X = CIroi;
end

if isDlNetwork
    Ascores = iPredictDlnetwork(X, net, params, dataFormat);
else
    [h,w,c,~] = size(X);

    canUsePredict = isequal([h w c], netSize);

    if canUsePredict
        % Use predict when the input image resolution matches the network input
        % size for faster inference.
        Ascores = predict(net, X, 'MiniBatchSize', params.MiniBatchSize);

    else
        Ascores = activations(net, X, layerName, 'OutputAs', 'channels', ...
            'MiniBatchSize', params.MiniBatchSize);
    end
end

numNetImgDims = numel(netSize);
[Iscores, L] = max(Ascores, [], numNetImgDims);
 
if (strcmp(OutputType, 'uint8'))
    Lroi = uint8(L);
else
    Lroi = L;
end

% Remove singleton 3rd dim for 2-D inputs or 4th dim for 3-D inputs
% i.e. remove singleton channel dimension
sqzRoi = squeeze(Lroi);
sqzScores = squeeze(Iscores);

outputCategorical = coder.const(strcmpi(OutputType, 'categorical'));
channelDim = numel(netSize);
observationsDim = channelDim+1;
sizeI = cell(1,observationsDim);
[sizeI{1:observationsDim}] = size(I);

% Check whether we can copy the cropped results back into the original
% image. The assumption is the network output size matches the input
% size.
if useROI
    iAssertOutputSizeMatchesROISize(sqzRoi, X, channelDim);
end

isDlNetwork = coder.const(isa(net, 'dlnetwork'));

if isDlNetwork
    if ~iIsAuto(Classes)
        classes = Classes;
    end
    classesOne = coder.const(feval('categorical', classes, classes));

else
    if ~iIsAuto(Classes)
        classes = Classes;
        classesOne =  categorical(classes, classes);
    else
        classesOne = classes;
    end
end

classesTwo = classesOne(:);
classNames = categories(classesTwo);
label2Categorical = categorical(1:numel(classNames), 1:numel(classNames), classNames);

sizeC = cell(1, observationsDim-1);
k = 1;
for i = 1:coder.internal.indexInt(observationsDim)
    if i~=channelDim
        sizeC{1,k} = sizeI{i};
        k = k+1;
    else
        sizeC{1,k-1} = sizeI{i-1};
    end
end

numelSize = coder.const(numel(size(I)));
K = numel(classNames);
if numelSize <=3
    sizeC = [size(I,1:2), 1];
    sizeAllScores = [size(I,1:2), K];
else
    sizeC = [size(I,1:2), size(I,4)];
    sizeAllScores = [size(I,1:2), K, size(I,4)];
end

if outputCategorical
    % Convert label matrix to categorical
    Croi = label2Categorical(sqzRoi);

    % Replace NaN maxima with <undefined> labels
    nans = isnan(Iscores);
    if any(nans(:))
        Croi(nans) = categorical(NaN);
    end

    if useROI
        % Copy data into ROI region. Treat region outside of ROI as
        % <undefined>. <undefined> scores are NaN.
        C = NaN(sizeC);
        C = categorical(C, 1:numel(classNames), classNames);
        [startROI, endROI] = iCropRanges(roi, channelDim);

        C(startROI{1}:endROI{1}, startROI{2}:endROI{2}, :) = Croi;
    else
        C = Croi;
    end
else
    if useROI
        % Copy data into ROI region. Treat region outside of ROI as
        % undefined with label ID zero.
        C = zeros(sizeC, 'like', sqzRoi);
        [startROI, endROI] = iCropRanges(roi, channelDim);
        C(startROI{1}:endROI{1}, startROI{2}:endROI{2}, :) = sqzRoi;
    else
        C = sqzRoi;
    end
end

if useROI
    if nargout >= 2
        scores = NaN(sizeC, class(sqzScores));
        scores(startROI{1}:endROI{1}, startROI{2}:endROI{2}, :) = sqzScores;
    end
     
    if nargout == 3
        allScores = NaN(sizeAllScores, class(sqzScores));
        allScores(startROI{1}:endROI{1}, startROI{2}:endROI{2}, :, :) = Ascores;
    end
else
    scores = sqzScores;
    allScores = Ascores;
end

%--------------------------------------------------------------------------
function [startROI, endROI] = iCropRanges(roi, channelDim)
% iCropRanges - Returns the ROI indices as two cell arrays, startROI and
% endROI containing the start indices and corresponding end indices
% respectively for each spatial dimension. The channel dimension,
% channelDim is used to calculate the number of spatial dimensions.

numDims = channelDim-1;
startROI = cell(1,numDims);
endROI = cell(1,numDims);

% ROI format is [x y ... w h ...]. Swap order of x and y to make the first
% dimension the row and the second dimension the column.
roi(1:2) = roi([2 1]);

% Swap the order of width and height to compute the row-column ranges.
% 2D ROI
roi(3:4) = roi([4 3]);

for idx = 1:numDims
    startROI{idx} = roi(idx);
    endROI{idx} = startROI{idx} + roi(numDims+idx)-1;
end
%--------------------------------------------------------------------------
function params = iParseInputsCG(roi,varargin)

% Specify defaults of parameters
defaults = struct( ...
    'MiniBatchSize', 128, ...
    'ExecutionEnvironment', 'auto', ...
    'Acceleration', 'auto', ...
    'OutputType', coder.const('categorical'), ...
    'Classes', 'auto', ...
    'WriteLocation', uint32(0), ...
    'UseParallel', uint32(0), ...
    'OutputFolderName', uint32(0), ...
    'NamePrefix', uint32(0), ...
    'NameSuffix', uint32(0), ...
    'Verbose', uint32(0));

% Define parser mapping struct
pvPairs = struct( ...
    'MiniBatchSize', uint32(0), ...
    'ExecutionEnvironment', uint32(0), ...
    'Acceleration', uint32(0), ...
    'OutputType', uint32(0), ...
    'Classes', uint32(0), ...
    'WriteLocation', uint32(0), ...
    'UseParallel', uint32(0), ...
    'OutputFolderName', uint32(0), ...
    'NamePrefix', uint32(0), ...
    'NameSuffix', uint32(0), ...
    'Verbose', uint32(0));

% Specify parser options
poptions = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand',    false, ...
    'PartialMatching', 'unique', ...
    'IgnoreNulls', true);
% Parse PV pairs
pstruct = coder.internal.parseParameterInputs(pvPairs, poptions, varargin{:});

% Extract inputs
miniBatchSize = coder.internal.getParameterValue(pstruct.MiniBatchSize, defaults.MiniBatchSize, varargin{:});
executionEnvironment = coder.internal.getParameterValue(pstruct.ExecutionEnvironment, defaults.ExecutionEnvironment, varargin{:});
acceleration = coder.internal.getParameterValue(pstruct.Acceleration, defaults.Acceleration, varargin{:});
OutputType = coder.internal.getParameterValue(pstruct.OutputType, defaults.OutputType, varargin{:});
writeLocation = coder.internal.getParameterValue(pstruct.WriteLocation, defaults.WriteLocation, varargin{:});
useParallel = coder.internal.getParameterValue(pstruct.UseParallel, defaults.UseParallel, varargin{:});
outputFolderName = coder.internal.getParameterValue(pstruct.OutputFolderName, defaults.OutputFolderName, varargin{:});
namePrefix = coder.internal.getParameterValue(pstruct.NamePrefix, defaults.NamePrefix, varargin{:});
nameSuffix = coder.internal.getParameterValue(pstruct.NameSuffix, defaults.NameSuffix, varargin{:});
verbose = coder.internal.getParameterValue(pstruct.Verbose, defaults.Verbose, varargin{:});

% Explicitly constant fold
coder.internal.assert(coder.internal.isConst(miniBatchSize), 'vision:semanticseg:VariableSizeMiniBatch');

% Validate minibatchsize
vision.internal.cnn.validation.checkMiniBatchSize( ...
    coder.const(miniBatchSize), mfilename);

% Ignore ExecutionEnvironment
if coder.const(pstruct.ExecutionEnvironment ~= zeros('uint32'))
    coder.internal.compileWarning(...
        'vision:semanticseg:IgnoreInputArg', 'detect', 'ExecutionEnvironment');
end

% Ignore Acceleration
if coder.const(pstruct.Acceleration ~= zeros('uint32'))
    coder.internal.compileWarning(...
        'vision:semanticseg:IgnoreInputArg', 'detect', 'Acceleration');
end

% Ignore WriteLocation
if coder.const(pstruct.WriteLocation ~= zeros('uint32'))
    coder.internal.compileWarning(...
        'vision:semanticseg:IgnoreDatastoreNV', 'WriteLocation');
end


% Ignore UseParallel
if coder.const(pstruct.UseParallel ~= zeros('uint32'))
    coder.internal.compileWarning(...
        'vision:semanticseg:IgnoreDatastoreNV', 'UseParallel');
end

% Ignore OutputFolderName
if coder.const(pstruct.OutputFolderName ~= zeros('uint32'))
    coder.internal.compileWarning(...
        'vision:semanticseg:IgnoreDatastoreNV', 'OutputFolderName');
end

% Ignore NamePrefix
if coder.const(pstruct.NamePrefix ~= zeros('uint32'))
    coder.internal.compileWarning(...
        'vision:semanticseg:IgnoreDatastoreNV', 'NamePrefix');
end

% Ignore NameSuffix
if coder.const(pstruct.NameSuffix ~= zeros('uint32'))
    coder.internal.compileWarning(...
        'vision:semanticseg:IgnoreDatastoreNV', 'NameSuffix');
end

% Ignore Verbose
if coder.const(pstruct.Verbose ~= zeros('uint32'))
    coder.internal.compileWarning(...
        'vision:semanticseg:IgnoreDatastoreNV', 'Verbose');
end

params.ROI                      = single(roi);
params.MiniBatchSize            = coder.const(miniBatchSize);
params.ExecutionEnvironment     = executionEnvironment;
params.Acceleration             = acceleration;
params.OutputType               = OutputType;


%--------------------------------------------------------------------------
function iCheckNetwork(net)
validateattributes(net, {'SeriesNetwork', 'DAGNetwork', 'dlnetwork', 'deep.prune.TaylorPrunableNetwork'}, ...
    {'scalar', 'nonempty'}, mfilename, 'net');

coder.internal.errorIf( numel(net.InputNames) > 1,'vision:semanticseg:tooManyInputLayers');

%--------------------------------------------------------------------------
function [inputsize,classes,numClasses,dataFormat, layerName] = getNetworkProperties(net)
found = false;
for i = 1:numel(net.Layers)
    if isa(net.Layers(i), 'nnet.cnn.layer.ImageInputLayer') || ...
            isa(net.Layers(i), 'nnet.cnn.layer.Image3DInputLayer')
        found = true;
        break
    end
end

if ~found
    error(message('vision:semanticseg:missingInputLayer'));
end

inputsize = net.Layers(i).InputSize;

dlnet = coder.const(isa(net, 'dlnetwork'));

if dlnet
    pxLayerID = iFindAndAssertNetworkHasOneOutputLayer(net);
else
    pxLayerID = iFindAndAssertNetworkHasOnePixelClassificationLayer(net);
end

if dlnet
    [numClasses, dataFormat] = iFindFinalLayerSize(net);
    classes = 1:numClasses;
    classes = "C" + classes;
else
    dataFormat = '';
    numClasses = numel(net.Layers(pxLayerID).Classes);
    classes  = net.Layers(pxLayerID).Classes;
end

layerName = net.Layers(pxLayerID).Name;


%--------------------------------------------------------------------------
function iCheckImage(I, netSize)

classAttributes = {'double','single','int16','uint16','uint8','logical'};
validateattributes(I, classAttributes, {}, mfilename, 'I');

if isnumeric(I) || islogical(I)
    % Check ~empty, real, ~sparse for numeric inputs
    validateattributes(I, {'numeric', 'logical'}, {'nonempty', 'real', 'nonsparse'},...
        mfilename, 'I');


    if ndims(I) > 3
        % Max 4D inputs permitted
        validateattributes(I, {'numeric', 'logical'}, {'ndims', 4}, mfilename, 'I');
    end

    isNetImageRGB = numel(netSize) == 3 && netSize(end) == 3;
    isImageRGB    = size(I,3) == 3;

    % Check channel size for non RGB images
    if((ndims(I) >= 3) && ~(isImageRGB || isNetImageRGB)) %#ok<ISMAT>
        % Verify if numChannels of image = numChannels of imageInputLayer
        coder.internal.errorIf(size(I,3) ~= netSize(3), 'vision:semanticseg:invalidNumChannels');
    end

    sz = size(I);
    coder.internal.errorIf(any(sz(1:2) < netSize(1:2)), 'vision:ObjectDetector:imageSmallerThanNetwork');
end

%--------------------------------------------------------------------------
function iAssertOutputSizeMatchesROISize(Croi, Iroi, inputChannelDim)
% ROI processing expects the output size of the network to match the ROI
% size. Some networks may produce output sizes that do not match the
% cropped input size. Issue an error to handle these cases.

% Croi is the output of network without the singleton channel dimension,
% but it can have a trailing batch dimension. Get the size of just the
% spatial dimensions.
assert(inputChannelDim > 2);
szC = size(Croi, 1:inputChannelDim-1); 
szROI = size(Iroi, 1:inputChannelDim-1);

coder.internal.errorIf(~isequal(szC, szROI), 'vision:semanticseg:outputROISizeMismatch', szC, szROI)

%--------------------------------------------------------------------------
function id = iFindAndAssertNetworkHasOneOutputLayer(dlnet)
outputName = string(dlnet.OutputNames);

% TaylorPrunableNetworks contain Gating layers as output layers.
% Ensure number of non-gating ouputs are not more than one by counting only
% non-gating outputs.
isOutputLayerGating = contains(dlnet.OutputNames, "GatingLayer");
if sum(~isOutputLayerGating) > 1
    error(message('vision:semanticseg:tooManyOutputLayers'));
end

fun = @(x)isequal(string(x.Name), outputName);
idx = arrayfun(fun, dlnet.Layers);
id = find(idx);

%--------------------------------------------------------------------------
function id = iFindAndAssertNetworkHasOnePixelClassificationLayer(net)
% Network must have a pixel classification layer or a user-defined
% classification layer.
id = arrayfun(@(x)isa(x, 'nnet.cnn.layer.PixelClassificationLayer') || ...
    isa(x, 'nnet.cnn.layer.DicePixelClassificationLayer') || ...
    isa(x, 'nnet.cnn.layer.FocalLossLayer') || ...
    isa(x,'nnet.layer.ClassificationLayer'), net.Layers);
id = find(id);
if isempty(id)
    error(message('vision:semanticseg:noPixelClassificationLayer'));
end

if numel(id) > 1
    error(message('vision:semanticseg:tooManyPixelClsLayers'));
end

%--------------------------------------------------------------------------
function [numClasses,dataFormat] = iFindFinalLayerSize(dlnet)
coder.extrinsic('deep.internal.sdk.forwardDataAttributes');
[sizes, formats] = coder.const(@deep.internal.sdk.forwardDataAttributes,dlnet);
% sizes will contain channel and batch dimension at the end
% eg. sizes = {[32,32,2,1]}, formats = {'SSCB'}
numClasses = sizes{1}(end-1);
dataFormat = formats{1};

%--------------------------------------------------------------------------
function iCheckNumClasses(Classes, numClasses, isdlnetwork)
if ~isequal(Classes, 'auto') && numClasses ~= numel(Classes)
    if isdlnetwork
        error(message('vision:semanticseg:invalidClassesInputSizeDlnetwork'));
    else
        error(message('vision:semanticseg:invalidClassesInputSize'));
    end
end

%--------------------------------------------------------------------------
function type = iCheckOutputType(type,numClasses)
type = validatestring(type,{'categorical','double','uint8'},mfilename,'OutputType');
type = char(type);

coder.internal.errorIf(strcmp(type,'uint8')&&(numClasses>255), 'vision:semanticseg:tooLargeNumClasses');

%--------------------------------------------------------------------------
function allScores = iPredictDlnetwork(Y, net, params, dataFormat)
if isinteger(Y)
    % Cast data to single, if the data is 'int16','uint16', or
    % 'uint8'.
    X = single(Y);
else
    X = Y;
end

%H W C B
batchSize = size(X,4);
batchDim = 4;

if batchSize > params.MiniBatchSize
    allScores = iPredictOneBatchAtATimeDlnetwork(X, net, params,dataFormat,batchSize, batchDim);
else
    allScores = iPredictOneBatchDlnetwork(dlarray(X, dataFormat), net);
end

%--------------------------------------------------------------------------
function dlY = iPredictOneBatchDlnetwork(dlX, dlnet)
dlY = predict(dlnet, dlX);
% gather from GPU as GPU can only fit MiniBatchSize amount of data.
dlY = gather(extractdata(dlY));

%--------------------------------------------------------------------------
function allScores = iPredictOneBatchAtATimeDlnetwork(X, dlnet, params,dataFormat, batchSize, batchDim)

numBatches = ceil(batchSize/params.MiniBatchSize);
data = cell(numBatches, 1);

for i =1:numBatches
    data{i} = coder.nullcopy(zeros([size(X,1:3), params.MiniBatchSize],'like',X));
end
ii = 1;

for sidx = 1:params.MiniBatchSize:batchSize
    endIdx = min(sidx+params.MiniBatchSize-1, batchSize);
    imBatch = zeros([size(X,1:3),params.MiniBatchSize],'like',X);
    imBatch(:, :, :, 1:endIdx-sidx+1) = X(:, :, :, sidx:endIdx);
    dlX = dlarray(imBatch, dataFormat);
    data{ii} = iPredictOneBatchDlnetwork(dlX, dlnet);
    ii = ii+1;
end

data2 = cat(batchDim, data{:});
allScores = data2(:, :, :, 1:endIdx);

%--------------------------------------------------------------------------
function Iout = iConvertImageToMatchNumberOfNetworkImageChannels(I, netImageSize)
is2dNetwork = coder.const(numel(netImageSize)==3);
if is2dNetwork
    isNetImageRGB = coder.const(numel(netImageSize) == 3 && netImageSize(end) == 3);
    isImageRGB    = coder.const(size(I,3) == 3);

    if isImageRGB && ~isNetImageRGB
        Iout = rgb2gray(I);
    elseif ~isImageRGB && isNetImageRGB
        Iout = repmat(I, 1, 1, 3);
    else
        Iout = I;
    end
else
    Iout = I;
end
%--------------------------------------------------------------------------
function iAssertValidClasses(value)

if ~iIsAuto(value)
    coder.internal.errorIf(isempty(value) || ~isvector(value) || ~iIsValidDataType(value), 'nnet_cnn:layer:ClassificationOutputLayer:InvalidClasses');
    
    coder.internal.errorIf(~iscategorical(value) && iHasDuplicates(value), 'nnet_cnn:layer:ClassificationOutputLayer:DuplicateClasses');

    names = value;
    % There should be 2 or more classes
    coder.internal.errorIf(numel(names)<2, 'vision:semanticseg:MultiClass');
end

%--------------------------------------------------------------------------
function tf = iIsAuto(val)
tf = isequal(val, "auto");

%--------------------------------------------------------------------------
function tf = iIsValidDataType(value)
tf = iscategorical(value) || iscellstr(value) || isstring(value);

%--------------------------------------------------------------------------
function tf = iHasDuplicates(value)
tf = ~isequal(value, unique(value, 'stable'));

