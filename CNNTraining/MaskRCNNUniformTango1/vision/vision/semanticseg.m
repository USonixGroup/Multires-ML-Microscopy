function [C, scores, allScores] = semanticseg(I, net, varargin)
%#codegen

% Copyright 2017-2024 The MathWorks, Inc.

narginchk(2, inf);
isSimMode = coder.const(isempty(coder.target));
if isSimMode
    [params, net] = iParseInputs(I, net, varargin{:});
    if iIsDatastore(I)
        nargoutchk(1,1);

        % Make a copy of the data store to prevent altering the state of the
        % input datastore.
        ds = copy(I);
        ds.reset();

        if isa(ds,'matlab.io.datastore.ImageDatastore')
            % As an optimization for imageDatastore, set the ReadSize to match
            % the mini-batch size. This enables async pre-fetching.
            ds.ReadSize = params.MiniBatchSize;
        else
            ds = iAddFirstColumnSelectionTransform(ds);
        end

        params.OutputFolderName = iCreateOutputFolder(params.WriteLocation, params.OutputFolderName);

        if params.UseParallel
            filenames = iProcessImageDatastoreInParallel(ds, net, params);
        else
            filenames = iProcessImageDatastoreSerially(ds, net, params);
        end

        % Create output pixelLabelDatastore
        classnames = params.ClassNames;
        values = 1:numel(classnames);
        if iIs2dNetwork(params.NetImageSize)
            C = pixelLabelDatastore(filenames, classnames, values);
        else
            C = pixelLabelDatastore(filenames, classnames, values,...
                'ReadFcn',@(x)pixelLabelMatFileReader(x,values,classnames), 'FileExtensions',{'.mat'});
        end

    else % process single image.

        nargoutchk(1,3);

        roi    = params.ROI;
        useROI = params.UseROI;

        if params.Is3D
            Iroi = vision.internal.detector.cropImageIfRequested(I, roi, useROI, params.Is3D);
        else
            Iroi = vision.internal.detector.cropImageIfRequested(I, roi, useROI);
        end

        % Convert image from RGB <-> grayscale as required by network.
        % Do nothing for 3-D images.
        Iroi = iConvertImageToMatchNumberOfNetworkImageChannels(...
            Iroi, params.NetImageSize);

        if ~isa(Iroi,'uint8')
            % convert data to single if not uint8. Network processes data in
            % single. casting to single preserves user data ranges.
            Iroi = single(Iroi);
        end

        try %#ok
            [Lroi, scores, allScores] = iClassifyImagePixels(Iroi, net, params);
        catch ME
            throwAsCaller(ME)
        end

        % remove singleton 3rd dim for 2-D inputs or 4th dim for 3-D inputs
        % i.e. remove singleton channel dimension
        Lroi = squeeze(Lroi);
        scores = squeeze(scores);

        outputCategorical = strcmpi(params.OutputType,'categorical');

        channelDim = numel(params.NetImageSize);
        observationsDim = channelDim+1;
        sizeI = cell(1,observationsDim);
        [sizeI{1:observationsDim}] = size(I);

        % Check whether we can copy the cropped results back into the original
        % image. The assumption is the network output size matches the input
        % size.
        if useROI
            iAssertOutputSizeMatchesROISize(Lroi, Iroi, channelDim);
        end

        cats = params.ClassNames;
        if outputCategorical
            % Convert label matrix to categorical
            Croi = params.Label2Categorical(Lroi);

            % Replace NaN maxima with <undefined> labels
            nans = isnan(scores);
            if any(nans(:))
                Croi(nans) = categorical(NaN);
            end

            if useROI
                % copy data into ROI region. Treat region outside of ROI as
                % <undefined>. <undefined> scores are NaN.
                C = categorical(NaN, 1:numel(cats), cats);
                sizeC = sizeI;
                sizeC(channelDim) = [];
                C = repelem(C,sizeC{:});
                [startROI, endROI] = iCropRanges(roi, channelDim);

                C = iCopyDataIntoROIRegion(C, Croi, startROI, endROI, channelDim);
            else
                C = Croi;
            end
        else
            if useROI
                % copy data into ROI region. Treat region outside of ROI as
                % undefined with label ID zero.
                sizeC = sizeI;
                sizeC(channelDim) = [];
                C = zeros(sizeC{:},'like',Lroi);
                [startROI, endROI] = iCropRanges(roi, channelDim);

                C = iCopyDataIntoROIRegion(C, Lroi, startROI, endROI, channelDim);
            else
                C = Lroi;
            end
        end

        if useROI
            if nargout >= 2
                s = NaN(sizeC{:}, class(scores));
                scores = iCopyDataIntoROIRegion(s, scores, startROI, endROI, channelDim);
            end

            if nargout == 3
                K  = numel(cats);
                sizeAllScores = sizeI;
                sizeAllScores{channelDim} = K;
                as = NaN(sizeAllScores{:}, class(scores));

                allScores = iCopyDataIntoROIRegion(as, allScores, startROI, endROI, channelDim);
            end

        end
    end
else
    [C,scores,allScores] = vision.internal.codegen.semanticseg(I, net, varargin{:});
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
if numel(roi) == 4
    % 2D ROI
    roi(3:4) = roi([4 3]);
else
    % 3D ROI
    roi(4:5) = roi([5 4]);
end

for idx = 1:numDims
    startROI{idx} = roi(idx);
    endROI{idx} = startROI{idx} + roi(numDims+idx)-1;
end

%--------------------------------------------------------------------------
function [params, net] = iParseInputs(I, net, varargin)

iCheckNetwork(net);

isPruningNetwork = isa(net,'deep.prune.TaylorPrunableNetwork');
if (isPruningNetwork)
    net = net.getUnderlyingNetwork;
end

netSize = iNetworkImageSize(net);
iCheckImage(I, netSize);

isDatastore = iIsDatastore(I);

if isa(net, 'dlnetwork')
    pxLayerID = iFindAndAssertNetworkHasOneOutputLayer(net);
else
    pxLayerID = iFindAndAssertNetworkHasOnePixelClassificationLayer(net);
end


p = inputParser;
if iIs2dNetwork(netSize)
    expectedNumelROI = 4;
    Is3D = false;
else
    expectedNumelROI = 6;
    Is3D = true;
end
p.addOptional('roi', zeros(0,expectedNumelROI));

p.addParameter('OutputType', 'categorical');

p.addParameter('MiniBatchSize', 128, ...
    @(x)vision.internal.cnn.validation.checkMiniBatchSize(x,mfilename));

p.addParameter('ExecutionEnvironment', 'auto');

p.addParameter('Acceleration', 'auto');

p.addParameter('Classes', 'auto', @iAssertValidClasses);

p.addParameter('WriteLocation', pwd);

p.addParameter('OutputFolderName', 'semanticsegOutput');

p.addParameter('NamePrefix', 'pixelLabel', @iCheckNamePrefix);

p.addParameter('NameSuffix', "", @iCheckNameSuffix);

p.addParameter('Verbose', true, ...
    @(x)vision.internal.inputValidation.validateLogical(x,'Verbose'))

if isDatastore
    useParallelDefault = vision.internal.useParallelPreference();
else
    useParallelDefault = false;
end
p.addParameter('UseParallel', useParallelDefault, ...
    @(x)vision.internal.inputValidation.validateLogical(x,'UseParallel'));

p.parse(varargin{:});

userInput = p.Results;

useROI = ~ismember('roi', p.UsingDefaults);

if useROI
    if isDatastore
        error(message('vision:semanticseg:imdsROIInvalid'));
    end
    vision.internal.detector.checkROI(userInput.roi, size(I), Is3D);
end

wasSpecified = @(x)~ismember(x,p.UsingDefaults);
if ~isDatastore && ...
        (wasSpecified('WriteLocation') || ...
        wasSpecified('NamePrefix') ||...
        wasSpecified('Verbose') || ...
        wasSpecified('UseParallel'))
    
    warning(message('vision:semanticseg:onlyApplyWithImds'))
end

if isDatastore
    % Only check write location when input is a datastore.
    iCheckWriteLocation(userInput.WriteLocation);    

    iCheckOutputFolderName(userInput.OutputFolderName);

    if ~wasSpecified('NameSuffix')
        % By default, use input file names as the suffix.
        params.NameSuffixSource = "filename";
    else
        params.NameSuffixSource = "user";
    end
end

if userInput.UseParallel
    % Check for PCT installation
    try %#ok
        % GCP will error if PCT is not available.
        gcp('nocreate');
    catch
        userInput.UseParallel = false;
    end
end

exeenv = vision.internal.cnn.validation.checkExecutionEnvironment(...
    userInput.ExecutionEnvironment, mfilename);
exeenv = iCheckGPUMismatch(exeenv);

accel = vision.internal.cnn.validation.checkAcceleration(...
    userInput.Acceleration, mfilename);


isdlnetwork = isa(net,'dlnetwork');
if isdlnetwork
    [numClasses, dataFormat] = iFindFinalLayerSize(net);
    iCheckNumClasses(userInput, numClasses, isdlnetwork);

    if iIsAuto(userInput.Classes)
        % pixelLabelDatastore expects class names that are valid MATLAB
        % varnames.
        classes = 1:numClasses;
        classes = "C" + classes; 
    else
        classes = userInput.Classes;
    end
    classes = categorical(classes, classes);
else
    dataFormat = '';
    numClasses = numel(net.Layers(pxLayerID).Classes);
    iCheckNumClasses(userInput, numClasses, isdlnetwork);

    if iIsAuto(userInput.Classes)
        classes = net.Layers(pxLayerID).Classes;
    else
        classes = userInput.Classes;
        classes = categorical(classes, classes);
    end
end

classes = classes(:);
classNames = categories(classes);

type = iCheckOutputType(userInput.OutputType, numClasses);

params.ROI                  = double(userInput.roi);
params.UseROI               = useROI;
params.MiniBatchSize        = double(userInput.MiniBatchSize);
params.OutputType           = type;
params.ExecutionEnvironment = exeenv;
params.PixelLayerID         = pxLayerID;
params.WriteLocation        = strtrim(char(userInput.WriteLocation));
params.OutputFolderName     = strtrim(char(userInput.OutputFolderName));
params.NamePrefix           = strtrim(char(userInput.NamePrefix));
params.NameSuffix           = strtrim(char(userInput.NameSuffix));
params.Verbose              = logical(userInput.Verbose);
params.UseParallel          = logical(userInput.UseParallel);
params.NetImageSize         = netSize;
params.Is3D                 = Is3D;
params.Acceleration         = accel;
params.Classes              = classes;
params.ClassNames           = classNames;
params.Label2Categorical    = categorical(1:numel(classNames), 1:numel(classNames), classNames);
params.IsDlnetwork          = isdlnetwork;
params.DataFormat           = dataFormat;

%--------------------------------------------------------------------------
function [L, scores, allScores] = iClassifyImagePixels(X, net, params)

if params.IsDlnetwork
    allScores = iPredictDlnetwork(X, net, params);
else
    allScores = iPredictDAGSeriesNetwork(X, net, params);
end

numNetImgDims = numel(params.NetImageSize);
[scores, L] = max(allScores,[],numNetImgDims);

if(strcmp(params.OutputType,'uint8'))
    L = uint8(L);
end

%--------------------------------------------------------------------------
function allScores = iPredictDlnetwork(X, net, params)
if isinteger(X)
    % Cast data to single, if the data is 'int16','uint16', or
    % 'uint8'.
    X = single(X);
end
if params.Is3D
    % H W D C B
    batchSize = size(X, 5);
    batchDim = 5;
else
    % H W C B
    batchSize = size(X, 4);
    batchDim = 4;
end
if batchSize > params.MiniBatchSize
    allScores = iPredictOneBatchAtATimeDlnetwork(X, net, params, batchSize, batchDim);
else
    allScores = iPredictOneBatchDlnetwork(dlarray(X,params.DataFormat), net, ...
        params);
end

%--------------------------------------------------------------------------
function allScores = iPredictDAGSeriesNetwork(X, net, params)
[h,w,c,~] = size(X);

if params.Is3D == 1
    canUsePredict = isequal([h w c],params.NetImageSize(1:3));
else
    canUsePredict = isequal([h w c],params.NetImageSize);
end

if canUsePredict
    % Use predict when the input image resolution matches the network input
    % size for faster inference.
    allScores = predict(net, X, ...
        'Acceleration',params.Acceleration,...
        'ExecutionEnvironment', params.ExecutionEnvironment, ...
        'MiniBatchSize', params.MiniBatchSize);

else
    layerName = net.Layers(params.PixelLayerID).Name;
    allScores = activations(net, X, layerName, ...
        'OutputAs', 'channels', ...
        'Acceleration',params.Acceleration,...
        'ExecutionEnvironment', params.ExecutionEnvironment, ...
        'MiniBatchSize', params.MiniBatchSize);
end

%--------------------------------------------------------------------------
function dlY = iPredictOneBatchDlnetwork(dlX, dlnet, params)
if isequal(params.ExecutionEnvironment, 'gpu')
    dlX = gpuArray(dlX);
end
dlY = predict(dlnet, dlX, 'Acceleration', params.Acceleration);
% gather from GPU as GPU can only fit MiniBatchSize amount of data.
dlY = gather(extractdata(dlY));

%--------------------------------------------------------------------------
function allScores = iPredictOneBatchAtATimeDlnetwork(X, dlnet, params, batchSize, batchDim)
ds = arrayDatastore(X, 'IterationDimension', batchDim);
mbq = minibatchqueue(ds, 'MiniBatchSize', params.MiniBatchSize, ...
    'MiniBatchFormat', params.DataFormat,...
    'MiniBatchFcn', @(X)createMiniBatch(X,batchDim), ...
    'OutputEnvironment', params.ExecutionEnvironment);
numBatches = ceil(batchSize/params.MiniBatchSize);
data = cell(numBatches, 1); 
ii = 1;
while hasdata(mbq)
    dlX = next(mbq);
    data{ii} = iPredictOneBatchDlnetwork(dlX, dlnet, params);
    ii = ii + 1;
end
allScores = cat(batchDim, data{:});

%--------------------------------------------------------------------------
function images = createMiniBatch(images, batchDim)
% Collate the input cell arrays
images = cat(batchDim,images{:});        

%--------------------------------------------------------------------------
function type = iCheckOutputType(type,numClasses)
type = validatestring(type,{'categorical','double','uint8'},mfilename,'OutputType');
type = char(type);

if strcmp(type,'uint8')
    if(numClasses>255)
        error(message('vision:semanticseg:tooLargeNumClasses'));
    end
end

%--------------------------------------------------------------------------
function iCheckImage(I, netSize)
classAttributes = {'double','single','int16','uint16','uint8','logical',...
    'matlab.io.datastore.Datastore','matlab.io.Datastore'};
validateattributes(I, classAttributes, {}, mfilename, 'I');

if isnumeric(I) || islogical(I)
    % Check ~empty, real, ~sparse for numeric inputs
    validateattributes(I, {'numeric', 'logical'}, {'nonempty','real', 'nonsparse'},...
        mfilename, 'I');
    
    if iIs2dNetwork(netSize)
        if ndims(I) > 3
            % Max 4D inputs permitted
            validateattributes(I, {'numeric', 'logical'}, {'ndims', 4}, mfilename, 'I');
        end
        
        isNetImageRGB = numel(netSize) == 3 && netSize(end) == 3;
        isImageRGB    = size(I,3) == 3;
        
        % Check channel size for non RGB images
        if((ndims(I) >= 3) && ~(isImageRGB || isNetImageRGB)) %#ok<ISMAT>
            % Verify if numChannels of image = numChannels of imageInputLayer
            if(size(I,3) ~= netSize(3))
                error(message('vision:semanticseg:invalidNumChannels', netSize(3)));
            end
        end
        
        sz = size(I);
        if any(sz(1:2) < netSize(1:2))
            error(message('vision:ObjectDetector:imageSmallerThanNetwork',mat2str(netSize(1:2))));
        end
    else % 3-D Network
        maxDims = 5;
        numDims = ndims(I);
        if numDims > 4
            % Max 5D inputs permitted
            validateattributes(I, {'numeric', 'logical'}, {'ndims', maxDims}, mfilename, 'I');
        end
        
        sz = ones(1,maxDims);
        sz(1:numDims) = size(I);
        if any(sz(1:3) < netSize(1:3))
            error(message('vision:ObjectDetector:imageSmallerThanNetwork',mat2str(netSize(1:3))));
        end
    end
    
end

%--------------------------------------------------------------------------
function iCheckNetwork(net)
validateattributes(net, {'SeriesNetwork', 'DAGNetwork', 'dlnetwork', 'deep.prune.TaylorPrunableNetwork'}, ...
    {'scalar', 'nonempty'}, mfilename, 'net');

if numel(net.InputNames) > 1
    error(message('vision:semanticseg:tooManyInputLayers'))
end

%--------------------------------------------------------------------------
function iAssertValidClasses(value)
iEvalAndThrow(@()...
    nnet.internal.cnn.layer.paramvalidation.validateClasses(value));
if ~iIsAuto(value)
    names = string(value);
    % There should be 2 or more classes
    if numel(names)<2
        error(message('vision:semanticseg:MultiClass'));
    end
end

%--------------------------------------------------------------------------
function tf = iIsAuto(val)
tf = isequal(string(val), "auto");

%--------------------------------------------------------------------------
function iEvalAndThrow(func)
% Omit the stack containing internal functions by throwing as caller
try %#ok
    func();
catch exception
    throwAsCaller(exception)
end

%--------------------------------------------------------------------------
function iCheckNumClasses(userInput, numClasses, isdlnetwork)
if ~isequal(userInput.Classes, 'auto') && numClasses ~= numel(userInput.Classes)
    if isdlnetwork
        error(message('vision:semanticseg:invalidClassesInputSizeDlnetwork'));
    else
        error(message('vision:semanticseg:invalidClassesInputSize'));
    end
end

%--------------------------------------------------------------------------
function [numClasses,dataFormat] = iFindFinalLayerSize(dlnet)
[sizes, formats] = deep.internal.sdk.forwardDataAttributes(dlnet);
% sizes will contain channel and batch dimension at the end
% eg. sizes = {[32,32,2,1]}, formats = {'SSCB'}
numClasses = sizes{1}(end-1);
dataFormat = formats{1};


%--------------------------------------------------------------------------
function id = iFindAndAssertNetworkHasOneOutputLayer(dlnet)
outputName = string(dlnet.OutputNames);

% TaylorPrunableNetworks contain Gating layers as output layers.
% Ensure number of non-gating ouputs are not more than one by counting only
% non-gating outputs. 
isOutputLayerGating = contains(dlnet.OutputNames,"GatingLayer");
if sum(~isOutputLayerGating) > 1
    error(message('vision:semanticseg:tooManyOutputLayers'));
end

fun = @(x)isequal(string(x.Name),outputName);
idx = arrayfun(fun,dlnet.Layers);
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
function iCheckWriteLocation(x)
validateattributes(x, {'char','string'}, {'scalartext'}, ...
    mfilename, 'WriteLocation')

if ~exist(x,'dir')
    error(message('vision:semanticseg:dirDoesNotExist'));
end

vision.internal.inputValidation.checkWritePermissions(x);

%--------------------------------------------------------------------------
function iCheckOutputFolderName(x)
validateattributes(x, {'char','string'}, {'scalartext'}, ...
    mfilename, 'OutputFolderName')

%--------------------------------------------------------------------------
function iCheckNamePrefix(x)
validateattributes(x, {'char','string'}, {'scalartext'}, ...
    mfilename, 'NamePrefix')

%--------------------------------------------------------------------------
function iCheckNameSuffix(x)
validateattributes(x, {'char','string'}, {'scalartext'}, ...
    mfilename, 'NameSuffix')

%--------------------------------------------------------------------------
function filenames = iWritePixelLabelData(L, indices, params, N, info)
writeLocation = params.WriteLocation;
name = iCreateFileName(params.NamePrefix, indices, N, params.NetImageSize, info);
filenames = iPrependOutputLocation(name, writeLocation, params.OutputFolderName);
iWriteImageBatch(L,filenames,params.NetImageSize);

%--------------------------------------------------------------------------
function name = iCreateFileName(prefix, idx, numImages, netImageSize, info)
% Choose PNG format for 2-D images and MAT files for 3-D.
if iIs2dNetwork(netImageSize)
    ext = 'png';
else
    ext = 'mat'; 
end

% Determine the output file format. When the number of observations
% is nonfinite, simply append the file ID. Otherwise, use %0d to have the ID
% string based on the number of observations. For example, for 1000
% observations, produce label_0001, label_0002, ..., label_1000.
if isfinite(numImages) 
    format = sprintf('%%s_%%0%dd%%s.%s', string(numImages).strlength,ext);
else
    format = sprintf('%%s_%%d%%s.%s',ext);
end

% Generate the filenames.
name = cell(numel(idx),1);
for i = 1:numel(idx)
    suffix = string(info(i));
    if ~isequal(suffix,"")
        [~,suffix,~] = fileparts(suffix);
        suffix = "_" + suffix;
    end
    name{i} = sprintf(format, prefix, idx(i), suffix);
end

%--------------------------------------------------------------------------
function iPrintHeader(printer)
printer.printMessage('vision:semanticseg:verboseHeader');
printer.print('-------------------------------------');
printer.linebreak();

%--------------------------------------------------------------------------
function updateMessage(printer, prevMessage, nextMessage)
backspace = sprintf(repmat('\b',1,numel(prevMessage))); % figure how much to delete
printer.print([backspace nextMessage]);

%--------------------------------------------------------------------------
function nextMessage = iPrintInitProgress(printer, prevMessage, k)
nextMessage = getString(message('vision:semanticseg:verboseProgressTxt',k));
updateMessage(printer, prevMessage(1:end-1), nextMessage);

%--------------------------------------------------------------------------
function nextMessage = iPrintProgress(printer, prevMessage, k)
nextMessage = getString(message('vision:semanticseg:verboseProgressTxt',k));
updateMessage(printer, prevMessage, nextMessage);

%--------------------------------------------------------------------------
function [futureWriteBuffer, filename] = ...
    iParallelWritePixelLabelData(L, idx, params, futureWriteBuffer, numImages, info)
% Push write operation onto future buffer. First remove finished futures.
% If buffer is full, wait till one complete then pop it from the buffer.
%
% L can be a single image or a batch of images. idx is a scalar for a
% single image and a vector for a batch of images.

iErrorIfAnyFutureFailed(futureWriteBuffer);

% Remove finished futures.
finished = arrayfun(@(f)strcmp(f.State,'finished'),futureWriteBuffer);
futureWriteBuffer(finished) = [];

% Add to future buffer.
filename = iCreateFileName(params.NamePrefix, idx, numImages, params.NetImageSize, info);
filename = iPrependOutputLocation(filename, params.WriteLocation, params.OutputFolderName);

futureWriteBuffer(end+1) = parfeval(...
    @iWriteImageBatch, 0, uint8(L), filename, params.NetImageSize);

if length(futureWriteBuffer) > params.MiniBatchSize
    % Buffer is full. Wait till one of the futures is done.
    idx = fetchNext(futureWriteBuffer);
    futureWriteBuffer(idx) = [];
end

%--------------------------------------------------------------------------
function outputFolderName = iCreateOutputFolder(writeLocation, outputFolderName)
if isequal(strlength(outputFolderName),0)
    % Output foldername is empty, write results into write location.   
    outputFolderName = '';
else
    % Create output folder.     
    outputFolderName = iCreateUniqueFoldername(writeLocation, outputFolderName);
    outputLocation = fullfile(writeLocation, outputFolderName);
    try %#ok        
        [success, msg, msgId] = mkdir(outputLocation);
        if ~success
            throw(MException(msgId,msg));            
        end
    catch ME
        iThrowUnableToCreateOutputFolderMessage(ME,outputLocation);
    end
end

%--------------------------------------------------------------------------
function uniqueName = iCreateUniqueFoldername(writeLocation, outputFolderName)
putativeLocation = fullfile(writeLocation, outputFolderName);
uniqueName = outputFolderName;
k = 0;
while exist(putativeLocation) %#ok<EXIST>
    k = k + 1;
    uniqueName = outputFolderName + "_" + k;
    putativeLocation = fullfile(writeLocation, uniqueName);
end
uniqueName = char(uniqueName);

%--------------------------------------------------------------------------
function iThrowUnableToCreateOutputFolderMessage(cause,outputFolderName)
msg = message('vision:semanticseg:UnableToCreateOutputFolder',outputFolderName);
exception = MException(msg);
exception = addCause(exception,cause);
throwAsCaller(exception);

%--------------------------------------------------------------------------
function folder = iPrependOutputLocation(filename, writeLocation, outputFolderName)
folder = fullfile(writeLocation, outputFolderName, filename);

%--------------------------------------------------------------------------
function iWriteImageBatch(I,names,netImageSize)

if iIs2dNetwork(netImageSize)
    for i = 1:numel(names)
        imwrite(I(:,:,:,i),names{i});
    end
else
    for i = 1:numel(names)
        labelImage = I(:,:,:,:,i);
        save(names{i},'labelImage');
        clear labelImage;
    end
end

%--------------------------------------------------------------------------
function filenames = iProcessImageDatastoreSerially(ds, net, params)
numImages = iNumberOfObservations(ds);

if isfinite(numImages)
    filenames = cell(numImages,1);
else
    filenames = {};
end

printer = vision.internal.MessagePrinter.configure(params.Verbose);

iPrintHeader(printer);
msg = iPrintInitProgress(printer,'', 1);

loader = iCreateDataLoader(ds,params);

% Iterate through data and write results to disk.
k = 1;
while hasdata(loader)
    out = nextBatch(loader);

    X = out{1};
    info = out{2};
    
    [batchSize, isDataBatched] = iDetermineBatchSize(X, params.NetImageSize);

    idx = k:k+batchSize-1;
    
    if isDataBatched
        L = iClassifyImagePixels(X, net, params);
        filenames(idx) = iWritePixelLabelData(uint8(L), idx, params, numImages, info);
        msg = iPrintProgress(printer, msg, idx(end));
    else
        for i = 1:numel(idx)
            
            L = iClassifyImagePixels(X{i}, net, params);
            
            filenames(idx(i)) = iWritePixelLabelData(uint8(L), idx(i), params, numImages, info(i));
            
            msg = iPrintProgress(printer, msg, idx(i));
            
        end
    end
    k = idx(end)+1;
end
printer.linebreak(2);

%--------------------------------------------------------------------------
function filenames = iProcessImageDatastoreInParallel(ds, net, params)

isLocalPoolOpen = iAssertOpenPoolIsLocal();

if ~isLocalPoolOpen
    tryToCreateLocalPool();
end

numImages = iNumberOfObservations(ds);

if isfinite(numImages)
    filenames = cell(numImages,1);
else
    filenames = {};
end

printer = vision.internal.MessagePrinter.configure(params.Verbose);

iPrintHeader(printer);

msg = iPrintInitProgress(printer,'', 1);

% pre-allocate future buffer.
futureWriteBuffer = parallel.FevalFuture.empty();

loader = iCreateDataLoader(ds,params);

k = 1;
while hasdata(loader)
    out = nextBatch(loader);

    X    = out{1};
    info = out{2};
    
    [batchSize, isDataBatched] = iDetermineBatchSize(X, params.NetImageSize);
    
    idx = k:k+batchSize-1;
    
    if isDataBatched
        
        L = iClassifyImagePixels(X, net, params);
        
        [futureWriteBuffer, filenames(idx)] = ...
            iParallelWritePixelLabelData(L, idx, params, futureWriteBuffer, numImages, info);
        
        msg = iPrintProgress(printer, msg, idx(end));
        
    else
        
        for i = 1:numel(idx)
            
            L = iClassifyImagePixels(X{i}, net, params);
            
            [futureWriteBuffer, filenames(idx(i))] = ...
                iParallelWritePixelLabelData(L, idx(i), params, futureWriteBuffer, numImages, info(i));
            
            msg = iPrintProgress(printer, msg, idx(i));
        end
    end
    k = idx(end)+1;
end

% wait for all futures to finish
fetchOutputs(futureWriteBuffer);
iErrorIfAnyFutureFailed(futureWriteBuffer);

printer.linebreak(2);

%--------------------------------------------------------------------------
function out = iTryToBatchData(X, netImageSize)
try %#ok
    numNetImgDims = numel(netImageSize);
    channelDims = numNetImgDims+1;
    if iscell(X)
        batch = cat(channelDims,X{:,1});        
    end
catch
    % Return X as-is.
    batch = X(:,1);
end
out = {batch  X(:,2)};


%--------------------------------------------------------------------------
function iErrorIfAnyFutureFailed(futures)
failed = arrayfun(@(x)strcmpi(x.State,'failed'), futures);

if any(failed)
    % kill existing work and throw error.
    for i = 1:numel(futures)
        futures(i).cancel();
    end
    
    throw(futures(find(failed,1)).Error);
end

%--------------------------------------------------------------------------
function sz = iNetworkImageSize(net)
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
sz = net.Layers(i).InputSize;

%--------------------------------------------------------------------------
function I = iConvertImageToMatchNumberOfNetworkImageChannels(I, netImageSize)

if iIs2dNetwork(netImageSize)
    isNetImageRGB = numel(netImageSize) == 3 && netImageSize(end) == 3;
    isImageRGB    = size(I,3) == 3;
    
    if isImageRGB && ~isNetImageRGB
        I = rgb2gray(I);
        
    elseif ~isImageRGB && isNetImageRGB
        I = repmat(I,1,1,3);
    end
end

%--------------------------------------------------------------------------
function pool = tryToCreateLocalPool()
defaultProfile = ...
    parallel.internal.settings.ProfileExpander.getClusterType(parallel.defaultProfile());

if(defaultProfile == parallel.internal.types.SchedulerType.Local)
    % Create the default pool (ensured local)
    pool = parpool;
else
    % Default profile not local
    error(message('vision:vision_utils:noLocalPool', parallel.defaultProfile()));
end

%--------------------------------------------------------------------------
function TF = iAssertOpenPoolIsLocal()
pool = gcp('nocreate');
if isempty(pool)
    TF = false;
else
    if pool.Cluster.Type ~= parallel.internal.types.SchedulerType.Local
        error(message('vision:vision_utils:noLocalPool', pool.Cluster.Type));
    else
        TF = true;
    end
end

%--------------------------------------------------------------------------
function TF = iIs2dNetwork(netImageSize)

TF = numel(netImageSize) == 3;

%--------------------------------------------------------------------------
function out = pixelLabelMatFileReader(filename,values,classnames)

labelStruct = load(filename);
if iIfUint8OrLogicalImage(labelStruct.labelImage)
    out = labelStruct.labelImage;
else
    out = categorical(labelStruct.labelImage, values, classnames);
end

%--------------------------------------------------------------------------
function TF = iIfUint8OrLogicalImage(C)
TF = (isa(C,'uint8')||isa(C,'logical'));

%--------------------------------------------------------------------------
function out = iCopyDataIntoROIRegion(out, in, startROI, endROI, channelDim)
% iCopyDataIntoROIRegion - Function copies values in the variable, in to the
% output variable, out at the specified ROI indices for all the spatial
% dimensions. The channel dimension, channelDim is used to determine the
% number of spatial dimensions. The data in the remaining dimensions (i.e
% other than spatial dimensions) are copied as-is from in to out. The ROI
% indices are specified as two cell arrays, startROI and endROI containing
% the start indices and corresponding end indices respectively for each
% spatial dimension.

% Calculate number of spatial dimensions
numDims = channelDim-1;

outDims = ndims(out);
subscripts = cell(1,outDims);

% Use ROI subscripts for spatial dimensions
for idx = 1:numDims
    subscripts(idx) = {startROI{idx}:endROI{idx}};
end
% Use ':' for remaining dimensions
for idx = numDims+1:outDims
    subscripts(idx) = {':'};
end
S.type = '()';
S.subs = subscripts;

out = subsasgn(out,S,in);

%--------------------------------------------------------------------------
function [batchSize, isDataBatched] = iDetermineBatchSize(X,networkInputSize)
if iscell(X)
    batchSize = numel(X);
    isDataBatched = false;
else
    numNetImgDims = numel(networkInputSize);
    obsDim = numNetImgDims+1;
    batchSize = size(X,obsDim);
    isDataBatched = true;
end

%--------------------------------------------------------------------------
function loader = iCreateDataLoader(ds,params)
if params.NameSuffixSource == "filename"
    ds = transform(ds,@(data,info)iExtractFilename(data,info),'IncludeInfo',true);
else
    ds = transform(ds,@(data,info)iAddCustomSuffix(data,info,params.NameSuffix),'IncludeInfo',true);
end
loader = nnet.internal.cnn.DataLoader(ds,...
    'MiniBatchSize',params.MiniBatchSize,...
    'CollateFcn',@(x)iTryToBatchData(x,params.NetImageSize));

%--------------------------------------------------------------------------
function [data,info] = iExtractFilename(data,info)
if ~iscell(data)
    data = {data};
end
N = size(data,1);
if isfield(info,'Filename')
    try %#ok
        data = [data info.Filename];
    catch
        % Number of data and filename is not 1-to-1. Unable to add filename
        % as a suffix.
        data = [data repelem({''},N,1)];
    end
else
    data = [data repelem({''},N,1)];
end

%--------------------------------------------------------------------------
function [data, info] = iAddCustomSuffix(data,info,suffix)
if ~iscell(data)
    data = {data};
end
N = size(data,1);
data = [data repelem({suffix},N,1)];

%--------------------------------------------------------------------------
function n = iNumberOfObservations(ds)
if isa(ds,'matlab.io.datastore.ImageDatastore')
    n = numpartitions(ds);
else
    n = inf;
end

%--------------------------------------------------------------------------
function tf = iIsDatastore(x)
tf = isa(x,'matlab.io.Datastore') || isa(x,'matlab.io.datastore.Datastore');

%--------------------------------------------------------------------------
function ds = iAddFirstColumnSelectionTransform(ds)
% Add transform to select only first column of data when read(ds) returns
% anything other than a numeric or M-by-1 cell array.
ds = transform(ds,@iFirstColumnSelector);

%--------------------------------------------------------------------------
function data = iFirstColumnSelector(data)
iCheckDatastoreOutput(data);
isNumericOrCellVector = isnumeric(data) || (iscell(data) && size(data,2)==1);

if ~isNumericOrCellVector
    if iscell(data)   
        data = data(:,1);
    elseif istable(data)
        data = data{:,1};
    end
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
szC = size(Croi,1:inputChannelDim-1); 
szROI = size(Iroi,1:inputChannelDim-1);

if ~isequal(szC, szROI)
    error(message('vision:semanticseg:outputROISizeMismatch',...
        mat2str(szC),mat2str(szROI)));
end

%--------------------------------------------------------------------------    
function iCheckDatastoreOutput(data)
% Output of datastore must be a numeric, M-by-N cell, or M-by-N table.
cellOrTable = iscell(data) || istable(data);
invalidOutput = ~isnumeric(data) && ~(cellOrTable && numel(size(data))==2);
if invalidOutput
    error(message('vision:semanticseg:invalidDatastoreReadOutput'));
end

%--------------------------------------------------------------------------
function exeenv = iCheckGPUMismatch(exeenv)
if isequal(exeenv, 'gpu') && ~canUseGPU
    % Explicit gpu specification and no GPUs.
    error(message('nnet_cnn:internal:cnngpu:GPUArchMismatch'))
end
if canUseGPU && isequal(exeenv, 'auto')
    % GPU available and 'auto'.
    exeenv = 'gpu';
end

% LocalWords:  roi cuboidal pxds th readall pretrained labeloverlay BW visiondata DBrain Labeler
% LocalWords:  async grayscale imds GCP nocreate nonsparse Cls scalartext nonfinite DInput utils
% LocalWords:  Croi
