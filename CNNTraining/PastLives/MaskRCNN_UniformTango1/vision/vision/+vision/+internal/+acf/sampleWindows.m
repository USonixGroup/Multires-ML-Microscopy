function [Is, IsOrig] = sampleWindows(ds, params, positive, detector, printer)
% Sample windows for training detector.
%
% ds       : A datastore that returns image, boxes, and labels.
% params   : struct of parameters
% positive : true if we want to sample positive examples, false for
%            negative examples
% 
% Is       : 4-dimension array (h x w x channel x k), each (h x w x channel)
%            block is a cropped/resized/jittered image
% IsOrig   : 4-dimension array, each block is an image without jittering

% This code is a modified version of that found in:
%
% Piotr's Computer Vision Matlab Toolbox      Version 3.23
% Copyright 2014 Piotr Dollar & Ron Appel.  [pdollar-at-gmail.com]
% Licensed under the Simplified BSD License [see pdollar_toolbox.rights]

% Copyright 2021-2022 The Mathworks, Inc.

narginchk(4, 5);
verbose = (nargin == 5);

numImages   = params.NumImages;
shrink      = params.Shrink;
modelDs     = params.ModelSize;
modelDsPad  = params.ModelSizePadded;
extPadSize  = max(2, ceil(64 / shrink)) * shrink;
modelDsBig  = max(8 * shrink, modelDsPad);

if all(extPadSize./modelDsBig >= 2)
    extPadSize = 2 * shrink;
end
modelDsBig  = modelDsBig + extPadSize;
ratio       = modelDsBig ./ modelDs;

if params.UseParallel
    pool = gcp('nocreate');
    if isempty(pool)
        tryToCreateLocalPool();
    end
end    

Is = cell(numImages*max(100,params.NumNegativePerImage), 1);
k = 0; 
i = 0; 

% A batch of 16 is fixed to trade-off memory usage and speed. The algorithm
% extracts data from 16 images and loads that into the batch.
batchSize = 16;

if positive
    n = params.NumPositiveSamples;
    if verbose
        printer.printMessageNoReturn('vision:acfObjectDetector:trainSamplePositive');
    end
else
    % It might not be able to retrieve n negative examples, either because
    % the number of negative examples per image is too few, or the detector
    % is too good to return any false alarms, or there are not enough
    % images.
    n = params.NumNegativeSamples;
    if verbose
        printer.printMessageNoReturn('vision:acfObjectDetector:trainSampleNegative');
    end
end

if verbose
    msg = [sprintf('(~%d%', 0),'%% Completed)'];
    printer.printDoNotEscapePercent(msg);
end

% Add a transform to preprocess the image data.
tds = transform(ds, @(data)preprocessImage(data));

% The sampling algorithm processes the list of images in batches. Define
% the number of partitions in the datastore based on the desired batch size.
% This ensures each datastore partition has roughly that many images in
% it. 
numPartitions = max(1,floor(numImages/batchSize));
part = 0;

% Datastores are not required to return a fixed number of elements so we
% cannot rely upon read(ds) return 1 images. Instead we divide the
% datastore using partition and then process the entire partition. After
% the processing, we form a fixed size batch of the results and maintain
% a surplus buffer to manage uneven partitions manually.
%
% N.B. This approach is required to replicate the previous version of this
% algorithm that supported table data where data was processed in exact
% batches of 16.
%
% Define a buffer to store the excess data.
surplusBuffer = {};
while (i < numImages && k < n)

    % Get the next partition from the datastore.
    part = part + 1;
    if part <= numPartitions
        subds = partition(tds, numPartitions, part);
    
    if params.UseParallel
            Is1 = extractSampleWindowsParallel(subds, positive, detector, params, ratio, modelDsBig);
        else
            Is1 = extractSampleWindowsSerial(subds, positive, detector, params, ratio, modelDsBig);
            end

    else
        % All the data has been read from the partitions. The loop
        % continues to process the remaining surplus.
        Is1 = {};
            end

    [Is1, surplusBuffer] = formBatch(Is1, surplusBuffer, batchSize);
    
    i = i + numel(Is1);

    Is1 = [Is1{:}]; 
    k1 = length(Is1); 
    Is(k + 1:k + k1) = Is1; 
    k = k + k1;
    if k > n
        Is = Is(vision.internal.samplingWithoutReplacement(k, n)); 
        k = n; 
    end

    if verbose
        nextMessage = [sprintf('(~%d%', round(100*k/n)),'%% Completed)'];
        msg = updateMessage(printer, numel(msg)-1, nextMessage);
    end
end
Is = Is(1 : k); 

if verbose
    if k ~= n 
        nextMessage = [sprintf('(~%d%', 100),'%% Completed)'];
        updateMessage(printer, numel(msg)-1, nextMessage);
    end
    printer.linebreak;
end

if length(Is) < 2 % make sure this returns a 4-D array
    Is = [];
    if nargout > 1
        IsOrig = Is;
    end
    return
end

nd = ndims(Is{1}) + 1; 
Is = cat(nd, Is{:});

if nargout > 1
    IsOrig = Is;
end

% optionally jitter positive windows
if params.MaxJitters > 0  
    Is = vision.internal.acf.jitterImage(Is, ...
        'MaxJitters', params.MaxJitters, ...
        'NumSteps', params.NumSteps, ...
        'Bound', params.Bound, ...
        'Flip', params.Flip, ...
        'HasChn', (nd==4));
    ds = size(Is); 
    ds(nd) = ds(nd) * ds(nd+1); 
    Is = reshape(Is, ds(1:nd));
end

% make sure dims are divisible by shrink and not smaller than modelDsPad
ds = size(Is); 
cr = rem(ds(1:2), shrink); 
s = floor(cr / 2) + 1;
e = ceil(cr / 2); 
Is = Is(s(1):end-e(1), s(2):end-e(2), :, :); 
ds = size(Is);
if any(ds(1:2) < modelDsPad)
    error(message('vision:acfObjectDetector:trainSampleWindowTooSmall')); 
end

%--------------------------------------------------------------------------
function [data, remainder] = formBatch(data, previousData, batchSize)
% Returns a batch of data of size batchSize by combining the input data and
% previous data and then selecting 1:batchSize elements and returning the
% remainder.
data = [previousData data];

if numel(data) > batchSize
    remainder = data(batchSize+1:end);
    data(batchSize+1:end) = [];
else
    remainder = {};
end

%--------------------------------------------------------------------------
function bboxes = sampleWins(I, gt, detector, params)
% Sample windows from I given its ground truth gt.
if isempty(detector)
    % generate candidate bounding boxes in a grid
    [h, w, ~] = size(I); 
    h1 = params.ModelSize(1); 
    w1 = params.ModelSize(2);
    n = params.NumNegativePerImage; 
    ny = sqrt(n*h/w); 
    nx = n/ny; 
    ny = ceil(ny); 
    nx = ceil(nx);
    [xs, ys] = meshgrid(linspace(1,w-w1,nx),linspace(1,h-h1,ny));
    bboxes = [xs(:) ys(:)]; 
    bboxes(:,3) = w1; 
    bboxes(:,4) = h1; 
    bboxes = bboxes(1:min(n, size(bboxes, 1)), :);
else
    % run detector to generate candidate bounding boxes
    P = vision.internal.acf.computePyramid(I, params);
    [bboxes, scores] = vision.internal.acf.detect(P, detector, params);
    [~, ord] = sort(scores, 'descend');
    bboxes = bboxes(ord(1:min(length(ord), params.NumNegativePerImage)), :);
end

if (~isempty(gt))
    % discard any candidate negative bb that matches the gt
    n = size(bboxes, 1); 
    keep = false(1, n);
    for i = 1 : n
        o = bboxOverlapRatio(bboxes(i,:), gt);
        keep(i) = all(o < 0.1); 
    end
    bboxes = bboxes(keep, :);
end

%--------------------------------------------------------------------------
function nextMessage = updateMessage(printer, prevMessageSize, nextMessage)
backspace = sprintf(repmat('\b',1,prevMessageSize)); % figure how much to delete
printer.printDoNotEscapePercent([backspace nextMessage]);

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
function data = preprocessImage(data)
for i = 1:size(data,1)
    I = data{i,1};
    % Rescale and convert grayscale to RGB, if needed.
if (isa(I, 'single') || isa(I, 'double') || isa(I, 'int16') || isa(I, 'uint16'))
    I = uint8(255 * rescale(I));
end
    if ismatrix(I)
        I = cat(3, I, I, I);
    end
    data{i,1} = I;
end

%--------------------------------------------------------------------------
function out = extractSampleWindowsSerial(ds, positive, detector, params, ratio, modelDsBig)
k = 1;
out = {};
while hasdata(ds)
    data = read(ds);
    for i = 1:size(data,1)
        I   = data{i,1};
        bbs = data{i,2};

        if ~positive
            bbs = sampleWins(I, bbs, detector, params);
        end

        bbs = vision.internal.acf.resizeBboxes(bbs, ratio(1), ratio(2));
        out{k} = vision.internal.acf.cropBboxes(I, bbs, 'replicate', modelDsBig([2 1])); %#ok<AGROW> 
        k = k + 1;
    end
end

%--------------------------------------------------------------------------
function out = extractSampleWindowsParallel(ds, positive, detector, params, ratio, modelDsBig)
n = numpartitions(ds);
out = cell(1,n);

% Create constant variable objects to prevent excessive data transfer
% between workers.
modelDsBigConst = parallel.pool.Constant(modelDsBig);
ratioConst = parallel.pool.Constant(ratio);
detectorConst = parallel.pool.Constant(detector);
paramsConst = parallel.pool.Constant(params);

parfor i = 1:n
    subds = partition(ds,n,i);
    [out{i}] = extractSampleWindowsSerial(subds, positive, detectorConst.Value, paramsConst.Value, ratioConst.Value, modelDsBigConst.Value);
end
out = horzcat(out{:});

