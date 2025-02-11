function info = rcnnDatasetStatistics(datastore, layerGraph, params)
% Returns average image info and box regression mean and std, for a given groundTruth
% datastore.

% Copyright 2019 The MathWorks, Inc.

% Find source of the RPN softmax layer.
analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(layerGraph);

% Determine network input size.
idx = [analysis.LayerAnalyzers.IsImageInputLayer];
params.NetworkInputSize = analysis.LayerAnalyzers(idx).Outputs.Size{1};

params.StandardizeRegressionTargets = isempty(params.InternalOptions.BoxRegressionMeanStd);

if params.StandardizeRegressionTargets
    params.RPNSoftmaxLayerSource = iGetRPNSoftmaxLayerSource(analysis);

    % Feature map size cache map. Keys are size strings. Avoids recomputing
    % sizes for the same image sizes.
    params.Cache = containers.Map;
end

% For all the datastores, we have a isPartitionable method that depicts
% whether we can use partition and numpartition methods.
if isPartitionable(datastore) && params.UseParallel
    out = partitionAndGetInformation(datastore, params, layerGraph);
else
    out = readThroughAndGetInformation(datastore, params, layerGraph);
end
numImages = size(out.sizes, 1);
info.Sizes = out.sizes;
info.AverageImage = single(out.accum ./ numImages);

if params.StandardizeRegressionTargets
    [info.BoxRegressionMean, info.BoxRegressionStd] = vision.internal.cnn.utils.meanStdForTargets(out.targets);
else
    info.BoxRegressionMean = params.InternalOptions.BoxRegressionMeanStd(1,:);
    info.BoxRegressionStd = params.InternalOptions.BoxRegressionMeanStd(2,:);
end

%--------------------------------------------------------------------------
function info = partitionAndGetInformation(datastore, params, layerGraph)
N = numpartitions(datastore,gcp);
infos = cell(N, 1);
parfor ii = 1:N
    subds = partition(datastore, N, ii);
    infos{ii} = readThroughAndGetInformation(subds, params, layerGraph);
end
infos = vertcat(infos{:});
info.sizes = vertcat(infos.sizes);
info.accum = sum(vertcat(infos.accum));
info.targets = vertcat(infos.targets);

%--------------------------------------------------------------------------
function info = readThroughAndGetInformation(datastore, params, layerGraph)
accum = zeros(1,1,params.NetworkInputSize(3));
% figure out scaling for each image
k = 1;
isAnImageDatastore = isa(datastore, 'matlab.io.datastore.ImageDatastore');
if isAnImageDatastore
    numImages = numpartitions(datastore);
    if numImages > 1
        datastore.ReadSize = min(10, numImages);
    end
end

targets = {};

reset(datastore);
while hasdata(datastore)
    batch = read(datastore);
    batchSize = size(batch, 1);
    for i = 1:batchSize
        I = batch{i, 1};

        % Return size information for each image.
        [M, N, ~] = size(I);
        sz(k,:) = [M N];

        if params.StandardizeRegressionTargets
            fmapSize = iComputeFeatureMapSizeForImageSize(layerGraph,sz(k,:),params);
            t = vision.internal.cnn.rpn.generateTargetsFromBoxes(batch{i,2}, sz(k,:), fmapSize, params);
            % TODO: Make this efficient, with amortized allocation.
            targets = vertcat(targets, t);
        end
        % Compute per channel mean
        accum = accum + vision.internal.cnn.utils.perChannelMean(I, params.NetworkInputSize);

        k = k + 1;
    end
end
info.sizes = sz;
info.accum = accum;
info.targets = targets;

%--------------------------------------------------------------------------
function fmapSize = iComputeFeatureMapSizeForImageSize(layerGraph, imageSize, params)
% Compute input feature map size for the RPN softmax layer. This
% feature maps size is used to compute the classification targets for
% training. We must compute the feature map size because RPN training
% support training with variable image sizes.
imageSize = imageSize(1:2);
key = iGenerateKeyForFeatureMapCache(imageSize);
if params.Cache.isKey(key)
    % pull feature map size from cache.
    fmapSize = params.Cache(key);
else
    % Preserve channel dim to prevent errors in case imageSize is
    % grayscale and network assumes RGB. gray to rgb conversion is
    % handled during inference.
    imageSize = [imageSize params.NetworkInputSize(3:end)];

    fmapSize = vision.internal.cnn.RCNNLayers.inferOutputSizesGivenImageInputSize(...
        params.RPNSoftmaxLayerSource, layerGraph, imageSize);

    % Update cache.
    params.Cache(key) = fmapSize;
end

%--------------------------------------------------------------------------
function rpnSoftmaxLayerSource = iGetRPNSoftmaxLayerSource(analysis)
externalLayers = [analysis.LayerAnalyzers.ExternalLayer];
idx = arrayfun(@(x)isa(x , 'nnet.cnn.layer.RPNSoftmaxLayer'), externalLayers);
rpnSoftmaxLayerSource = analysis.LayerAnalyzers(idx).Inputs.Source{1};

%--------------------------------------------------------------------------
function key = iGenerateKeyForFeatureMapCache(sz)
key = string(sz).join("");
