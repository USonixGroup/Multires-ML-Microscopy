function [trainingData, trainingSamples, validTrainingData] = generateTrainingSamples(trainingData, rpnLayerGraph, params)
% Generate training data for RPN. This includes partitioning anchor boxes
% into positive and negative samples. 

% Also checks the trainingData table for valid ROIs. 

% Copyright 2016-2017 The MathWorks, Inc. 

params.RandomSelector  = vision.internal.rcnn.RandomSelector();

numfiles = height(trainingData);
isvalid(numfiles) = struct('HasValidData',[],'HasNoBoxes',[]);

% Find source of the RPN softmax layer.
analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(rpnLayerGraph);
externalLayers = [analysis.LayerAnalyzers.ExternalLayer];
idx = arrayfun(@(x)isa(x , 'nnet.cnn.layer.RPNSoftmaxLayer'), externalLayers);
rpnSoftmaxLayerSource = analysis.LayerAnalyzers(idx).Inputs.Source{1};


scaleImage = params.ScaleImage;
scale      = params.ImageScale;
gt = table2struct(trainingData);
% Get the name of the first column for use in struct access in parfor
% loop. This avoids assuming the first column has a specific name.
imgFileNameVar = trainingData.Properties.VariableNames{1};

N = numel(gt);
% Get the size of all the images.
sz(N) = struct('Size',[],'SizeString',[]);

if params.UseParallel
    parfor i = 1:N
        
        % scale image
        filename = gt(i).(imgFileNameVar);
        if scaleImage
            I = fastRCNNObjectDetector.scaleImage(filename, scale);
        else
            I = imread(filename);
        end
        sz(i).Size = size(I);
        sz(i).SizeString = iGenerateKeyForFeatureMapCache(sz(i).Size);
        
    end
else
    for i = 1:N
        
        % scale image
        filename = gt(i).(imgFileNameVar);
        if scaleImage
            I = fastRCNNObjectDetector.scaleImage(filename, scale);
        else
            I = imread(filename);
        end
        sz(i).Size = size(I);
        sz(i).SizeString = iGenerateKeyForFeatureMapCache(sz(i).Size);
        
    end
end

fmapSize = iComputeFeatureMapSize(rpnSoftmaxLayerSource, analysis, sz);

% Select training samples.
for i = 1:N
    imageSize = sz(i).Size;
    [trainingData(i,:), isvalid(i).HasValidData, isvalid(i).HasNoBoxes] = vision.internal.cnn.utils.hasValidTrainingData(imageSize,trainingData(i,:));
    
    c = table2cell(trainingData(i,:));
    s(i) = vision.internal.cnn.rpn.selectTrainingSamples(imageSize, fmapSize(i).Size, params, c{:});
end

trainingSamples   = struct2table(s, 'AsArray', true);

hasNoBoxes        = vertcat(isvalid(:).HasNoBoxes);
validTrainingData = ~hasNoBoxes & vertcat(isvalid(:).HasValidData);

% Remove rows with no data
trainingData(hasNoBoxes,:) = [];
trainingSamples(hasNoBoxes,:) = [];

if isempty(trainingData)
    error(message('vision:rcnn:noValidTrainingData'));
end

%--------------------------------------------------------------------------
function fmapSize = iComputeFeatureMapSize(rpnSoftmaxLayerSource, analysis, sz)
% Compute input feature map size for the RPN softmax layer. This
% feature maps size is used to compute the classification targets for
% training. We must compute the feature map size because RPN training
% support training with variable image sizes.
%
% This is done in serial to avoid sending large networks to the
% MATLAB workers.

% Determine network input size.
idx = [analysis.LayerAnalyzers.IsImageInputLayer];
networkInputSize = analysis.LayerAnalyzers(idx).Outputs.Size{1};

N = numel(sz);
fmapSize(N) = struct('Size',[]);

% Feature map size cache map. Keys are size strings. Avoids recomputing
% sizes for the same image sizes.
cache = containers.Map;

for i = 1:N
    if cache.isKey(sz(i).SizeString)
        % pull feature map size from cache.
        fmapSize(i).Size = cache(sz(i).SizeString);
    else
        
        % Preserve channel dim to prevent errors in case imageSize is
        % grayscale and network assumes RGB. gray to rgb conversion is
        % handled during inference. 
        imageSize = sz(i).Size;
        imageSize = [imageSize(1:2) networkInputSize(3:end)];
        
        fmapSize(i).Size = vision.internal.cnn.RCNNLayers.inferOutputSizesGivenImageInputSize(...
            rpnSoftmaxLayerSource, analysis.LayerGraph, imageSize);
        
        % Update cache.
        key = iGenerateKeyForFeatureMapCache(sz(i).Size);
        cache(key) = fmapSize(i).Size;
    end
end

%--------------------------------------------------------------------------
function key = iGenerateKeyForFeatureMapCache(sz)
key = string(sz).join("");
