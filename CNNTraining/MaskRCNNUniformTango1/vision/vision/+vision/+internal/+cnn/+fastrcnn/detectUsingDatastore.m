function detectionResults = detectUsingDatastore(ds, network, dataMap, layerOut, params)
%DETECTUSINGDATASTORE Obtain detected bboxes, scores and labels of images in a datastore.
%   detectionResults = detectUsingDatastore(ds, network, dataMap, layerOut, params)
%   detects objects within the series of images returned by the read method of
%   datastore, DS. DS, can be a datastore that returns a table or a cell array with the first
%   column containing images. detectionResults is a 3-column table with
%   variable names 'Boxes', 'Scores', and 'Labels' containing bounding
%   boxes, scores, and the labels.
%
%   See also fastRCNNObjectDetector/detect, fasterRCNNObjectDetector/detect.

%   Copyright 2019-2021 The MathWorks, Inc.
    try
        [features, regression, proposals] = network.activationsMIMO(ds,...
            dataMap, layerOut, ...
            'ExecutionEnvironment', params.ExecutionEnvironment,...
            'MiniBatchesInCell', true,...
            'MiniBatchSize', params.MiniBatchSize);
    catch ME
        if strcmp(ME.identifier,'nnet_cnn:internal:cnn:GeneralDatastoreDispatcher:VariableInputSizes')
            error(message('vision:ObjectDetector:unableToBatchImagesForDetect'));
        else
            rethrow(ME);
        end
    end

    numBatches = numel(features);
    numFiles = iFindNumFiles(proposals);
    [bboxes, scores, labels] = vision.internal.cnn.fastrcnn.postProcessBatchActivations(...
        features,regression,proposals,numFiles,numBatches,params);
    varNames = {'Boxes', 'Scores', 'Labels'};
    detectionResults = table(bboxes, scores, labels, 'VariableNames', varNames);
end

%-----------------------------------------------------------------------
function numFiles = iFindNumFiles(bboxes)
    numFiles = 0;
    for ii = 1:numel(bboxes)
        numFiles = numFiles + bboxes{ii}(end,5);
    end
end
