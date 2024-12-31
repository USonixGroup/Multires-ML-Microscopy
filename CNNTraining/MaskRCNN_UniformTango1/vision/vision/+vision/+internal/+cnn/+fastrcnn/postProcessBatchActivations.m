function [bboxes, scores, labels] = postProcessBatchActivations(features,regression,proposals,numFiles,numBatches,params)
%postProcessBatchActivations Post process a batch of activations.
%   Activations outputs are features, regression and proposal values
%   from the network.activationsMIMO method.

%   Copyright 2019 The MathWorks, Inc.

    bboxes = cell(numFiles, 1);
    scores = cell(numFiles, 1);
    labels = cell(numFiles, 1);
    startIndex = 0;
    for ii = 1:numBatches
        fmapBatch = features{ii};
        regBatch  = regression{ii};
        proposalBatch = proposals{ii};

        % proposal batch is numObs*numBoxes-by-5
        % Image indices are encoded in the 5th column of proposal boxes.
        imgIndices = proposalBatch(:,5);
        bboxBatch  = proposalBatch(:,1:4);
        maxIndex   = imgIndices(end);

        for jj = 1:maxIndex
            currentIndices = imgIndices==jj;

            % feature maps batch is 1-by-1-by-numClasses-by-numObs*numBoxes
            fmap  = fmapBatch(:,:,:,currentIndices);
            % regression batch is 1-by-1-by-4*numClasses-by-numObs*numBoxes
            reg   = regBatch(:,:,:,currentIndices);
            % bounding boxes batch is numObs*numBoxes-by-4 
            boxes = bboxBatch(currentIndices,:);

            asgnIdx = startIndex + jj;
            [bboxes{asgnIdx},scores{asgnIdx},labels{asgnIdx}] = ...
                fastRCNNObjectDetector.postProcessActivations(fmap, reg, boxes, params);
        end
        startIndex = startIndex + maxIndex;
    end
end
