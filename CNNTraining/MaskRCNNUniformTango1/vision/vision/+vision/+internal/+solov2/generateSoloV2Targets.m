function  [mlvlMaskPredsBatch, mlvlPosMaskTargetsBatch, mlvlLabelsTargetsBatch, mlvlPosMasksBatch] = generateSoloV2Targets(mlvlKernelPreds, mlvlClsPreds, maskFeats, gtLabels, gtMasks, gtBoxes, objSizeRanges, gridSizes, numClasses)
% Generate SoloV2 targets for training 
%
% Input:
% mlvlKernelPreds - Kernel predictions at each FPN level. numLvlx1 cell.
%                   Each cell holds numGridxnumGridxnumFeaturesxnumBatch
%                   tensor.
% mlvlClsPreds    - Classification scores at each FPN level. numLvlx1 cell.
%                   Each cell holds numGridxnumGridxnumClassesxnumBatch
%                   tensor.
% maskFeats       - Unified Mask features representation. Feat_h x Feat_w x
%                   numFeatures. Feat_h & Feat_w is the largest FPN
%                   resolution.
% gtMasks         - HxWxnumGT logical
% gtLabels        - numGTx1 categorical
% gtBoxes         - numGTx4 numeric: [x y h w] format
% objSizeRanges   - numLvls x2
% gridSizes       - numLvlsx2
%
% Output:
% mlvlMaskPredsBatch     - Feat_h x Feat_w x numPos mask prediction. All the 
%                          predictionsfrom the batches are flattened along the 
%                          third dimension.
% mlvlPosMaskTargetsBatch- Feat_h x Feat_w x numPos assigned mask targets. 
%                          Arragned in the same order as the predictions.
% mlvlLabelsTargetsBatch - numLvlsx1 cell-array. Each cell contains a
%                          numGridxnumGridx1xnumBatch
%                          list of assigned labels. with a numClasses+1
%                          label used for background.
% mlvlPosMasksBatch      - numLvlsx1 cell-array. Each cell contains a numGrid^2 x 1
%                          boolean list indicating postive samples along linear indices.

featSize = size(maskFeats);

batchSz = length(gtLabels);

mlvlMaskPredsBatch = [];
mlvlPosMaskTargetsBatch = [];
mlvlLabelsTargetsBatch = cell(length(mlvlKernelPreds),1); 
mlvlPosMasksBatch = [];

for batchIdx = 1:batchSz
    
    feats = maskFeats(:,:,:,batchIdx);
    featSize = size(feats);
    % Pick out the batchIdx-th index from each prediction level
    kernelPreds = cellfun(@(x)x(:,:,:,batchIdx),mlvlKernelPreds, 'UniformOutput',false);
    clsPreds = cellfun(@(x)x(:,:,:,batchIdx),mlvlClsPreds, 'UniformOutput',false);

    currGtlabels = gtLabels{batchIdx};
    currGtMasks = gtMasks{batchIdx};
    currgtBoxes = gtBoxes{batchIdx};

    % Step 1: Label assignment
    [mlvlPosIdx, mlvlPosMaskTargets, mlvlLabels, mlvlPosMasks] =...
            vision.internal.solov2.assignSolov2Labels(...
            currGtMasks, currGtlabels, currgtBoxes, featSize, objSizeRanges, gridSizes, numClasses);


    % Step 2: Filter and create multi-level kernel preds with positive
    % preds
    mlvlPoskernelPreds = [];

    for lvl = 1:length(mlvlPosIdx)
        
        lvlPosIdx = mlvlPosIdx{lvl};
        lvlKernelpreds = kernelPreds{lvl}; 

        %Reshape and flatten kernels from SSC to 1 1 C S*S
        % 1. SSC->S*S x C 
        lvlKernelpreds = reshape(lvlKernelpreds, [], size(lvlKernelpreds,3));
        lvlKernelpreds = permute(lvlKernelpreds, [3 4 2 1]);
         
        % Append the positive kernels at each level to a flattened list of
        % all kernels
        mlvlPoskernelPreds = cat(4,mlvlPoskernelPreds, lvlKernelpreds(:,:,:,lvlPosIdx));

    end

    % Step 3: Generate mlvl mask predictions for all positive kernels
    if(~isempty(mlvlPoskernelPreds))
        mlvlMaskPreds = dlconv(feats, mlvlPoskernelPreds, 0);
    else
        mlvlMaskPreds = [];
    end

    
    % Accumulate targets across batches
    mlvlMaskPredsBatch = cat(3,mlvlMaskPredsBatch,mlvlMaskPreds);

    mlvlPosMaskTargets = cat(3,mlvlPosMaskTargets{:});
    mlvlPosMaskTargetsBatch = cat(3,mlvlPosMaskTargetsBatch, mlvlPosMaskTargets);
    mlvlLabelsTargetsBatch = cellfun(@(x,y)cat(4, x, y), mlvlLabelsTargetsBatch,mlvlLabels, 'UniformOutput',false);

    mlvlPosMasksBatch = cat(1,mlvlPosMasksBatch,mlvlPosMasks{:});

end

end