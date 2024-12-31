function metrics = evaluateInstanceSegmentation(dsResults, dsTruth, threshold, options)
%evaluateInstanceSegmentation returns evaluation metrics for instance
% segmentation.
%
% metrics = evaluateInstanceSegmentation(dsResults,dsTruth) computes
% various metrics to evaluate the quality of the instance segmentation
% results. dsResults represents the results of instance segmentation and
% holds the predicted object masks with their labels and scores. dsTruth 
% represents the ground truth instance segmentation and holds the true
% object masks along with their labels.
%
% The first argument dsResults is a predicted datastore which returns the
% following outputs in any order-
%         Cell 1  : Predicted logical object masks for M objects as a
%                   H-by-W-by-M logical array.
%         Cell 2  : Predicted object labels as a Mx1 categorical vector.
%
%         Cell 3  : Prediction scores as a Mx1 numeric vector.
%
% The second argument dsTruth is the groundTruth datastore which returns 2
% the following two outputs in any order - 
%         Cell 1  : Ground truth logical object masks for M objects as a
%                   H-by-W-by-M logical array.
%         Cell 2  : Ground truth object labels as a Mx1 categorical vector.
% The dsTruth dataStore can return up to 4 outputs. 
% 
% In both dsResults and dsTruth, the first categorical output will be 
% treated as the label data, the first output of type logical
% will be treated as object masks and the first numeric output will be
% treated as probability scores. 
%
% The output metrics is a instanceSegmentationMetrics object.
%
% instanceSegmentationMetrics object properties:
%  
%      * metrics.ConfusionMatrix is the confusion matrix for the classes in
%        the data set. For a scalar threshold value, It is a square table 
%        where element (i,j) is the count of objects known to belong to 
%        class i but predicted to belong to class j. For a vector threshold
%        the confusion matrix is a numClasses-by-numClasses-by-numThreshold
%        where each threshold value has it's own confusion matrix.
% 
%      * metrics.NormalizedConfusionMatrix is the confusion matrix normalized
%        by the number of objects known to belong to each class. Element (i,j)
%        represents the fraction, in [0,1], of pixels known to belong to
%        class i but predicted to belong to class j. For a vector threshold
%        the norm confusion matrix is a numClasses-by-numClasses-by-numThreshold
%        where each threshold value has it's own normalized confusion matrix.
%
%      * metrics.DataSetMetrics contains the metrics computed over the data
%       set. The following metrics are returned with the default threshold
%       values:
%       
%       - mAP : Mean average precision is average precision values computed at 
%               the default threshold range and averaged over all the classes.
%       - AP  : Average precision computed at all the overlap thresholds 
%               specified in threshold argument as a numThresh-by-numObjects array.
%
%       In cases where the threshold input is provided, these metrics are
%       replaced with AP values at the input overlap thresholds.
%
%       * metrics.ClassMetrics contains the metrics computed for each class.
%         The output here is a numClasses-by-numMetrics table. The
%         following matrics are returned with the default threshold values:
%      
%         - mAP       : Average precision computed for each class across the
%                       dataset.
%         - AP        : Average precision computed at all the overlap thresholds 
%                       specified in threshold argument as a numThresh-by-1 array.
%         - Precision : The precision values as a numThresh-by-(numObjects+1) numeric
%                       array.
%         - Recall    : The recall values as a numThresh-by-(numObjects+1) numeric
%                       array.
%
%       * metrics.ImageMetrics contains the metrics computed for each
%         image in the dataset. This is returned as a numImages-by-numMetrics
%         table. The following metrics are returned with the default threshold
%         values:
%       
%         - mAP : Mean average precision is average precision values computed at 
%                 the default threshold range and averaged over all the classes.
%         - AP  : Average precision computed at all the overlap thresholds 
%                 specified in threshold argument as a numThresh-by-1 array. 
%
%       * metrics.ClassNames contains a vector of string holding the class names of
%         the objects which are segmented.
%
%       * metrics.OverlapThreshold contains a vector of mask overlap threshold values
%         over which the mean average precision(mAP) is computed.
%
% [...] = evaluateInstanceSegmentation(..., threshold) specifies the overlap
% threshold for assigning a object mask to a ground truth mask. The overlap
% ratio is computed as the intersection over union of the two logical masks. 
% The threshold values can be specified in one of the following formats -
%     1. Scalar -   The Average Precision(AP) values are computed with the  
%                   scalar threshold values used to find the true
%                   assignments.
%     2. Vector -   The AP metrics are computed at each of the threshold 
%                   in the vector and returned as an average across all the
%                   threshold values in the vector.
%
% Default : 0.5
% 
% [...] = evaluateInstanceSegmentation(..., name=value) 
%
% The following name-value pair arguments provide additional options to configure the 
% evaluation.
%        
%     'Verbose'           Set true to display evaluation progress information.
%  
%                         Default: true

%   Copyright 2022-2023 The MathWorks, Inc.

arguments
    dsResults (1,1) {iValidateResultsDS}
    dsTruth (1,1){iValidateTruthDS}
    threshold (1,:) {mustBeNumeric, mustBeReal, mustBeInRange(threshold,0,1),...
        mustBeNonempty, iValidateUniqueThresholds} = 0.5
    options.Verbose (1,1) {validateLogicalFlag} = false
end

% Handle random ordering of outputs. Look for 
% 1. First logical output = Mask
% 2. First categorical vector = Label
% 3. First numeric vector = score
dsResultsCopy = copy(dsResults);
dsTruthCopy = copy(dsTruth);

predOutIdxs = iFindOutputIdxsFromDS(dsResultsCopy, 3, "dsResults");
truthOutIdxs = iFindOutputIdxsFromDS(dsTruthCopy, 2, "dsTruth");

dsRes = transform(dsResults, @(x)vision.internal.reorderDatastoreOutputs(x, predOutIdxs));
dsTrue = transform(dsTruth, @(x)vision.internal.reorderDatastoreOutputs(x, truthOutIdxs));

metrics = instanceSegmentationMetrics(dsRes, dsTrue, threshold, ...
                                      false, options.Verbose);

end

function outIdxs = iFindOutputIdxsFromDS(ds, numOutputs, dsName)
% Return indices corresponding to masks, labels and scores from each ds
% output.

    % Holds [maskIdx, labelIdx, scoreIdx] for dsResults and [maskIdx, labelIdx]
    % for dsTruth
    outIdxs = zeros(1, numOutputs); 
    sample = preview(ds);
    
    for pIdx = 1:numel(sample)
        if(isMask(sample{pIdx}) && outIdxs(1)==0)
            outIdxs(1) = pIdx;
        elseif(isLabel(sample{pIdx}) && outIdxs(2)==0)
            outIdxs(2) = pIdx;
        elseif(numOutputs==3 && isScore(sample{pIdx}) && outIdxs(3)==0)
            outIdxs(3) = pIdx;
        end
    end
    
    if(nnz(outIdxs)<numOutputs) % Not all outputs found
        outputNames = ["Masks", "Label", "Scores"];
        missingOutputs = join(outputNames(outIdxs==0), " and ");
        
        error(message('vision:evaluateInstanceSegmentation:missingDSOutputs', dsName, missingOutputs));
    end
end


function iValidateResultsDS(in)
    if(~matlab.io.datastore.internal.shim.isDatastore(in))
        error(message('vision:evaluateInstanceSegmentation:invalidResultDS'));
    end

    out = preview(in);
    if(~iscell(out) || numel(out) < 3)
        error(message('vision:evaluateInstanceSegmentation:invalidDSNumOutputs', 'resultsDS', 'three'));
    end
end

function iValidateTruthDS(in)
    if(~matlab.io.datastore.internal.shim.isDatastore(in))
        error(message('vision:evaluateInstanceSegmentation:invalidTruthDS'));
    end

    out = preview(in);
    if(~iscell(out) || numel(out) < 2)
        error(message('vision:evaluateInstanceSegmentation:invalidDSNumOutputs', 'truthDS', 'two'));
    end
end

function iValidateUniqueThresholds(in)
    if numel(in) ~= numel(unique(in))
        error(message('vision:evaluateInstanceSegmentation:invalidDuplicateThresholds'))
    end
end

function TF = isMask(in)
    if(islogical(in)&&ndims(in)<=3)
        TF = true;
    else
        TF = false;
    end
end

function TF = isLabel(in)
    if(iscategorical(in)&&(isvector(in)||isempty(in)))
        TF = true;
    else
        TF = false;
    end
end

function TF = isScore(in)
    if(isnumeric(in)&&(isvector(in)||isempty(in)))
        TF = true;
    else
        TF = false;
    end
end

function validateLogicalFlag(in)
    validateattributes(in,{'logical'}, {'scalar','finite', 'real'});
end
