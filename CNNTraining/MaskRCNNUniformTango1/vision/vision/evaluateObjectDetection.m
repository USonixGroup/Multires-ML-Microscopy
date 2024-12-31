function metrics = evaluateObjectDetection(detectionResults, groundTruthData, threshold, options)

%   Copyright 2023 The MathWorks, Inc.

arguments
    detectionResults (:,:) {iValidateResultsTable} 
    groundTruthData (:,:) {iValidateTruth} 
    threshold (1,:) {mustBeNumeric, mustBeReal, mustBeInRange(threshold,0,1),... 
        mustBeNonempty,iValidateUniqueThresholds} = 0.5
    options.Verbose (1,1) {validateLogicalFlag} = false
    options.AdditionalMetrics (1,:) string {iValidateAdditionalMetrics} = string.empty
end
options.AdditionalMetrics = unique(upper(options.AdditionalMetrics));

dsResults = arrayDatastore(table2cell(detectionResults),"OutputType","same");
if istable(groundTruthData)
    groundTruthData = arrayDatastore(table2cell(groundTruthData),"OutputType","same");
end

% Handle random ordering of outputs. Look for 
% 1. First numeric array of size Mx4 or Mx5 = bbox
% 2. First categorical vector = Label
% 3. First numeric vector of size Mx1 = score
dsResultsCopy = copy(dsResults);
dsTruthCopy = copy(groundTruthData);

predOutIdxs = iFindOutputIdxsFromDS(dsResultsCopy, 3, "detectionResults");
truthOutIdxs = iFindOutputIdxsFromDS(dsTruthCopy, 2, "groundTruthData");

dsRes = transform(dsResults, @(x)vision.internal.reorderDatastoreOutputs(x, predOutIdxs));
dsTrue = transform(groundTruthData, @(x)vision.internal.reorderDatastoreOutputs(x, truthOutIdxs));

if any(strcmpi(options.AdditionalMetrics,"AOS"))
    sample = preview(dsRes);
    if size(sample{1},2) ~= 5
        error(message('vision:evaluateObjectDetection:unavailableAOS'));
    end
end

% Handle empties in predictions and ground truth
[dsRes, dsTrue] = iProcessEmpties(dsRes, dsTrue);

metrics = objectDetectionMetrics(dsRes, dsTrue, threshold, ...
                                      false, options.Verbose,options.AdditionalMetrics);

end

function outIdxs = iFindOutputIdxsFromDS(ds, numOutputs, dsName)
% Return indices corresponding to bboxes, labels and scores from each ds
% output.

    % Holds [bboxIdx, labelIdx, scoreIdx] for dsResults and [bboxIdx, labelIdx]
    % for dsTruth
    outIdxs = zeros(1, numOutputs); 
    sample = read(ds);
    
    for pIdx = 1:numel(sample)
        if(isBBox(sample{pIdx}) && outIdxs(1)==0)
            outIdxs(1) = pIdx;
        elseif(isLabel(sample{pIdx}) && outIdxs(2)==0)
            outIdxs(2) = pIdx;
        elseif(numOutputs==3 && isScore(sample{pIdx}) && outIdxs(3)==0)
            outIdxs(3) = pIdx;
        end
    end
    
    if(nnz(outIdxs)<numOutputs) % Not all outputs found
        outputNames = ["Bounding Boxes", "Label", "Scores"];
        missingOutputs = join(outputNames(outIdxs==0), " and ");
        
        error(message('vision:evaluateObjectDetection:missingDSOutputs', dsName, missingOutputs));
    end
end

function bboxSize = iGetBoxSize(dsRes, dsTrue)
    % Iterate over both predictions and truths datastores to find out the
    % first non-empty box and determine the box size. The iteration exits
    % at the first occurrence of a non-empty box.
    bboxSize = 4;
    while hasdata(dsRes)
        % check boxes in dsRes for non-empty boxes
        dataRes = read(dsRes);
        bbox = dataRes{:,1};
        if ~isempty(bbox)
            bboxSize = size(bbox,2);
            break
        end

        % check dsTrue, only when dsRes has returned empty boxes
        if hasdata(dsTrue)
            dataTrue = read(dsTrue);
            bbox = dataTrue{:,1};
            if ~isempty(bbox)
                bboxSize = size(bbox,2);
                break
            end
        end
    end
end

function [dsRes, dsTrue] = iProcessEmpties(dsRes, dsTrue)
    % Create a correctly-sized empty box
    boxSize = iGetBoxSize(dsRes, dsTrue);
    sizedEmptyBox = single.empty(0,boxSize);

    % Replace incorrectly-sized empty boxes in prediction and ground truth
    function x = processEmptyBoxes(x)
        if isempty(x{1}) & (size(x{1},2) ~= boxSize)
            x{1} = sizedEmptyBox;
        end
    end
    dsRes = transform(dsRes, @(x)processEmptyBoxes(x));
    dsTrue = transform(dsTrue, @(x)processEmptyBoxes(x));
end

function iValidateResultsTable(in)
    if(~istable(in))
        error(message('vision:evaluateObjectDetection:invalidResultTable'));
    end

    if(width(in) < 3)
        error(message('vision:evaluateObjectDetection:invalidTableNumVariables','detectionResults','three'));
    end
end

function iValidateTruth(in)
    if(matlab.io.datastore.internal.shim.isDatastore(in))
        out = read(in);
        in.reset();
        if(~iscell(out) || numel(out) < 2)
            error(message('vision:evaluateObjectDetection:invalidDSNumOutputs','groundTruthData','two'));
        end

    elseif(istable(in))
        if(width(in) < 2)
            error(message('vision:evaluateObjectDetection:invalidTableNumVariables','groundTruthData','two'));
        end
    else
        error(message('vision:evaluateObjectDetection:invalidGroundTruth'));
    end    
end

function iValidateAdditionalMetrics(in)
    validMetrics = ["LAMR" "AOS"];
    for idx = 1:numel(in)
        if ~any(strcmpi(in(idx),validMetrics))
            error(message('vision:evaluateObjectDetection:invalidAdditionalMetrics'))
        end
    end
end

function iValidateUniqueThresholds(in)
    if numel(in) ~= numel(unique(in))
        error(message('vision:evaluateObjectDetection:invalidDuplicateThresholds'))
    end
end

function TF = isBBox(in)
    if(isnumeric(in)&&ismatrix(in))
        ncol = size(in,2);
        if (ncol==4||ncol==5)
            TF = true;
        else
            TF = false;
        end
    else
        TF = false;
    end
end

function TF = isLabel(in)
    if(iscategorical(in)&&ismatrix(in))
        if (size(in,2)==1)
            TF = true;
        else
            TF = false;
        end
    else
        TF = false;
    end
end

function TF = isScore(in)
    if(isnumeric(in)&&ismatrix(in))
        if (size(in,2)==1)
            TF = true;
        else
            TF = false;
        end
    else
        TF = false;
    end
end

function validateLogicalFlag(in)
    validateattributes(in,{'logical'}, {'scalar','finite', 'real'});
end