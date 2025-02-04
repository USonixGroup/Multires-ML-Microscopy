function [imds, roids, txtds] = ocrTrainingData(gTruth, labelName, attributeName)

% Copyright 2022-2023 The MathWorks, Inc.

    arguments
        gTruth          {mustBeVector, mustBeA(gTruth, "groundTruth")}
        labelName       {mustBeTextScalar, mustBeNonzeroLengthText}
        attributeName   {mustBeTextScalar, mustBeNonzeroLengthText}
    end

    nargoutchk(0,3);
    
    % Validate inputs.
    gTruth = validateInputs(gTruth, labelName, attributeName);
    
    % Extract text label.
    [textLabels, isEmptyText] = extractTextLabels(gTruth, labelName, attributeName);

    % Extract ROI labels.
    bboxLabels = extractROILabels(gTruth, labelName);

    % Remove empty labels.
    [bboxLabels, textLabels, isEmptyImage] = removeEmptyLabels(bboxLabels, textLabels, isEmptyText);

    % Extract valid images.
    imageSources = extractValidImageSources(gTruth, isEmptyImage, attributeName);

    % Construct datastores.
    [imds, roids, txtds] = constructDatastores(imageSources, bboxLabels, textLabels);
end

%--------------------------------------------------------------------------
function sources = extractValidImageSources(gTruth, isEmptyImage, attributeName)

    if all(isEmptyImage)
        error(message('vision:ocr:emptyGroundTruth', attributeName));
    end

    sources = cell(length(gTruth),1);
    for i = 1:length(gTruth)
        sources{i} = gTruth(i).DataSource.Source;
    end

    sources = vertcat(sources{:});
    sources(isEmptyImage) = [];
end


%--------------------------------------------------------------------------
function bboxLabels = extractROILabels(gTruth, labelName)

    bboxLabels = cell(length(gTruth),1);
    for i = 1:length(gTruth)
        labelData = gTruth(i).LabelData;
        bboxLabels{i} = extractBoundingBoxes(labelData, labelName);
    end

    bboxLabels = vertcat(bboxLabels{:});
end

%--------------------------------------------------------------------------
% extractBoundingBoxes extracts bounding boxes information from the
% labelInfo corresponding to the labelName.
%--------------------------------------------------------------------------
function bboxes = extractBoundingBoxes(labelInfo, labelName)
    
    bboxes = labelInfo.(labelName);
    
    for idx = 1:height(labelInfo)
        thisItem = labelInfo{idx, labelName};

        % thisItem is always a cell.
        thisItem = thisItem{1};
        
        if isstruct(thisItem)
            
            numROIs = length(thisItem);
            position = zeros(numROIs,4);
            
            for rIdx = 1:numROIs
                position(rIdx, :) = thisItem(rIdx).Position;
            end
            bboxes{idx} = position;
        end
    end
end

%--------------------------------------------------------------------------
function [textLabels, isEmptyText] = extractTextLabels(gTruth, labelName, attributeName)

    attributeInfo = cell(length(gTruth),1);
    for i = 1:length(gTruth)
        labelData = gTruth(i).LabelData;
        labelInfo = labelData.(labelName);
    
        collectAttribute = @(labelInfoForEachImage) collectAttributeImpl(labelInfoForEachImage, attributeName);
        attributeInfo{i} = cellfun(collectAttribute, labelInfo);
    end
    
    attributeInfo = vertcat(attributeInfo{:});
    textLabels = {attributeInfo.Value}';
    isEmptyText = {attributeInfo.IsEmptyValue}';
end

%--------------------------------------------------------------------------
function attributeInfo = collectAttributeImpl(labelInfoForEachImage, attributeName)
    
    % attributeInfo.Value will be a string vector of size numBboxes-by-1.
    attributeInfo.Value = string({labelInfoForEachImage.(attributeName)}');

    if isempty(attributeInfo.Value)
        % Image Labeler returns string.empty for the attribute if there
        % were no bounding boxes in an image.
        attributeInfo.IsEmptyValue = true;
    else
        attributeInfo.IsEmptyValue = strlength(attributeInfo.Value)==0;
    end
end

%--------------------------------------------------------------------------
% Helper to remove bounding box and text labels based on empty text values.
%
% Input:
%   bboxLabels  - numImages-by-1 cell array of numBboxes-by-4 vector.
%   textLabels  - numImages-by-1 cell array of numBboxes-by-1 string vector.
%   isEmptyText - numImages-by-1 cell array of numBboxes-by-1 logical vector.
%
% Output:
%   bboxLabels  - numValidImages-by-1 cell array of numValidBboxes-by-4 vector.
%   textLabels  - numValidImages-by-1 cell array of numValidBboxes-by-1 string vector.
%   isEmptyText - numImages-by-1 logical vector.
%--------------------------------------------------------------------------
function [bboxLabels, textLabels, isEmptyImage] = removeEmptyLabels(bboxLabels, textLabels, isEmptyText)

    % Find image indices that contain any bounding boxes with empty text.
    hasEmptyBbox = cellfun(@any, isEmptyText);
    imageIdx = find(hasEmptyBbox);

    % Iterate through label data of those images that contain bounding
    % boxes with empty text.
    for i = 1:length(imageIdx)
        idx = imageIdx(i);

        % Remove empty text labels.
        if ~isempty(textLabels{idx})
            textLabels{idx}(isEmptyText{idx}) = [];
        end
        
        % Remove bounding boxes associated to empty text labels.
        if ~isempty(bboxLabels{idx})
            bboxLabels{idx}(isEmptyText{idx}, :) = [];
        end
    end

    % Remove empty cell elements in bboxLabels.
    isBboxEmpty = cellfun(@isempty,bboxLabels);
    bboxLabels = bboxLabels(~isBboxEmpty);

    % Remove empty cell elements in textLabels.
    isTextEmpty = cellfun(@isempty,textLabels);
    textLabels = textLabels(~isTextEmpty);

    % Find images associated to empty bounding boxes and text labels.
    isEmptyImage = isBboxEmpty & isTextEmpty;
end

%--------------------------------------------------------------------------
function [imds, roids, txtds] = constructDatastores(imageSources, bboxLabels, textLabels)

    imds = imageDatastore(imageSources);
    txtds = arrayDatastore(textLabels, OutputType="same");
    roids = arrayDatastore(bboxLabels, OutputType="same");
end

%--------------------------------------------------------------------------
function gTruth = validateInputs(gTruth, labelName, attributeName)

    gTruth = validateGtruth(gTruth);

    validateLabelName(gTruth, labelName)
    
    validateAttributeName(gTruth, labelName, attributeName)
end

%--------------------------------------------------------------------------
function gTruth = validateGtruth(gTruth)

    % Discard empty data sources.
    invalidSource = arrayfun(@(x)~x.hasValidDataSource(),gTruth);
    if all(invalidSource)
        error(message('vision:ocr:invalidGroundTruth'));
    end
    gTruth(invalidSource) = [];

    % Check if any sources point to videos, image sequences or custom data sources.
    isVideo = arrayfun(@(x)x.DataSource.isVideoFileSource,gTruth);
    isImageSequence = arrayfun(@(x)x.DataSource.isImageSequenceSource,gTruth);
    isCustomSrc = arrayfun(@(x)x.DataSource.isCustomSource,gTruth);

    invalidGTruth = isVideo | isImageSequence | isCustomSrc;

    if all(invalidGTruth)
        error(message('vision:ocr:invalidGroundTruth'));
    end    
    gTruth(invalidGTruth) = [];
end

%--------------------------------------------------------------------------
function validateLabelName(gTruth, labelName)

    labeltype = labelType.Rectangle;
    availableLabelNames = vision.internal.inputValidation.checkGroundTruthLabelDefinitions(gTruth, labeltype);
    availableLabelNames = string(availableLabelNames);

    isLabelNameAvailable = any(availableLabelNames == labelName);
    
    if ~isLabelNameAvailable
        error(message('vision:ocr:NoLabel', labelName));
    end
end

%--------------------------------------------------------------------------
function validateAttributeName(gTruth, labelName, attributeName)

    for i = 1:numel(gTruth)
        hasValidationFailed = true;

        columnNames = string(gTruth(i).LabelDefinitions.Properties.VariableNames);

        % Attributes information is stored in the Hierarchy column of label
        % definitions.
        if any(columnNames == "Hierarchy")

            availableLabelNames = string(gTruth(i).LabelDefinitions.Name);
    
            if height(gTruth(i).LabelDefinitions) == 1
                % If there is only one label definition in the ground
                % truth, gTruth(i).LabelDefinitions.Hierarchy is a struct.
                hierarchy = gTruth(i).LabelDefinitions.Hierarchy;
                
                if iscell(hierarchy)
                    % A groundTruth object created programmatically with
                    % attributes, returns gTruth(i).LabelDefinitions.Hierarchy 
                    % as a cell of struct.
                    hierarchy = hierarchy{1};
                end
            else 
                % If there are mutliple label definitions in the ground
                % truth, gTruth(i).LabelDefinitions.Hierarchy is a cell
                % array of structs.
    
                % Get hierarchy corresponding to the queried label name.
                hierarchy = gTruth(i).LabelDefinitions.Hierarchy{availableLabelNames == labelName};
            end
            
    
            doesAttributeExist = isfield(hierarchy, attributeName);
            if doesAttributeExist

                attributeInfo = hierarchy.(attributeName);
                isSubLabel = isfield(attributeInfo, "Type");
                isAttributeListType = ~isfield(attributeInfo, "DefaultValue");
                if ~isSubLabel && ~isAttributeListType
                    isAttributeStringDatatype = ...
                        isstring(attributeInfo.DefaultValue) || ...
                        ischar(attributeInfo.DefaultValue);
                    
                    hasValidationFailed = ~isAttributeStringDatatype;
                end
            end
        end

        if hasValidationFailed
            error(message('vision:ocr:NoAttribute', labelName, attributeName, i));
        end
    end
end