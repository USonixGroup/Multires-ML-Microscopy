function varargout = objectDetectorTrainingData(varargin)

% Copyright 2017-2023 The MathWorks, Inc.

nargoutchk(0,3);
[gTruth, samplingFactor, writeParams, idxStruct, labelNames, labeltype] = validateInputs(varargin{:});

sampleIndices = sampleDataSource(gTruth, labelNames, samplingFactor);

imageNames = cell(size(gTruth));
imageNames(idxStruct.isVideoOrCustomOrDSSource) = vision.internal.trainingData.writeImages(gTruth(idxStruct.isVideoOrCustomOrDSSource), writeParams, sampleIndices(idxStruct.isVideoOrCustomOrDSSource));

[trainingData, attributeAndSublabelTable, hasAttributes, emptyArrayDS] = populateTrainingTable(gTruth, labelNames, sampleIndices, imageNames, idxStruct, labeltype);

if isempty(trainingData)
    error( message('vision:objectDetectorTrainingData:emptyObjectDetectorData') )
end
if nargout <= 1
    if idxStruct.useCustomReader
        error(message('vision:objectDetectorTrainingData:tableOutputNotSupported'));
    else
        % Attributes are dropped in tabular output.
        if hasAttributes
            warning(message('vision:objectDetectorTrainingData:droppingAttributeData'));
        end
        varargout{1} = trainingData;
    end
else
    if idxStruct.useCustomReader
        [varargout{1:2}] = iConstructDatastoresCustomReader(trainingData, gTruth);
    else
        [varargout{1:2}] = iConstructDatastores(trainingData);
    end
end
if nargout==3
    % If input data has no attrubutes or sublabels the ouput arraydatastore 
    % has all empty structs.
    if emptyArrayDS
        warning(message('vision:objectDetectorTrainingData:emptyAttributeSublabelData'));
    end

    varargout{3} = arrayDatastore(attributeAndSublabelTable.Var1,'OutputType','same');

end
end

%--------------------------------------------------------------------------
function [imds, blds] = iConstructDatastores(trainingData)
files     = trainingData{:, 1};
boxLabels = trainingData(:, 2:end);

imds = vision.internal.customizedImageDatastore(files);

blds = boxLabelDatastore(boxLabels);
end

%--------------------------------------------------------------------------
function [imds, blds] = iConstructDatastoresCustomReader(trainingData, gTruth)
files     = trainingData{:, 1};
boxLabels = trainingData(:, 2:end);

% File extensions for the image files in the datastore
[~,~,fileExtensions] = fileparts(files);
fileExtensions = unique(fileExtensions);

imds = imageDatastore(files, 'FileExtensions', fileExtensions);
imds.ReadFcn = gTruth(1).DataSource.Source.ReadFcn;
blds = boxLabelDatastore(boxLabels);
end

%--------------------------------------------------------------------------
function [gTruth, samplingFactor, writeParams, idxStruct, labelNames, labeltype] = validateInputs(varargin)

inputs = parseInputs(varargin{:});

gTruth                  = inputs.gTruth;
samplingFactor          = inputs.SamplingFactor;
writeParams.Location    = inputs.WriteLocation;
writeParams.Prefix      = inputs.NamePrefix;
writeParams.Format      = inputs.ImageFormat;
writeParams.Verbose     = inputs.Verbose;
writeParams.UseParallel = inputs.UseParallel;
labeltype               = inputs.LabelType;


[gTruth, idxStruct, validGTruthIndices, samplingFactor] = ...
    vision.internal.trainingData.checkGroundTruthSources(gTruth, writeParams.Location, samplingFactor);

labelNames = vision.internal.inputValidation.checkGroundTruthLabelDefinitions(gTruth, labeltype, validGTruthIndices);

end


%--------------------------------------------------------------------------
function sampleIndices = sampleDataSource(gTruth, labelNames, samplingFactor)
%sampleDataSource sample data source from a groundTruth array
%
%   Inputs
%   ------
%   gTruth          - Array of groundTruth objects with valid (non-empty)
%                     data sources.
%   samplingFactor  - Scalar greater than 1 representing the down sample
%                     factor.
%
%   Outputs
%   -------
%   sampleIndices   - Cell array of the same size as gTruth, containing a
%                     list of indices to sample from gTruth.
%

labelDatas     = {gTruth.LabelData};
samplingFactor =  num2cell(samplingFactor);

sampleIndices = cellfun(@(data, sf)computeSampleIndices(data, labelNames, sf), labelDatas, samplingFactor, 'UniformOutput', false);
end


%--------------------------------------------------------------------------
function samples = computeSampleIndices(labelData, labelNames, samplingFactor)
%computeSampleIndices compute indices to sample from non-empty label data
%
%   Inputs
%   ------
%   labelData       - table or timetable of label data from a single
%                     groundTruth object.
%   labelNames      - cell array of label names for Rectangle label types.
%   samplingFactor  - scalar greater than 1 representing the down sample
%                     factor.
%
%   Outputs
%   -------
%   samples         - row indices into label data to be sampled for
%                     training.
%

labelData = labelData(:, labelNames);

% Find non-empty rows, which can be used for training.
areAllNotEmpty = @(varargin) ~all( cellfun(@isempty, varargin) );
validIndices = find(rowfun(areAllNotEmpty, labelData, 'ExtractCellContents', true, 'OutputFormat', 'uniform'));

% Sample among these rows
samples = validIndices( 1 : samplingFactor : end );
end


%--------------------------------------------------------------------------
function [trainingData,attributeAndSublabelTable, hasAttributes, emptyArrayDS] = ...
    populateTrainingTable(gTruth, labelNames, sampleIndices, imageNames, idxStruct,labeltype)
%populateTrainingTable populate training data table from groundTruth
%datasources.
%
%   Inputs
%   ------
%   gTruth                 - Array of groundTruth objects.
%   labelNames             - Cell array of label names included in training data.
%   sampleIndices          - Cell array of sample indices into label data table.
%   imageNames             - Cell array of image names pre-populated for video
%                            or custom data sources.
%   isVideoOrCustomSource  - Logical indices to groundTruth array specifying which
%                            groundTruths belong to video or custom data sources.
%   labeltype              - Type of label - rectangular, rotated rectangle,
%                            or projected cuboid.
%
%   Outputs
%   -------
%   trainingData              - training data table that can be fed to 
%                               training functions.
%   attributeAndSublabelTable - data table containing attributes and
%                               sublabels corresponding to labels in trainingData.
%   hasAttributes             - boolean indicating whether attributes are
%                               present.
%   emptyArrayDS              - boolean indicating if the arrayds is empty.


trainingData = table();
attributeAndSublabelTable = table();

tablesWithNoAttributeSublabel = 0;
for n = 1 : numel(gTruth)
    if idxStruct.useCustomReader || idxStruct.isDefaultDS(n)
        imageNameTable = table(gTruth(n).DataSource.Source.Files(sampleIndices{n}), 'VariableNames', {'imageFilename'});
    else
        if idxStruct.isVideoOrCustomOrDSSource(n)
            % Image names have already been populated for video sources or custom data sources.
            imageNameTable = table(imageNames{n}, 'VariableNames', {'imageFilename'});
        else
            % Populate image names
            imageNameTable = table(gTruth(n).DataSource.Source(sampleIndices{n}), 'VariableNames', {'imageFilename'});
        end
    end
    
    % Populate bounding boxes
    if isa(gTruth(n).LabelData, 'timetable')
        labelInfo = timetable2table(gTruth(n).LabelData(sampleIndices{n}, labelNames), 'ConvertRowTimes', false);
    else
        labelInfo = gTruth(n).LabelData(sampleIndices{n}, labelNames);
    end
    
    % groundTruthLabeler/ImageLabeler LabelData can have attributes and sublabels.
    [bboxes, allData, hasAttributes] = extractLabelData(labelInfo, labelNames, labeltype);
    
    % Concatenate with training data table.
    trainingData = [trainingData; [imageNameTable bboxes]]; %#ok<AGROW>
    
    % If attributes or sublabels exist, extract their values and store them
    % in a separate table.
    if ~isempty(bboxes)
        % Find the field names for attributes.
        fieldNames = structFieldNames(gTruth(n));
        if isempty(fieldNames)
            tablesWithNoAttributeSublabel = tablesWithNoAttributeSublabel + 1;
        end
        % gather all the attribute and sublabel data.
        attributeAndSublabelData = getLabelDataFromStructTable(allData,fieldNames);
        attributeAndSublabelTable = [attributeAndSublabelTable;table(attributeAndSublabelData(:,1))]; %#ok<AGROW>
    end
end
% Check if all input gtruths are missing attribute and sublabel data.
if(tablesWithNoAttributeSublabel==numel(gTruth))
    emptyArrayDS = 1;
else
    emptyArrayDS = 0;
end

end
%--------------------------------------------------------------------------
function fieldNames = structFieldNames(gTruth)
% Find all the field names (attributes and sublabels) from label definitions.

if any(ismember(gTruth.LabelDefinitions.Properties.VariableNames,'Hierarchy'))
    numDefinitions = numel(gTruth.LabelDefinitions.Hierarchy);
    fieldNames = {};
    for i=1:numDefinitions
        if iscell(gTruth.LabelDefinitions.Hierarchy(i))
            % Skip if not a struct.
            if ~isstruct(gTruth.LabelDefinitions.Hierarchy{i})
                continue
            end
            names = fieldnames(gTruth.LabelDefinitions.Hierarchy{i});
        else
            % Skip if not a struct.
            if ~isstruct(gTruth.LabelDefinitions.Hierarchy(i))
                continue
            end
            names = fieldnames(gTruth.LabelDefinitions.Hierarchy(i));
        end
        fieldNames(end+1:end+numel(names)) = names;
    end
    fieldNames = unique(fieldNames);
else
    fieldNames = {};
end
end
%--------------------------------------------------------------------------
function inputs = parseInputs(varargin)

% Defaults
samplingFactor  = 'auto';
writeLocation   = pwd;
imageFormat     = 'png';
namePrefix      = '';
verbose         = true;
labeltype       = labelType.Rectangle;

parser = inputParser;

% gTruth
addRequired(parser, 'gTruth', ...
    @(in)validateattributes(in, {'groundTruth'}, {'nonempty', 'vector'}));

% SamplingFactor
addParameter(parser, 'SamplingFactor', samplingFactor);

% WriteLocation
addParameter(parser, 'WriteLocation', writeLocation, ...
    @(in)validateattributes(in,{'char','string'},{'scalartext'}));

% ImageFormat
addParameter(parser, 'ImageFormat', imageFormat, ...
    @vision.internal.trainingData.validateImageFormat);

% NamePrefix
addParameter(parser, 'NamePrefix', namePrefix, ...
    @(in)validateattributes(in,{'string','char'},{'scalartext'}));

% Verbose
addParameter(parser, 'Verbose', verbose, ...
    @(in)vision.internal.inputValidation.validateLogical(in,'Verbose'));

% UseParallel
addParameter(parser, 'UseParallel', vision.internal.useParallelPreference);

% labeltype
addParameter(parser, 'LabelType', labeltype, ...
    @validateLabelType);

parse(parser, varargin{:});

inputs = parser.Results;

if isString(inputs.SamplingFactor)
    validatestring(inputs.SamplingFactor, {'auto'}, mfilename, 'SamplingFactor');
    inputs.SamplingFactor = autoFillSamplingFactor(inputs.gTruth);
else
    validateattributes(inputs.SamplingFactor, {'numeric'},...
        {'vector','integer', 'positive', 'numel', numel(inputs.gTruth)}, ...
        mfilename, 'SamplingFactor');
    
    if isscalar(inputs.SamplingFactor)
        % expand scalar for each gTruth
        repelem(inputs.SamplingFactor, numel(inputs.gTruth),1);
    end
end

inputs.UseParallel = logical(parser.Results.UseParallel);
if inputs.UseParallel
    % Check for PCT installation
    try
        % GCP will error if PCT is not available.
        gcp('nocreate');
    catch
        inputs.UseParallel = false;
    end
end
end


%--------------------------------------------------------------------------
function s = autoFillSamplingFactor(gTruth)
% Sources with timestamps get a sampling factor of 5. Others get 1.
s = ones(1,numel(gTruth));
i = arrayfun(@(x)istimetable(x.LabelData), gTruth);
s(i) = 5;
end


%--------------------------------------------------------------------------
function tf = isString(s)
tf = ischar(s) || isstring(s);
end

%--------------------------------------------------------------------------
function [labelPosition, allData, hasAttributes] = extractLabelData(labelInfo, labelNames, type)
    % extractLabelData extracts position, attribute and sublabel information from
    % the labelInfo. The position information of labels is stored in labelPosition
    % and all the extra information like attributes and sublabels is stored in 
    % allData.
    % labelInfo is either a table containing position of labels when there are
    % no attributes/sublabels, or table containing cell arrays, with
    % attribute and sublabel information.
    % labelNames is a cell array of parent label names that need to be
    % populated.
    % labelPosition is a table containing a Mx4 or Mx8 numeric matrix 
    % containing rectangle, rotated rectangle, or projected-cuboid labels
    % and M is the number of parent labels. 
    % allData is a table containing cell arrays containing rectangle, rotated
    % rectangle, or projected-cuboid labels, attributes and sublabel data.

    labelPosition = labelInfo;
    hasAttributes = false;
    allData = labelInfo;
    for idx = 1:height(labelInfo)
        for lIdx = 1:length(labelNames)
            thisItem = labelInfo{idx, labelNames{lIdx}};
            % thisItem is always a cell. If it is a cell array of cell
            % arrays, it must then include a hierarchy.
            thisItem = thisItem{1};
            if isstruct(thisItem)
                hasAttributes = true;
                numROIs = length(thisItem);
                if (type == labelType.Rectangle)
                    position = repmat([0 0 0 0], numROIs, 1);
                elseif (type == labelType.RotatedRectangle)
                    position = repmat([0 0 0 0 0], numROIs, 1);
                else
                    position = repmat([0 0 0 0 0 0 0 0], numROIs, 1);    
                end
                for rIdx = 1:numROIs
                    position(rIdx, :) = thisItem(rIdx).Position;
                end
                labelPosition{idx, labelNames{lIdx}} = {position};
            end
            % If thisItem isn't a cell, it must be [x y w h] already, in
            % which case, this is a no-op.
        end
    end
end
%--------------------------------------------------------------------------
function TF = validateLabelType(fmt)
if ~any(fmt == [labelType.Rectangle, labelType.RotatedRectangle, labelType.ProjectedCuboid])
    error( message('vision:objectDetectorTrainingData:labelTypeNotSupported') )
end
TF = true;
end
%--------------------------------------------------------------------------
function data = getLabelDataFromStructTable(varargin)
% Tables with label data in multiple columns

fieldNames = varargin{2};

% By default all data is converted to a cell array of M-by-2 size,
% bounding boxes in the 1st column and labels in the 2nd column.

data = cellfun(@(x)mergeRowLabels(x, fieldNames),...
    {varargin{1}}, 'UniformOutput', false);
data = vertcat(data{:});
end
% -------------------------------------------------------------------------
function data = mergeRowLabels(data, fieldNames)
% Helper function to merge label data of each row of table.  
bboxes = cell(height(data), 1);
for ii = 1:height(data)
    % for every row, aggregate the attribute and sublabel values.
    bboxRow = data{ii, :};
    x = aggregateBboxesAndLabels(bboxRow, fieldNames);
    bboxes{ii} = x;    
end

data = cell(numel(bboxes), 1);
data(:,1) = bboxes;
end
% -------------------------------------------------------------------------
function x = aggregateBboxesAndLabels(labeledBoxes, fieldNames)
% Aggregate annotations in a single row of a table. 

% find the empty annotations - that class is not present in image
emptyIdx = cellfun(@isempty, labeledBoxes, 'un', 0);
emptyIdx = cell2mat(emptyIdx);

% merge non-empty bboxes
x = labeledBoxes(~emptyIdx);
if ~isempty(fieldNames)
    for i = 1:numel(x)
        % Loop over each struct and add missing fields (present in other
        % structs and not current one) to make vertcat possible.
        for j = 1:numel(x{i})
            defaultFields = {'Description', 'Type', 'Position'};
            missingFields = isfield(x{i}(j), fieldNames(1:end));
            idx = find(~missingFields);
            for k = 1:numel(idx)
                if any(ismember(fieldNames{idx(k)}, defaultFields))
                    continue;
                end
                x{i}(j).(fieldNames{idx(k)})=[];
            end
            % Remove default fields as the 3rd output(arrds) only stores
            % attributes and sublabels.
            for k = 1:numel(defaultFields)
                if isfield(x{i}(j), defaultFields{k})
                    y{i}(j) = rmfield(x{i}(j), defaultFields{k}); 
                end
            end
        end
    end
else
    y = {{}};
end
x = horzcat(y{:})';

end


