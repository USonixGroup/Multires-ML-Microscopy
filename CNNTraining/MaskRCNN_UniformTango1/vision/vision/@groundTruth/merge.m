function mergedGTruth = merge(gTruth,args)
% 

% Copyright 2023 The MathWorks, Inc.

arguments(Repeating)
    gTruth groundTruth
end

arguments
    args.OutputFolder {mustBeTextScalar} = pwd
end

gTruth = iParseInputs(gTruth);

% Extract source, definitions, and label data from groundTruth objects.
allSources     = iExtractSources(gTruth);
allDefinitions = iExtractLabelDefinitions(gTruth);
allData        = iExtractLabelData(gTruth);

% Merge sources, definitions, and label data. Wrap main merge code in
% try-catch block to throw a concise error.
try
    % Merge label definitions.
    mergedDef  = groundTruth.mergeLabelDefinitions(allDefinitions);

    hasPixelLabels = any(mergedDef.Type==labelType.PixelLabel);
    if hasPixelLabels
        % Create output folder for merged pixel labels. When more than one
        % unique video or image sequence source is input, there will be
        % sub-folders within the created the output folder. These
        % sub-folders are created during the merge procedure.
        args.OutputFolder = iCreateOutputFolder(args.OutputFolder);

        % Set subfolder to empty. This will be reset to the required name
        % as we merge the different source types. 
        args.SubfolderName = "";
    end
    
    % Ground truth objects with temporal sources can only be merged
    % together if the source is the same. Partition groundTruth based those
    % that have timestamps and those that do not. 
    isTemporalSource = cellfun(@(x)x.hasTimeStamps,allSources);   
    
    % Merge all groundTruth containing image sources.
    A = allSources(~isTemporalSource);
    mergedGTruthImage = [];
    if ~isempty(A)
        B = allData(~isTemporalSource);
        sourceInfo = iExtractSourceInfo(A);

        % When there are mixed sources (image and video), image source
        % pixel label data is merged into a subfolder named gTruth_1.
        % Otherwise, a subfolder is not used.
        if any(isTemporalSource)
            args.SubfolderName = "gTruth_1"; % also indicate we have image gTruth
        else
            args.SubfolderName = "";
        end
        
        % Merge sources and label data.
        [mergeSources, mergedData] = iMergeSourcesAndLabelData(...
            A, B, mergedDef, allDefinitions(~isTemporalSource), sourceInfo, args);

        % Create new groundTruth with merged data.
        mergedGTruthImage = groundTruth(mergeSources, mergedDef, mergedData);
    end

    % Merge all groundTruth with video sources. 
    mergedGTruthTemporal = {};
    if any(isTemporalSource)
        A = allSources(isTemporalSource);
        B = allData(isTemporalSource);

        % Group sources by common video or image sequence files. groups is
        % a set of indices that start at 1. The group index is also used to
        % add the numeric suffice to the subfolder names down below.
        groups = iFindGroups(A);

        subfolderName = iCreateSubfolderNamesForVideoSources(groups,isTemporalSource);

        % Merge each group of sources. Each group is merged into one
        % groundTruth object. 
        unmergedDefs = allDefinitions(isTemporalSource);
        mergedGTruthTemporal = splitapply(...
            @(a,b,c,d)iMergeVideoSource(a,b,c,d,mergedDef,args),...
            A,B,subfolderName,unmergedDefs,groups);
    else
        args.SubfolderNames = [];
    end

catch ME
    throwAsCaller(ME)
end

% Return all the image and video groundTruth objects.
mergedGTruth = vertcat(mergedGTruthImage, mergedGTruthTemporal{:});

end

%--------------------------------------------------------------------------
function gTruth = iParseInputs(gTruth)

gTruth = iVectorizeGroundTruthInputs(gTruth);

if numel(gTruth) < 2
    error(message('vision:groundTruth:mergeRequiresAtLeastTwoInputGTruth'));
end

iAssertGTruthHaveValidDataSources(gTruth);

iAssertGTruthExcludesUnsupportedLabelTypes(gTruth);

iAssertGTruthUsingSameSourceReadFcn(gTruth);

end

%--------------------------------------------------------------------------
function iAssertGTruthUsingSameSourceReadFcn(gTruth)
% groundTruth are mergeable if and only if they all have use the same
% underlying read function. This check if for imageDatastore sources.
%
% No error check is required for image collection data source because a
% custom read function is not supported.

% Assert all imageDatastores use the same read function. 
isDatastore = arrayfun(@(x)isImageDatastore(x.DataSource), gTruth);

if any(isDatastore)
    readFcns = arrayfun(@(x)x.DataSource.Source.ReadFcn, gTruth(isDatastore), UniformOutput=false);
    usingSameReadFcn = cellfun(@(x)isequal(x,readFcns{1}),readFcns);
    if ~all(usingSameReadFcn)
        error(message("vision:groundTruth:mergeDifferentReadFcn"));
    end

    % Mixture of image collections and imageDatastore sources. Verify all
    % ImageDatastore sources have the default read function.
    if ~all(isDatastore)
        usingDefaultReadFcn = cellfun(@(x)vision.internal.isDefaultImdsReadFcn(x),readFcns);

        if ~(usingDefaultReadFcn)
            error(message("vision:groundTruth:mergeImageCollectionAndDatastoreUsingNonDefaultReadFcn"));
        end
    end
end
end

%--------------------------------------------------------------------------
function iAssertGTruthExcludesUnsupportedLabelTypes(gTruth)
supportedTypes = [
    labelType.Rectangle; 
    labelType.RotatedRectangle;
    labelType.Line;
    labelType.PixelLabel;
    labelType.Point
    labelType.Scene;
    labelType.Polygon;
    labelType.ProjectedCuboid;
    labelType.Cuboid];

for i = 1:numel(gTruth)
    isSupported = ismember(gTruth(i).LabelDefinitions.Type, supportedTypes);
    if ~all(isSupported)
        error(message('vision:groundTruth:mergeUnsupportedLabelType'))
    end
end
end

%--------------------------------------------------------------------------
function gTruth = iVectorizeGroundTruthInputs(varargin)
gTruth = cellfun(@(x)reshape(x,[],1),[varargin{1}],UniformOutput=false);
gTruth = vertcat(gTruth{:});
end

%--------------------------------------------------------------------------
function iAssertGTruthHaveValidDataSources(gTruth)
% Assert that all groundTruth have valid data sources.
for i = 1:numel(gTruth)

    if ~hasValidDataSource(gTruth(i))
        error(message("vision:groundTruth:mergeInvalidDataSource"));
    end

    src = gTruth(i).DataSource;
    if ~(isImageCollection(src) || isImageDatastore(src) || isVideoFileSource(src) || isImageSequenceSource(src))
        error(message('vision:groundTruth:mergeUnsupportedSource'))
    end
end
end

%--------------------------------------------------------------------------
function srcs = iExtractSources(gTruth)
srcs = {gTruth(:).DataSource};
end

%--------------------------------------------------------------------------
function info = iExtractSourceInfo(allSources)
type = cellfun(@(x)string(class(x.Source)), allSources);
if all(strcmp(type,"matlab.io.datastore.ImageDatastore"))
    info.Type = "imageDatastore";

    % Store the read function to use for the combined datastore. Merging is
    % only supported if all the datastores have the same read function.
    info.ReadFcn = allSources{1}.Source.ReadFcn;
else
    % Mixture of source types. Merged source is groundTruthDataSource.
    info.Type = "groundTruthDataSource";
end
end

%--------------------------------------------------------------------------
function defs = iExtractLabelDefinitions(gTruth)
defs = {gTruth(:).LabelDefinitions};
end

%--------------------------------------------------------------------------
function data = iExtractLabelData(gTruth)
data = {gTruth(:).LabelData};
end

%--------------------------------------------------------------------------
function allFiles = iExtractFiles(allSources)
%
files = cell(numel(allSources),1);
for i = 1:numel(allSources)
    aSource = allSources{i};
    if aSource.isImageDatastore()
        files{i} = aSource.Source.Files;
    elseif aSource.isVideoFileSource()
        % Replicate video source. The names shall be used when creating new
        % pixel label data files.
        files{i} = repmat({aSource.Source},numel(aSource.TimeStamps),1);
    else
        % Image collection or sequence
        files{i} = aSource.Source;
    end
end
allFiles = vertcat(files{:});
end

%--------------------------------------------------------------------------
function gTruth = iMergeVideoSource(src,data,subfolder,originalDefs,mergedDef,args)
sourceInfo.Type = "groundTruthDataSource";
args.SubfolderName = subfolder(1);
[mergedSources, mergedData] = iMergeSourcesAndLabelData(...
    src, data, mergedDef, originalDefs, sourceInfo, args);
gTruth = {groundTruth(mergedSources, mergedDef, mergedData)};
end

%--------------------------------------------------------------------------
function subfolderName = iCreateSubfolderNamesForVideoSources(groups,isTemporalSource)
hasImageSources = any(~isTemporalSource);
if hasImageSources
    % When there mixed sources, the first output corresponds to the
    % merged output of the image groundTruth. Therefore, if a
    % subfolder is used for video pixel label data the folder names
    % will start at gTruth_2.
    subfolderName = "gTruth_" + (groups+1);
else
    % No images sources.
    if isscalar(unique(groups))
        % There is only one unique video source. Do not write pixel
        % label data into a subfolder.
        subfolderName = repmat("",1,numel(groups));
    else
        % More than one unique video source. Write pixel label data
        % to separate subfolders starting with gTruth_1 (because
        % there are no image sources).
        subfolderName = "gTruth_" + (groups);
    end
end
end

%--------------------------------------------------------------------------
function [src, data] = iMergeSourcesAndLabelData(allSources, allData, mergedDef,...
    originalDef, sourceInfo, args)

allFiles = iExtractFiles(allSources);

allData = iAddMissingLabelsIfNeeded(allData, mergedDef);

% Label data for temporal sources is stored in a timetable. The merged
% label data table will also be a time table with the same timestamps as
% the input label data tables.
hasTime = cellfun(@(x)istimetable(x),allData);
assert(all(hasTime)||~all(hasTime),"Should never be a mixture of timetables and tables!")
if all(hasTime)
    isVideoOrImgSeq = true;
    dataTable = vertcat(allData{:});
    dataTable = timetable2table(dataTable);
else
    isVideoOrImgSeq = false;
    dataTable = [allFiles vertcat(allData{:})];
end
offsetToData = 1;

% Determine the type of label in each label data column.
types = iFindLabelTypeForEachDataColumn(dataTable, mergedDef);

hasHierarchy = iHasHierarchy(mergedDef);

% Find groups of files and use splitapply to merge label data. Groups are
% created by file name or timestamps. When used with splitapply, this
% gathers all the label data associated with a particular file (timestamps)
% across multiple ground truth objects.
G = findgroups(dataTable(:,1));

% Run splitapply to generate the list of source files we expect after
% merging.
mergedFilesOrTimestamps = splitapply(@(in)in(1),dataTable(:,1),G);

if isVideoOrImgSeq
    % Create a new copy of the first source. 
    src = groundTruthDataSource.loadobj(saveobj(allSources{1}));
else
    % Create groundTruthDataSource or imageDatastore.
    if strcmp(sourceInfo.Type,"groundTruthDataSource")
        src = groundTruthDataSource(mergedFilesOrTimestamps);
    else
        ds = imageDatastore(mergedFilesOrTimestamps,ReadFcn=sourceInfo.ReadFcn);
        src = groundTruthDataSource(ds);
    end
end

% Prepare to merge pixel label data. 
hasPixelLabels = any(mergedDef.Type==labelType.PixelLabel);
if hasPixelLabels

    outputFolder = args.OutputFolder;

    % If the current subset of ground truth has pixel label data then
    % create a subfolder. 
    subsetHasPixelLabels = any(cellfun(@(x)any(x.Type==labelType.PixelLabel),originalDef));
    if subsetHasPixelLabels && strlength(args.SubfolderName) > 0
        outputFolder = iCreateSubfolder(outputFolder,args.SubfolderName);
    end

    % Generate index to originalDef for each file.
    numFiles = cellfun(@(x)height(x),allData);
    indexToDef = arrayfun(@(idx,n) repmat(idx,n,1),...
        1:numel(allData), numFiles, ...
        UniformOutput=false)';

    % Repackage as cell array to combine all data into one table.
    indexToDef = num2cell(vertcat(indexToDef{:}));

    % Add index to definition data and the grouping indices to the data
    % table. The grouping indices define the order of the src files after
    % they are merged and are used as the index values for creating pixel
    % label data file names: "Label_<index>_src.png".
    
    dataTable = [allFiles indexToDef num2cell(G) dataTable(:,2:end)];
    
    offsetToData = 3;
end

% Use splitapply to group the data to manually process the grouped items. 
groupedData = splitapply(@iGroupData,dataTable,G);

% Merge label data within each group. Using splitapply here is not feasible
% as it does not properly capture exception causes. By processing the
% groups manually we are able to throw better error messages. 
data = cell(size(groupedData,1),1);
for ii = 1:size(groupedData,1)
    data{ii} = nMergeLabelData(groupedData{ii,:});
end

% Stack the data to produce a table.
data = vertcat(data{:});

% Update variable names to match label definitions.
data.Properties.VariableNames = dataTable.Properties.VariableNames((offsetToData+1):end);

% Add times if the data was a timetable.
if isVideoOrImgSeq
    data = table2timetable(data,RowTimes=mergedFilesOrTimestamps);
end

    % Nested label data merging function for processing groups of data.
    function out = nMergeLabelData(varargin)
        n = numel(varargin)-offsetToData;
        row = cell(1,n);
        files = varargin{1};

        for i = 1:n
            switch types(i)
                case labelType.Scene
                    row{i} = iMergeSceneLabels(varargin{i+offsetToData});
                    
                case labelType.PixelLabel
                    pxData = varargin{i+offsetToData};
                    fileIdx = varargin{3};  
                    row{i} = iMergeAndWritePixelLabelData(files, pxData, fileIdx, ...
                        originalDef(varargin{2}), mergedDef, outputFolder);
                   
                otherwise % ROI labels
                    labels = varargin{i+offsetToData};
                    row{i} = iMergeROIs(hasHierarchy, labels{:});
                   
            end
        end
        out = table(row{:});
    end

end

%--------------------------------------------------------------------------
function allData = iAddMissingLabelsIfNeeded(allData, def)

% Replace all pixel label class names with PixelLabelData. This ensures
% we add a PixelLabelData variable to label data tables that need them.
names = def.Name;
types = def.Type;
hasHierarchy = iHasHierarchy(def);

if hasHierarchy
    hierarchy = def.Hierarchy;
else
    hierarchy = repmat({[]},height(def),1);
end

if any(def.Type == labelType.PixelLabel)
    keep = def.Type ~= labelType.PixelLabel;
    names = names(keep);
    types = types(keep);
    hierarchy = hierarchy(keep); 
    names = [names; 'PixelLabelData'];
    types = [types; labelType.PixelLabel];
    hierarchy = [hierarchy; {[]}];
end

for i = 1:numel(allData)

    vars = allData{i}.Properties.VariableNames;

    % What is this data table missing?    
    [v,idx] = setdiff(names,vars);

    nrows = height(allData{i});
    
    dataToAppend = cell(1,numel(idx));
    for j = 1:numel(idx)
       dataToAppend{j} = iEmptyDataByType(types(idx(j)), nrows, hasHierarchy, hierarchy{idx(j)});  
    end

    dataToAppend = table(dataToAppend{:},VariableNames=v);

    allData{i} = [allData{i} dataToAppend];

    % Set table variables the same order as in label definition table.
    allData{i} = allData{i}(:,names);
end

end

%--------------------------------------------------------------------------
function data = iEmptyDataByType(type, nrows, hasHierarchy, hierarchy)
switch type
    case labelType.Scene
        data = false(nrows,1);

    case labelType.PixelLabel
        data = repmat({''},nrows,1);

    case labelType.Custom
        assert(0, 'custom label type is not supported.')

    otherwise % ROI type  
        if hasHierarchy
            data = iCreateEmptyDataInStruct(nrows, hierarchy);
        else
            data = repmat({[]}, nrows, 1);
        end
end
end

%--------------------------------------------------------------------------
function tf = iHasHierarchy(def)
tf = any(strcmp('Hierarchy',def.Properties.VariableNames));
end

%--------------------------------------------------------------------------
function s = iCreateEmptyDataInStruct(nrows, hierarchy)
if isempty(hierarchy)
    % The ROI does not have attributes but other ROIs do have
    % attributes. This requires that even ROIs without attributes store
    % their data in a struct with a Position field.
    s = struct('Position',cell(0,1));
else
    
    hierarchy = rmfield(hierarchy,{'Type','Description'});
    names = fieldnames(hierarchy);
    names = [{'Position'}; names];

    args = cell(1,2*numel(names));
    args(1:2:end) = names;
    args(2:2:end) = repmat({cell(0,1)},1,numel(names));
    s = struct(args{:});
end
s = repmat({s},nrows,1);
end

%--------------------------------------------------------------------------
function out = iGroupData(varargin)
out = varargin;
end

%--------------------------------------------------------------------------
function mergedLabel = iMergeSceneLabels(labels)
% Input labels are logical data. Use accumarray to count occurrences by
% treating logical values as indices. This places false labels into
% counts(1) and true labels into counts(2).
counts = accumarray(labels+1,1,[2 1]);

if counts(1) == counts(2)
    % In the event of a tie, the merged label is the label from the first
    % entry in labels.
    mergedLabel = labels(1);
else
    % Use MAX to determine the majority label. The merged label is true if
    % the max is the second count, otherwise it is false.
    [~,idx] = max(counts);   
    mergedLabel = idx==2;
end
end

%--------------------------------------------------------------------------
function pxDataMerged = iMergeAndWritePixelLabelData(fileOrTimestamp, pxData, fileIdx, oldDef, mergedDef, outputFolder)

% The merged definition always have the following pixel label IDs: 1 to
% the "number of pixel label classes".
mergedClasses = mergedDef.Name(mergedDef.Type==labelType.PixelLabel);

% Remove any empty PixelLabelData entries.
emptyIdx = cellfun(@(x)isempty(x),pxData);
pxData(emptyIdx) = [];
oldDef(emptyIdx) = [];

if isempty(pxData)
    % All entries are empty, return {''}.
    pxDataMerged = {''};

else
    % Merge pixel label data files pixel-wise majority consensus.
    try
        imds = imageDatastore(pxData);
    catch readException
        iThrowUnableToReadPixelLabelData(readException);
    end
    
    oldDefDS  = arrayDatastore(reshape(oldDef,[],1), IterationDimension=1, OutputType="same");
    if ~iscell(fileOrTimestamp)
        % For video based sources, convert duration array to cell array.
        fileOrTimestamp = num2cell(fileOrTimestamp);
    end
    srcDS = arrayDatastore(fileOrTimestamp,IterationDimension=1, OutputType="same");
    cds = combine(imds, oldDefDS, srcDS);

    catds = transform(cds, ...
        @(data,info)iStandardizePixelLabelDataTransform(data,info,mergedClasses),...
        IncludeInfo=true);

    L = iPixelLabelConsensus(catds);

    % Write merged pixel label data.
    try
        pxDataMerged{1} = iCreatePixelLabelDataFilename(outputFolder, fileIdx(1), fileOrTimestamp{1});
        imwrite(uint8(L),pxDataMerged{1});
    catch writeException
        iThrowUnableToWritePixelLabelData(writeException);
    end
end
end

%--------------------------------------------------------------------------
function C = iPixelLabelDataToCategorical(L,def)
pxidx = def.Type == labelType.PixelLabel;
id = vertcat(def.PixelLabelID{pxidx});
classes = def.Name(pxidx);
C = categorical(L,id,classes);
end

%--------------------------------------------------------------------------
function [C,info] = iStandardizePixelLabelDataTransform(data,info,mergedClasses)
M = size(data,1);
C = cell(M,1);
for i = 1:M

    % Convert to categorical. Here we leverage categorical to remap classes
    % to the new ID types by adding missing categories and then setting the
    % category order defined within mergedDef. 
    L = data{i,1};
    def = data{i,2};
    C{i} = iPixelLabelDataToCategorical(L,def);
   
    % Add missing categories and then reorder categories to match the order
    % defined by the merged definition. 
    missingClasses = setdiff(mergedClasses, categories(C{i}));
    C{i} = addcats(C{i},missingClasses);
    C{i} = reordercats(C{i}, mergedClasses);

    % Add source file name to info struct in case we need to throw an
    % error and point users to the source file.
    info{i}.SourceFilename = data{i,3};
end

end

%--------------------------------------------------------------------------
function L = iPixelLabelConsensus(catds)
% Perform pixel-wise label consensus. Unlabeled pixels are ignored and do
% not influence the per-pixel voting.
%
% The input datastore returns categorical matrices with the same
% categories and each matrix returned by the datastore must have the same
% size. 

C = preview(catds);
numcats = numel(categories(C{1}));
expectedSize = size(C{1},[1 2]);
votes = zeros([expectedSize numcats]);
while hasdata(catds)

    try
        [C, info] = read(catds);
    catch ME
        iThrowUnableToReadPixelLabelData(ME)
    end

    for i = 1:numel(C)

        if ~isequal(size(C{i}), expectedSize)
            actualSize = size(C{i},[1 2]);
            badPix = info{i}.Filename;
            badSrc = info{i}.SourceFilename;
            error(message('vision:groundTruth:mergePixelLabelUnequalSizes',...
                mat2str(actualSize), mat2str(expectedSize), ...
                badPix, badSrc));
        end

        % Aggregate votes.
        votes = votes + countcats(C{i},3);
    end
end

% Determine the majority class to generate a label matrix, L. Then set
% any pixels with a zero vote count to have class label 0 to indicate which
% pixels have no assigned label.
[maxCount,L] = max(votes,[],3);
L(maxCount==0) = 0;

end

%--------------------------------------------------------------------------
function iThrowUnableToReadPixelLabelData(cause)
ex = MException(...
    message('vision:groundTruth:mergeUnableToReadPixelLabelData'));
iAddCauseAndThrow(ex, cause);
end

%--------------------------------------------------------------------------
function iThrowUnableToWritePixelLabelData(cause)
ex = MException(...
    message('vision:groundTruth:mergeUnableToWritePixelLabelData'));
iAddCauseAndThrow(ex, cause);
end

%--------------------------------------------------------------------------
function name = iCreatePixelLabelDataFilename(outputFolder, idx, srcFilename)
[~,filename,~] = fileparts(srcFilename);
name = vision.internal.labeler.multiUser.model.project.ProjTaskUtil. ...
    createPixelLabelFilePath(outputFolder, idx, filename);

% groundTruth expects all filenames to be char.
name = char(name);
end

%--------------------------------------------------------------------------
function out = iMergeROIs(hasHierarchy, label)
% Use concatenation to merge ROIs.
arguments
    hasHierarchy
end
arguments(Repeating)
    label
end

% The groundTruth object may store ROI data as numeric, cell, or struct
% array. When the data is in a struct array, multiple ROI are represented
% as in a row vector. Otherwise, the data is stacked vertically. In
% addition, when merging across label definitions some labels may have data
% stored in structs or in the numeric/cell format. In this case, all data
% must be returned as in a struct to comply with the groundTruth syntax.


if hasHierarchy
    % Pack all non-struct data into a struct. Non-struct data should be
    % packed into the struct Position field.

    for i = 1:numel(label)
        if ~isstruct(label{i})
            data = label{i};
            if isempty(data)
                label{i} = [];
            else
                numROI = size(data,1);
                s = struct('Position',cell(1,numROI));
                for k = 1:numROI
                    s(k).Position = data(k,:);
                end
                label{i} = s;
            end
            
        end
    end
    out = horzcat(label{:});
else
    % Vertcat for numeric data. 
    out = vertcat(label{:});
end

out = iRemoveDuplicateROILabels(out);

% Pack into cell for splitapply to return table output.
out = {out};
end

%--------------------------------------------------------------------------
function uniqueLabels = iRemoveDuplicateROILabels(labels)
if isstruct(labels)
    % ROI labels with attributes and/or sublabels.
    
    fields = fieldnames(labels);
    
    if numel(fields) == 1 && strcmp(fields, 'Position') %#ok<ISCL>
        % Only position data. Extract data and call this function again to
        % remove duplicates.
        data = vertcat(labels(:).Position);
        data = iRemoveDuplicateROILabels(data);

        % Wrap cell data in a cell so we can use STRUCT to create a struct
        % array and place the data in the Position field as a cell.
        if iscell(data)
            data = arrayfun(@(x){x},data);
        else
            data = num2cell(data,2);
        end

        % Package data back into a 1-by-N struct array.
        uniqueLabels = reshape(struct('Position', data),1,[]);
    else
        % Attributes and/or sublabels. Scan through list and iteratively
        % remove duplicate labels. 
        k = 1;
        while k < numel(labels)

            isRepeat = false(1,numel(labels));
            for i = (k+1):numel(labels)
                isRepeat(i) = isequal(labels(k),labels(i));
            end

            % Remove repeats from labels.
            labels(isRepeat) = [];

            % Onto the next element.
            k = k + 1;

        end
        uniqueLabels = labels;
    
    end
 
else
    % ROI labels with only position data. 
    if iscell(labels)

        % Data in cells may have different lengths. Remove duplicates
        % by first grouping data by size and then find duplicates within
        % each size group. This makes it possible to apply UNIQUE to remove
        % duplicates from the data.
        
        % Group by size.
        sz = cellfun(@(x)size(x,1),labels);
        sizeGroups = findgroups(sz);
        
        % Apply UNIQUE to each size group. The grouping process reorders
        % the label data. Pass in an additional linear index to keep track
        % of the original data order and reorder the data. 
        linearIndex = (1:numel(labels))';
        [uniqueLabels, uniqueIndices] = splitapply(...
            @iUniqueLabelsForFixedSizeGroup, labels, linearIndex, sizeGroups);
        uniqueLabels = vertcat(uniqueLabels{:});

        % Return results back in the same order as input, minus duplicates.
        uniqueIndices = [vertcat(uniqueIndices{:})];
        [~, linearOrder] = sort(uniqueIndices);
        uniqueLabels = uniqueLabels(linearOrder);

    elseif isnumeric(labels)
        uniqueLabels = unique(labels,"rows","stable");
    else
        assert(0,"Internal error: ROI label data was not packaged in struct, cell array, or numeric array.");
    end
end
end

%--------------------------------------------------------------------------
function [uniqueLabels, linearIndex] = iUniqueLabelsForFixedSizeGroup(labels, linearIndex)
% Input label data is a cell array is stored in M-by-2 matrices where M is
% the number vertices. All cell elements have the same size.

% Repackage label data into a matrix numel(labels)-by-(M*2) matrix. 
labelsInMatrix = cellfun(@(x)reshape(x,1,[]),labels,UniformOutput=false);
labelsInMatrix = cell2mat(labelsInMatrix);

% Use unique "rows" to keep only the unique ROI data.
[~, uniqueIdx] = unique(labelsInMatrix, "rows", "stable");
uniqueLabels = {labels(uniqueIdx)};
linearIndex = {linearIndex(uniqueIdx)};
end

%--------------------------------------------------------------------------
function types = iFindLabelTypeForEachDataColumn(dataTable, def)
varNames = dataTable.Properties.VariableNames(2:end);
n = numel(varNames);

% Preallocate types array.
types(n) = labelType.Rectangle;

% Fill types array based on each label data table variable.
for k = 1:n
    name = varNames{k};
    if strcmp(name,"PixelLabelData")
        types(k) = labelType.PixelLabel;
    else
        idx = strcmp(def.Name,varNames{k});
        types(k) = def.Type(idx);
    end
end
end

%--------------------------------------------------------------------------
function location = iCreateOutputFolder(location)
% Create a folder named mergedPixelLabels within the specified location. Add
% unique suffix if required.

iAssertOutputFolderExists(location);

foldername = iCreateUniqueFoldername(location, 'mergedPixelLabelData');

% Create output folder.
location = fullfile(location,foldername);
iTryToCreateFolder(location);
end

%--------------------------------------------------------------------------
function location = iCreateSubfolder(location,subfolder)
% Create subfolder if needed when merging more than one video and image
% collections.
location = fullfile(location,subfolder);
iTryToCreateFolder(location);
end

%--------------------------------------------------------------------------
function iTryToCreateFolder(location)
try
    [success, msg, msgId] = mkdir(location);
    if ~success
        throw(MException(msgId,msg));
    end
catch ME
    iThrowUnableToCreateOutputFolderMessage(ME,location);
end
end

%--------------------------------------------------------------------------
function iAssertOutputFolderExists(location)
if ~exist(location,'dir')
    error(message('vision:groundTruth:mergeOutputFolderMustExist'));
end
end

%--------------------------------------------------------------------------
function uniqueName = iCreateUniqueFoldername(location, outputFolderName)
putativeLocation = fullfile(location, outputFolderName);
uniqueName = outputFolderName;
k = 0;
while exist(putativeLocation) %#ok<EXIST>
    k = k + 1;
    uniqueName = outputFolderName + "_" + k;
    putativeLocation = fullfile(location, uniqueName);
end
uniqueName = char(uniqueName);
end

%--------------------------------------------------------------------------
function iThrowUnableToCreateOutputFolderMessage(cause,outputFolderName)
ex = MException(...
    message('vision:groundTruth:mergeUnableToCreateFolder',outputFolderName));
iAddCauseAndThrow(ex, cause);
end

%--------------------------------------------------------------------------
function iAddCauseAndThrow(exception, cause)
exception = addCause(exception,cause);
throwAsCaller(exception);
end

%--------------------------------------------------------------------------
function groupIDs = iFindGroups(srcs)
% Create a dictionary to hold sources that are equal.
d = dictionary();

% Create a dictionary to hold the indices of the sources. These indices are
% used to sort the group IDs into an order that matches the original input
% sources.
id = dictionary();
indices = 1:numel(srcs);

% Group non-unique sources.
k = 1;
while numel(srcs) > 0
    idx = iIsEqualSource(srcs,srcs{1});
    d(k) = {srcs(idx)}; 
    id(k) = {indices(idx)};
    srcs(idx) = [];
    indices(idx) = [];
    k = k + 1;
end

% Extract the group IDs.
groupIDs = cellfun(@(x,y)repmat(y,numel(x),1),d.values,num2cell(d.keys),UniformOutput=false);
groupIDs = vertcat(groupIDs{:})';

% Resort the group IDs to match the order of the input sources. 
resortingIndices = [id.values{:}];
groupIDs = groupIDs(resortingIndices);
end

%--------------------------------------------------------------------------
function tf = iIsEqualSource(src1,src2)
tf = false(size(src1));
for i = 1:numel(src1)
    tf(i) = iIsEqualSourceElement(src1{i},src2);
end
end

%--------------------------------------------------------------------------
function tf = iIsEqualSourceElement(a,b)

tf = a.SourceType == b.SourceType;

if tf
    if a.isVideoFileSource() || a.isImageSequenceSource()
        % Video sources are equal if and only if the source
        % and timestamps are equal.
        tf = isequal(a.Source,b.Source);
        tf = tf && isequal(a.TimeStamps,b.TimeStamps);

    elseif a.isImageCollection() || a.isImageDatastore()
        tf = isequal(a.Source,b.Source);

    elseif a.isCustomSource()
        assert(0,"Unsupported source type!");
    end
end
end
