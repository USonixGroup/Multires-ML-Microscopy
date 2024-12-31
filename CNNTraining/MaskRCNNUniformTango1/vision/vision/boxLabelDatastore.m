classdef boxLabelDatastore <...
        matlab.io.Datastore &...
        matlab.io.datastore.mixin.Subsettable &...
        matlab.mixin.CustomDisplay

% Copyright 2019-2023 The MathWorks, Inc.

    properties (Dependent, SetAccess = private)
        %LABELDATA All of the labeled bounding box data.
        %   All of the labeled bounding box data provided to this
        %   datastore.
        %   An N-by-2 cell array where,
        %     1st column - contains all the bounding boxes, with each element being
        %                  an M-by-4 double matrix in the format [x y width height].
        %     2nd column - contains the label names corresponding to each bounding box,
        %                  with each element being an M-by-1 categorical vector.
        %
        %   An example LabelData looks like below:
        %
        %     >> lbl = ds.LabelData
        %     lbl =
        %       3x2 cell array
        %         {3x4 double}     {3x1 categorical}
        %         {9x4 double}     {9x1 categorical}
        %         {8x4 double}     {8x1 categorical}
        %
        %     In the above example,
        %         - lbl{1,2} is a categorical vector of size 3 with label values
        %           ["Car","Light", "Pedestrian"].
        %         - lbl{1,1} contains the corresponding bounding boxes
        %           of the labels in lbl{1,2}.
        LabelData
    end

    properties (Dependent)
        %READSIZE Height of the label data returned by the read method.
        %   Upper limit on the height of LabelData returned by the
        %   read method.
        ReadSize
    end

    properties (Access = private)
        % InMemoryDatastore object to hold boxes and labels.
        InMemoryDatastore
    end

    methods
        function obj = boxLabelDatastore(varargin)
            %boxLabelDatastore A datastore for bounding box labels.
            %   boxLabelDatastore creates a boxLabelDatastore that provides data in
            %   a table format with bounding boxes as the first column and labels as
            %   the second column. Use combine method on imageDatastore and
            %   boxLabelDatastore to feed the data to train object detectors, such as
            %   trainFasterRCNNObjectDetector.
            %
            %   blds = boxLabelDatastore(TBL) creates a boxLabelDatastore object
            %   using the table, TBL, that contains labeled bounding box data.
            %
            %   blds = boxLabelDatastore(TBL1, TBL2, ...) creates a boxLabelDatastore
            %   using the tables, TBL1, TBL2, ..., that contains labeled bounding box
            %   data. This allows creating a datastore that contains labeled bounding
            %   boxes of various classes in different tables. Label data in TBL1
            %   corresponds to first set of labels, TBL2 corresponds to second set of
            %   labels, and so on.
            %
            %   See also objectDetectorTrainingData, imageDatastore, boxLabelDatastore,
            %            trainFasterRCNNObjectDetector, trainYOLOv2ObjectDetector,
            %            groundTruthLabeler, videoLabeler, imageLabeler.
            import vision.internal.cnn.datastore.InMemoryDatastore;

            try
                narginchk(0,inf);

                thisFunctionName = mfilename;

                if nargin == 0
                    % Empty datastore with 2 columns.
                    data = cell.empty(0,2);
                    bSet = [];
                else
                    [tables,bSet] = iCheckIfBlockSetIsPresent(varargin);
                    tableType = iCheckAllGroundTruthTables(tables, thisFunctionName);
                    switch tableType
                        case 'type1'
                            data = iGetLabelDataFromTypeOne(tables{:});
                        case 'type2'
                            data = iGetLabelDataFromTypeTwo(tables{:});
                        case 'type3'
                            data = iGetLabelDataFromTypeThree(tables{:});
                    end
                end

                iVerifyImageNumbersAndBoxFormat(data,bSet);

                if ~isempty(bSet)
                    bboxSize = iGetBoxSize(data);
                    if bboxSize == 5
                        % Rotated rectangle bounding boxes require a
                        % threshold of 1 for use in bboxcrop.
                        overlapThreshold = 1;
                    else
                        overlapThreshold = 0.1;
                    end
                    data = vision.internal.bbox.bboxcropUsingBlockSet(data,bSet,...
                        overlapThreshold);
                end

                defaultReadSize = 1;
                obj.InMemoryDatastore = InMemoryDatastore(data, defaultReadSize, ...
                    thisFunctionName);
            catch ME
                throw(ME);
            end
        end

        function [data, info] = read(obj)
            %READ  Read subset of data from a datastore.
            %   T = READ(DS) reads READSIZE height of data from DS.
            %   T is a M-by-2 cell array with first column containing bounding
            %   boxes and the second column containing a categorical vector of
            %   label names. M is less than or equal to READSIZE.
            %   read(DS) errors if there is no more data in DS, and should be used
            %   with hasdata(DS).
            %
            %   [T,info] = READ(DS) also returns a structure with additional
            %   information about TDS. The fields of info are:
            %       CurrentIndex - Current index off of the total height of the data.
            %       ReadSize     - ReadSize property value of the datastore.
            %
            %   Example
            %   -------
            %      % Load training data that contains bounding boxes with labels for vehicles.
            %      data = load('vehicleTrainingData.mat');
            %      trainingData = data.vehicleTrainingData;
            %
            %      % Add fullpath to the local vehicle data folder.
            %      dataDir = fullfile(toolboxdir('vision'), 'visiondata');
            %      trainingData.imageFilename = fullfile(dataDir,...
            %                                              trainingData.imageFilename);
            %      blds = boxLabelDatastore(trainingData(:,2:end));
            %
            %      % Read and see how the box labels data looks for training.
            %      read(blds)
            %
            %      % Reset before the while-loop.
            %      reset(blds);
            %      while hasdata(blds)
            %         % Read one row of box labels at a time
            %         bxLabels = read(blds);
            %      end
            %
            %   See also imageDatastore, boxLabelDatastore, objectDetectorTrainingData,
            %            readall, hasdata, reset.
            [data, info] = read(obj.InMemoryDatastore);
        end

        function tf = hasdata(obj)
            %HASDATA Returns true if there is unread data in the boxLabelDatastore.
            %   TF = HASDATA(DS) returns true if the datastore has more data
            %   available to read with the read method. read(DS) returns an error
            %   when HASDATA(DS) returns false.
            %
            %   Example
            %   -------
            %      % Load training data that contains bounding boxes with labels for vehicles.
            %      data = load('vehicleTrainingData.mat');
            %      trainingData = data.vehicleTrainingData;
            %
            %      % Add fullpath to the local vehicle data folder.
            %      dataDir = fullfile(toolboxdir('vision'), 'visiondata');
            %      trainingData.imageFilename = fullfile(dataDir,...
            %                                              trainingData.imageFilename);
            %      blds = boxLabelDatastore(trainingData(:,2:end));
            %
            %      % Read and see how the box labels data looks for training.
            %      read(blds)
            %
            %      % Reset before the while-loop.
            %      reset(blds);
            %      while hasdata(blds)
            %         % Read one row of box labels at a time
            %         bxLabels = read(blds);
            %      end
            %
            %   See also imageDatastore, boxLabelDatastore, objectDetectorTrainingData,
            %            readall, read, reset.
            tf = hasdata(obj.InMemoryDatastore);
        end

        function reset(obj)
            %RESET Reset the boxLabelDatastore to the start of the data.
            %   RESET(DS) resets DS to the beginning of the datastore.
            %
            %   Example
            %   -------
            %      % Load training data that contains bounding boxes with labels for vehicles.
            %      data = load('vehicleTrainingData.mat');
            %      trainingData = data.vehicleTrainingData;
            %
            %      % Add fullpath to the local vehicle data folder.
            %      dataDir = fullfile(toolboxdir('vision'), 'visiondata');
            %      trainingData.imageFilename = fullfile(dataDir,...
            %                                              trainingData.imageFilename);
            %      blds = boxLabelDatastore(trainingData(:,2:end));
            %
            %      % Read and see how the box labels data looks for training.
            %      read(blds)
            %
            %      % Reset before the while-loop.
            %      reset(blds);
            %      while hasdata(blds)
            %         % Read one row of box labels at a time
            %         bxLabels = read(blds);
            %      end
            %
            %   See also imageDatastore, boxLabelDatastore, objectDetectorTrainingData,
            %            readall, read, hasdata.
            reset(obj.InMemoryDatastore);
        end

        function data = readall(obj)
            %READALL Read all of the data represented by the boxLabelDatastore.
            %   READALL(DS) reads all of the data represented by the datastore, DS.
            %
            %   Example
            %   -------
            %      % Load training data that contains bounding boxes with labels for vehicles.
            %      data = load('vehicleTrainingData.mat');
            %      trainingData = data.vehicleTrainingData;
            %
            %      % Add fullpath to the local vehicle data folder.
            %      dataDir = fullfile(toolboxdir('vision'), 'visiondata');
            %      trainingData.imageFilename = fullfile(dataDir,...
            %                                              trainingData.imageFilename);
            %      blds = boxLabelDatastore(trainingData(:,2:end));
            %
            %      % Read all of the data from boxLabelDatastore.
            %      readall(blds)
            %
            %   See also imageDatastore, boxLabelDatastore, objectDetectorTrainingData,
            %            read, reset, hasdata.
            data = readall(obj.InMemoryDatastore);
        end

        function data = preview(obj)
            %PREVIEW Read the first row of the data represented by the boxLabelDatastore.
            %   PREVIEW(DS) reads the first row of the data represented by the datastore, DS.
            %
            %   Example
            %   -------
            %      % Load training data that contains bounding boxes with labels for vehicles.
            %      data = load('vehicleTrainingData.mat');
            %      trainingData = data.vehicleTrainingData;
            %
            %      % Add fullpath to the local vehicle data folder.
            %      dataDir = fullfile(toolboxdir('vision'), 'visiondata');
            %      trainingData.imageFilename = fullfile(dataDir,...
            %                                              trainingData.imageFilename);
            %      blds = boxLabelDatastore(trainingData(:,2:end));
            %
            %      % preview the data from boxLabelDatastore.
            %      preview(blds)
            %
            %   See also imageDatastore, boxLabelDatastore, objectDetectorTrainingData,
            %            read, reset, hasdata.
            data = preview(obj.InMemoryDatastore);
        end

        function frac = progress(obj)
            %PROGRESS Return the percentage of read data between 0.0 and 1.0.
            %   PROGRESS(DS) returns a fraction between 0.0 and 1.0 indicating
            %   the progress as a double.
            %
            %   See also imageDatastore, boxLabelDatastore, read, hasdata, reset, readall,
            %   preview.
            %
            %   Example
            %   -------
            %      % Load training data that contains bounding boxes with labels for vehicles.
            %      data = load('vehicleTrainingData.mat');
            %      trainingData = data.vehicleTrainingData;
            %
            %      % Add fullpath to the local vehicle data folder.
            %      dataDir = fullfile(toolboxdir('vision'), 'visiondata');
            %      trainingData.imageFilename = fullfile(dataDir,...
            %                                              trainingData.imageFilename);
            %      blds = boxLabelDatastore(trainingData(:,2:end));
            %
            %      % Read a couple of times, before looking at the progress.
            %      read(blds);
            %      read(blds);
            %
            %      % See the progress made by the boxLabelDatastore.
            %      progress(blds)
            %
            %   See also imageDatastore, boxLabelDatastore, objectDetectorTrainingData,
            %            read, reset, hasdata, readall.
            frac = progress(obj.InMemoryDatastore);
        end

        function subds = partition(obj, N, ii)
            %PARTITION Returns a partitioned portion of the boxLabelDatastore.
            %
            %   SUBDS = PARTITION(DS,N,INDEX) partitions DS into N parts and returns
            %   the partitioned boxLabelDatastore, SUBDS, corresponding to INDEX.
            %   An estimate for a reasonable value for the input N can be obtained
            %   by using the NUMPARTITIONS function.
            %
            %   Example
            %   -------
            %      % Load training data that contains bounding boxes with labels for vehicles.
            %      data = load('vehicleTrainingData.mat');
            %      trainingData = data.vehicleTrainingData;
            %
            %      blds = boxLabelDatastore(trainingData(:,2:end));
            %
            %      % subds contains the first 5 rows of the training data.
            %      subds = partition(blds,59,5);
            %
            %      % If not empty, read the data represented by subds
            %      while hasdata(subds)
            %         % Read one row of box labels at a time
            %         bxLabels = read(subds);
            %      end
            try
                subds = copy(obj);
                subds.InMemoryDatastore = partition(obj.InMemoryDatastore, N, ii);
            catch ME
                throw(ME);
            end
        end

        function tbl = countEachLabel(obj)
            %countEachLabel Count the number of times each unique label occurs.
            %   TBL = countEachLabel(DS) counts the number of times each unique labels
            %   occurs in the boxLabelDatastore, DS.
            %
            %   TBL is a table with the following variables names:
            %
            %        Label        - The class label.
            %
            %        Count        - The number of objects of a given class.
            %
            %        ImageCount   - The total number of images that contain one or more
            %                       instances of a class.
            %
            %   Example
            %   -------
            %   % Load a table that contains bounding boxes with labels for vehicles.
            %   load('vehicleTrainingData.mat');
            %
            %   % Load a table that contains bounding boxes with labels for stop signs
            %   % and cars.
            %   load('stopSignsAndCars.mat');
            %
            %   % Combine ground truth boxes and labels, excluding the image filenames
            %   % in the first column.
            %   vehiclesTbl  = vehicleTrainingData(:, 2:end);
            %   stopSignsTbl = stopSignsAndCars(:, 2:end);
            %
            %   % Create a boxLabelDatastore using 2 tables, one with vehicle label data
            %   % and the other with stop signs label data.
            %   blds = boxLabelDatastore(vehiclesTbl, stopSignsTbl);
            %
            %   tbl = countEachLabel(blds)
            %
            %   % Create a histogram plot using the labels and the respective label counts.
            %   histogram('Categories',tbl.Label,'BinCounts',tbl.Count);
            %
            %   % Create another histogram overlaying the respective image counts.
            %   hold on;
            %   histogram('Categories',tbl.Label,'BinCounts',tbl.ImageCount);
            %
            %   See also pixelLabelDatastore, boxLabelDatastore.
            try
                variableNames = {'Label', 'Count', 'ImageCount'};
                if isempty(obj.LabelData)
                    tbl = table.empty(0,3);
                    tbl.Properties.VariableNames = variableNames;
                    return;
                end
                labels = vertcat(obj.LabelData{:,2});
                count = countcats(labels);
                classes = categories(labels);
                labels = categorical(classes, classes);

                numClasses = numel(labels);
                imageCount = zeros(numClasses,1);

                for ii = 1:size(obj.LabelData,1)
                    idx = ismember(labels,obj.LabelData{ii,2});
                    imageCount(idx) = imageCount(idx) + 1;
                end

                tbl = table(labels, count, imageCount, 'VariableNames', variableNames);
            catch e
                throw(e)
            end
        end

        function subds = subset(obj, indices)
            %subset Returns a subset of the boxLabelDatastore.
            %   subds = subset(blds, indices) partitions blds into the number
            %   of elements in the LabelData and returns a subset containing
            %   the LabelData corresponding to indices.
            %
            %   Example
            %   -------
            %      % Load training data that contains bounding boxes with labels for vehicles.
            %      data = load('vehicleTrainingData.mat');
            %      trainingData = data.vehicleTrainingData;
            %
            %      blds = boxLabelDatastore(trainingData(:,2:end));
            %
            %      % subds contains the first 5 rows of the training data.
            %      subds = subset(blds,1:5);
            %
            %      % If not empty, read the data represented by subds
            %      while hasdata(subds)
            %         % Read one row of box labels at a time
            %         bxLabels = read(subds);
            %      end
            %
            %   See also pixelLabelDatastore, matlab.io.datastore.mixin.Subsettable
            try
                subds = copy(obj);
                subds.InMemoryDatastore = subset(obj.InMemoryDatastore, indices);
            catch ME
                throw(ME);
            end
        end

        function labelData = get.LabelData(obj)
            %GET.LABELDATA Get method for obtaining the LabelData property.
            labelData = obj.InMemoryDatastore.InMemoryData;
        end

        function rSize = get.ReadSize(obj)
            %GET.READSIZE Get method for obtaining the ReadSize property.
            rSize = obj.InMemoryDatastore.ReadSize;
        end

        function set.ReadSize(obj, rSize)
            %SET.READSIZE Set method for setting the ReadSize property.
            obj.InMemoryDatastore.ReadSize = rSize;
        end

        function s = saveobj(obj)
            %SAVEOBJ Save boxLabelDatastore properties to a struct
            s.InMemoryDatastore = saveobj(obj.InMemoryDatastore);
            s.Version           = 1;
        end

    end
    
    methods (Hidden)
        
        function n = numobservations(obj)
            %NUMOBSERVATIONS   the number of observations in boxLabelDatastore
            %
            %   N = NUMOBSERVATIONS(DS) returns the number of observations in
            %   this boxLabelDatastore. This is equal to the number of box labels
            %   in the datastore i.e., size(DS.LabelData,1).
            %   Each box label data in the LabelData is in a row of the Mx2 cell
            %   array. Each row is a single observation.
            %
            %   Example
            %   -------
            %      % Load training data that contains bounding boxes with labels for vehicles.
            %      data = load('vehicleTrainingData.mat');
            %      trainingData = data.vehicleTrainingData;
            %
            %      blds = boxLabelDatastore(trainingData(:,2:end));
            %
            %      % Find the total number of observations in the datastore.
            %      N = numobservations(blds)
            %
            %   See also pixelLabelDatastore, matlab.io.datastore.mixin.Subsettable
            
            n = numobservations(obj.InMemoryDatastore);
        end
        
    end

    methods(Access = protected)
        function N = maxpartitions(obj)
            %MAXPARTITIONS Return the maximum number of partitions
            %   possible for the boxLabelDatastore.
            %
            %   N = MAXPARTITIONS(DS) returns the maximum number of
            %   partitions for a given boxLabelDatastore, DS. This
            %   number is always the height of the in-memory data.
            N = maxpartitions(obj.InMemoryDatastore);
        end


        function displayScalarObject(obj)
            %DISPLAYSCALAROBJECT Custom object display for a scalar boxLabelDatastore.

            % header
            disp(getHeader(obj));
            detailsStr = evalc('details(obj)');
            nsplits = strsplit(detailsStr, '\n');

            group = getPropertyGroups(obj);
            [labelDataIndent, labelDataStrDisp] = iDispMby2Cell(obj.LabelData, nsplits);
            if ~isempty(labelDataStrDisp)
                disp(labelDataIndent);
                disp(labelDataStrDisp);
                % Remove LabelData property from the group, since custom
                % display is used for LabelData.
                group.PropertyList = rmfield(group.PropertyList,'LabelData');
            end
            matlab.mixin.CustomDisplay.displayPropertyGroups(obj, group);
            disp(getFooter(obj));
        end

        % Override copyElement of Copyable.
        function copied = copyElement(obj)
            % Shallow copy of this object.
            copied = copyElement@matlab.mixin.Copyable(obj);
            % Deep copy of the composed handle.
            copied.InMemoryDatastore = copy(obj.InMemoryDatastore);
        end
    end

    methods (Static, Hidden)
        function obj = loadobj(s)
            import vision.internal.cnn.datastore.InMemoryDatastore;
            obj = boxLabelDatastore;
            obj.InMemoryDatastore = InMemoryDatastore.loadobj(s.InMemoryDatastore);
        end
    end
end

function [indentStr, dataStr] = iDispMby2Cell(labelData, nsplits)
    if isempty(labelData)
        indentStr = '';
        dataStr   = '';
    else
        lastLine = string.empty(0,1);
        firstDim = size(labelData, 1);
        if firstDim > 3
            labelData = labelData(1:3, :);
            lastLine = " ... and " + (firstDim - 3) + " more rows";
        end
        firstLine = firstDim + "x2 cell array";
        labelDataStr = nsplits(contains(nsplits, 'LabelData: '));
        % Find the indent spaces from details
        nLabelDataIndent = strfind(labelDataStr{1}, 'LabelData: ') - 1;
        if nLabelDataIndent > 0
            indentStr = [sprintf(repmat(' ',1,nLabelDataIndent)) 'LabelData: '];
            nlspacing = sprintf(repmat(' ',1,numel(indentStr)));
            labelDataStrDisp = evalc('display(labelData)');
            idx = regexp(labelDataStrDisp, 'cell', 'once');
            labelDataStrDisp = labelDataStrDisp(idx+1:end);
            labelDataStrDisp = splitlines(labelDataStrDisp);
            if size(labelDataStrDisp, 1) > 1
                % Remove the line with cell array description
                labelDataStrDisp = labelDataStrDisp(2:end);
                % Strip all empty spaces at the beginning
                labelDataStrDisp = strip(labelDataStrDisp);
                % Remove all empty char arrays
                emptyIndices = cellfun(@isempty, labelDataStrDisp);
                labelDataStrDisp(emptyIndices) = [];
                additionalLines = 3;
                numLines = numel(labelDataStrDisp);
                dataStr = cell(numel(labelDataStrDisp) + additionalLines, 1);
                dataStr(1) = cellstr(firstLine);
                dataStr(2) = {char}; % empty line
                dataStr(3:2+numLines) = labelDataStrDisp;
                dataStr(numLines+3) = {char}; % empty line
                if ~isempty(lastLine)
                    dataStr(end+1) = cellstr(lastLine);
                end
                dataStr = join(append(nlspacing, dataStr), sprintf('\n'));
                dataStr = dataStr{1};
            end
        end
    end
end

%--------------------------------------------------------------------------
function [tables,bSet] = iCheckIfBlockSetIsPresent(varinput)
    % For blockLocationSet to be a valid input, it must be the last argument
    % and there must be at least one input before the last argument.
    if numel(varinput) >= 2 && isa(varinput{end},'blockLocationSet')
        tables = varinput(1:end-1);
        bSet = varinput{end};
    else
        tables = varinput;
        bSet = [];
    end
end

%--------------------------------------------------------------------------
function currentTableNeedsMerge = iCheckAllGroundTruthTables(tables, name)
    numTables = numel(tables);

    % Just to error out, if there are mixed types of tables.
    previousTableNeedsMerge = false;
    currentTableNeedsMerge  = false;
    boxType = zeros(1,numTables);
    
    for ii = 1:numTables
        tbl = tables{ii};
        [currentTableNeedsMerge, boxType(ii)] = iCheckGroundTruthTable(tbl, name, ii);
        if ii == 1
            previousTableNeedsMerge = currentTableNeedsMerge;
        end
        if ~isequal(previousTableNeedsMerge, currentTableNeedsMerge)
            error(message('vision:boxLabelDatastore:invalidMixOfTables'));
        end
    end

    iCheckAllBoxesHaveSameType(boxType);
    
end

%--------------------------------------------------------------------------
function [tblType, boxType] = iCheckGroundTruthTable(gt, name, varIndex)
    % For checking ground truth data, at least one label
    % must be present.
    validateattributes(gt, {'table'},{'nonempty'}, name, 'trainingData', varIndex);

    if iTableIsTypeTwo(gt)
        % Table type 2:
        % Two columns
        %    - bboxes in the 1st column
        %    - labels as categoricals in the 2nd column.
        tblType = 'type2';
        labelsAsString = false;
        boxType = iCheckTableBoxesAndLabels(gt, varIndex, labelsAsString);
    elseif iTableIsTypeThree(gt)
        % Table type 3:
        % Two columns
        %    - bboxes in the 1st column
        %    - labels as strings/cellstr in the 2nd column.
        tblType = 'type3';
        labelsAsString = true;
        boxType = iCheckTableBoxesAndLabels(gt, varIndex, labelsAsString);
    else
        % Table type 1:
        % Each row contains a set of columns of bboxes, one column for each label.
        tblType = 'type1';
        boxType = iCheckTableBoxesInEachRow(gt, varIndex);
    end
end

%--------------------------------------------------------------------------
function iCheckAllBoxesHaveSameType(boxType)
% Check whether all box matrices have the same box format. boxType is
% either 0, 4, 5, or 9. The type is 0 for empty boxes.

% Remove [] boxes;
boxType(boxType == 0) = [];
sameBoxType = ~isempty(boxType) && all(boxType(1) == boxType);
if ~isempty(boxType) && ~sameBoxType
    error(message('vision:boxLabelDatastore:invalidMixOfBoxesInTable'));
end
end

%--------------------------------------------------------------------------
function boxTypePerColumn = iCheckTableBoxesInEachRow(tbl, varIndex)
    boxTypePerColumn = zeros(1,width(tbl));
    
    try
        for i = 1:width(tbl)
            
            boxes = tbl{:, i};
            
            if iscell(boxes)
                
                % Check boxes down the table column.
                boxType = cellfun(@(x)iCheckBoxes(x), boxes);
                
                iCheckAllBoxesHaveSameType(boxType);
                
            else
                error(message('vision:boxLabelDatastore:invalidBoxesInTable'));
            end
            
            boxTypePerColumn(i) = boxType(1);
        end
    catch ME
        msg = message('vision:boxLabelDatastore:invalidDataInTableColumn',varIndex,i);
        baseException = MException(msg);
        baseException = addCause(baseException,ME);
        throw(baseException);
        
    end
    
    msg = message('vision:boxLabelDatastore:invalidDataInTable',varIndex);
    iCheckAllBoxesHaveSameTypeAndThrow(boxTypePerColumn, msg);
    
    % Return box type stored in table.
    boxTypePerColumn = boxTypePerColumn(1); 
end

%--------------------------------------------------------------------------
function iCheckAllBoxesHaveSameTypeAndThrow(boxType,msg)
try
    % Check boxes across columns.
    iCheckAllBoxesHaveSameType(boxType);
catch ME
    baseException = MException(msg);
    baseException = addCause(baseException,ME);
    throw(baseException);
end
end

%--------------------------------------------------------------------------
function boxType = iCheckTableBoxesAndLabels(tbl, varIndex, labelsAsString)
    boxes  = tbl{:,1};
    labels = tbl{:,2};

    if ~iscell(boxes)
        error(message('vision:boxLabelDatastore:invalidBoxesInTable'));
    end
    if ~iscell(labels)
        error(message('vision:boxLabelDatastore:invalidLabelsInTable'));
    end
    
    boxType = zeros(1,height(tbl));
    for ii = 1:height(tbl)
        try
            boxType(ii) = iCheckBoxes(boxes{ii,:}, labels{ii, :}, ii, labelsAsString);
        catch ME
            msg = message('vision:boxLabelDatastore:invalidDataInTableRow',varIndex,ii);
            baseException = MException(msg);
            baseException = addCause(baseException,ME);
            throw(baseException);
        end
    end
    
    msg = message('vision:boxLabelDatastore:invalidDataInTable',varIndex);
    iCheckAllBoxesHaveSameTypeAndThrow(boxType,msg);
    
    % Return boxType stored in table.
    boxType = boxType(1);
    
end

%--------------------------------------------------------------------------
function iAssertValidBBoxFormat(boxes)
valid = isnumeric(boxes) && ~issparse(boxes) && ismatrix(boxes) && any(size(boxes,2) == [4 5 8 9]);
if ~valid
    error(message('vision:boxLabelDatastore:invalidBoxesInTable'));
end
end

%--------------------------------------------------------------------------
function boxType = iCheckBoxes(boxes, labels, rowIndex, labelsAsString)
    if isequal(boxes, [])
        % We want to allow empty boxes in case there are no labels.
        boxType = 0;
        return;
    end

    iAssertValidBBoxFormat(boxes);

    if nargin >= 4
        if size(boxes, 1) ~= size(labels, 1)
            error(message('vision:boxLabelDatastore:invalidNumLabelsInTable', rowIndex));
        end
        if labelsAsString
            iCheckStringLabels(labels, size(boxes, 1));
        else
            iCheckCategoricalLabels(labels, size(boxes, 1));
        end
    end
    
    % Return width of the boxes as an indication of box type.
    boxType = size(boxes,2);
end

%--------------------------------------------------------------------------
function iCheckStringLabels(labels, mSize)
    classes        = {'cell', 'string'};
    attrs          = {'nonsparse', '2d', 'ncols', 1, 'nrows', mSize};
   
    try
        validateattributes(labels, classes, attrs);
    catch me
        error(message('vision:boxLabelDatastore:invalidLabelsInTable'));
    end
    if iscell(labels)
        % If the labels input is a cellstr, check if each element is a valid label.
        iCheckStringLabels(string(labels), mSize);
    end
end

%--------------------------------------------------------------------------
function iCheckCategoricalLabels(labels, mSize)
    classes        = {'categorical'};
    attrs          = {'nonsparse', '2d', 'ncols', 1, 'nrows', mSize};
   
    try
        validateattributes(labels, classes, attrs);
    catch me
        error(message('vision:boxLabelDatastore:invalidLabelsInTable'));
    end
end

%--------------------------------------------------------------------------
function tf = iTableIsTypeTwo(gt)
    tf = width(gt) == 2 && ... % 2-column table
        iscell(gt{1,1}) && ...
        isnumeric(gt{1,1}{1}) && ...% first column is numeric
        iscell(gt{1,2}) && ...
        iscategorical(gt{1,2}{1});% second column is categorical
end

%--------------------------------------------------------------------------
function tf = iTableIsTypeThree(gt)
    tf = width(gt) == 2 && ... % 2-column table
        iscell(gt{1,1}) && ...
        isnumeric(gt{1,1}{1}) && ...% first column is numeric
        iscell(gt{1,2}) && ...
        (isstring(gt{1,2}{1}) || iscellstr(gt{1,2}{1}));% second column is string or cellstr
end

%--------------------------------------------------------------------------
function data = iGetLabelDataFromTypeOne(varargin)
% Tables with bboxes in multiple columns

    import vision.internal.trainingData.mergeLabelBoxes;

    % table with bboxes as multiple columns
    uniqueLabels = cellfun(@(x)x.Properties.VariableNames',...
        varargin, 'UniformOutput', false);
    uniqueLabels = unique(vertcat(uniqueLabels{:}), 'stable');

    % By default all data is converted to a cell array of M-by-2 size,
    % bounding boxes in the 1st column and labels in the 2nd column.
    outputType = 'cell';
    data = cellfun(@(x)mergeLabelBoxes(x,uniqueLabels, outputType),...
        varargin, 'UniformOutput', false);
    data = vertcat(data{:});
end

%--------------------------------------------------------------------------
function data = iGetLabelDataFromTypeTwo(varargin)
% Table with bboxes in the first column,
% categorical labels as second column

    uniqueLabels = {};
    if numel(varargin) > 1
        % Find all categories from each of the tables. The assumption is the first categorical
        % label will contain all the categories for the that table.
        allCategories = cellfun(@(x)categories(x{1,2}{1}), varargin, 'UniformOutput', false);

        % combine all categories and find unique categories.
        uniqueCategories = unique(vertcat(allCategories{:}), 'stable');

        % We need unique labels only when there are additional categories
        % from tables other than the first table (or one of the tables).
        if ~all(ismember(uniqueCategories, allCategories{1}))
            uniqueLabels = uniqueCategories;
        end
    end
    data = iGetLabelDataWithUniqueLabels(varargin, uniqueLabels);
end

%--------------------------------------------------------------------------
function data = iGetLabelDataFromTypeThree(varargin)
% Table with bboxes in the first column,
% string labels as second column

    % Find all categories from each of the tables.
    allCategories = cellfun(@(x)vertcat(x{:,2}{:}), varargin, 'UniformOutput', false);

    % combine all categories and find unique categories.
    uniqueLabels = unique(vertcat(allCategories{:}));

    data = iGetLabelDataWithUniqueLabels(varargin, uniqueLabels);
end

%--------------------------------------------------------------------------
function data = iGetLabelDataWithUniqueLabels(tables, uniqueLabels)
    for ii = 1:numel(tables)
        % Variable names from each table could be different.
        % Make them consistent, so vertcat will work.
        tables{ii}.Properties.VariableNames = {'Var1', 'Var2'};
    end
    data = vertcat(tables{:});
    % Convert from table to cell
    data = [data{:,1}, data{:,2}];

    if ~isempty(uniqueLabels)
        data(:,2) = cellfun(@(x)categorical(x,uniqueLabels), data(:,2), 'UniformOutput', false);
    end
end

%--------------------------------------------------------------------------
function iVerifyImageNumbersAndBoxFormat(data, bSet)
    if isempty(data) || isempty(bSet) || isempty(bSet.BlockOrigin)
        return;
    end
    h = size(data,1);
    if max(bSet.ImageNumber) > h
        error(message('vision:boxLabelDatastore:invalidImageNumberInBlockSet', h));
    end

    % At this point bboxes must be vertcat'able.
    cannotVertcat = false;
    try
        bboxes = vertcat(data{:,1});
    catch
        cannotVertcat = true;
    end

    bboxSize = size(bboxes,2);

    % Make sure only M-by-4 and M-by-5 boxes are allowed with blockLocationSet input.
    if ~isempty(bboxes) && (cannotVertcat || (bboxSize ~= 4 && bboxSize ~= 5))
        error(message('vision:boxLabelDatastore:invalidBoxFormatForBlockSet'));
    end
end

%--------------------------------------------------------------------------
function bboxSize = iGetBoxSize(data)
    bboxes = vertcat(data{:,1});
    bboxSize = size(bboxes,2);
end
