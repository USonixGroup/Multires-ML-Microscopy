function filenames = writeSceneVideosForTimeRanges(gTruth, timeRanges, rootFolder, folderNames, nvp)
%writeSceneVideosForTimeRanges Write video scenes to disk.
%
%   filenames = writeSceneVideosForTimeRanges(gTruth, timeRanges, rootFolder, folderNames)
%   write sequences of ground truth source data corresponding to each input time
%   range as separate video files. Video files are written to the folders specified
%   by folderNames.
%
%   The output filenames are full paths to the saved video scenes. filenames
%   is an M-by-1 cell array containing array of strings for each groundTruth
%   object, where M equals the number of elements in the gTruth vector.
%   The output filenames are automatically chosen as NamePrefix_<UID>.<EXT>,
%   where UID is a unique integer index for each written scene and EXT is the
%   video file extension supported by the VideoWriter. See name-value pair
%   arguments NamePrefix and VideoProfile for more information.
%
%   Inputs
%   ------
%   gTruth          A vector of groundTruth or groundTruthMultisignal objects.
%
%   timeRanges      A cell array of duration matrices specifying the time ranges
%                   of ground truth source data. Each duration matrix is of size
%                   T-by-2. Each row in this matrix corresponds to a time range
%                   for which a scene label has been applied in the ground truth.
%                   T is the number of time ranges. Rows in the matrix are of the
%                   form [rangeStart, rangeEnd], where rangeStart and rangeEnd
%                   specify the start and end of a time range for an applied scene
%                   label.
%                   timeRanges is an M-by-1 cell array for a groundTruth input.
%                   M is  the number of elements in gTruth.
%
%   folderNames     Specify the folder names for each duration to which to
%                   write the video scenes for each duration. Each folder name
%                   is a full path or a relative path from the current folder
%                   to write a video file for each duration. If a folder does
%                   not exist, it will be created. If files with the same file
%                   name exists in a folder, they will be overwritten, for example,
%                   when calling writeVideoScenes function twice with same
%                   input arguments.
%                   folderNames must be an M-by-1 cell array for a groundTruth input.
%                   Each element in the cell array can be an array of strings or
%                   a categorical vector specifying the path to folder names for each
%                   of the time range.
%
%   rootFolder      Specify a root folder name to prepend to each of the
%                   folder names.
%
%   [...] = writeSceneVideosForTimeRanges(...,Name,Value) specifies additional name-value
%   pair arguments described below:
%
%   NamePrefix      Specify the output filename prefix for each ground truth as a
%                   an array strings or cell array of character vectors.
%
%                   Default: Filenames (without the extension) in the DataSource
%                   property of the groundTruth input. The input signal names
%                   for groundTruthMultiSignal input.
%
%   VideoProfile    Specify the video profile for writing each video scene.
%                   The video profile is the same as the profile argument
%                   defined by <a href="matlab:doc('VideoWriter')">VideoWriter</a>.
%                   Available options (from VideoWriter):
%                       "Archival"
%                       "Motion JPEG AVI"
%                       "Motion JPEG 2000"
%                       "MPEG-4"
%                       "Uncompressed AVI"
%                       "Indexed AVI"
%                       "Grayscale AVI"
%
%                   Default: "Motion JPEG AVI"
%
%   Verbose         Specify true to display writing progress information in
%                   the command window.
%
%                   Default: true
%   Note:
%   -----
%   - The function writes video scenes using the specified groundTruth
%     objects that contain image data and the corresponding timestamp
%     data. All other non-image data, eg., lidar, are ignored.
%   - In order to select and write specific signals to the specified location,
%     use selectLabelsBySignalName function before calling writeVideoScenes
%     function.
%
%   Example : Gather scene label information and write video scenes.
%   ----------------------------------------------------------------
%   % Load ground truth scene label definitions and label data.
%   data             = load('groundTruthSceneLabels.mat');
%   labelDefinitions = data.labelDefinitions;
%   labelData        = data.labelData;
%
%   % Create ground truth datasource with a video file.
%   gSource = groundTruthDataSource('viptrain.avi');
%
%   % Create a ground truth object.
%   gTruth = groundTruth(gSource, labelDefinitions, labelData);
%
%   % Gather all the scene time ranges and the scene labels.
%   [timeRanges, sceneLabels] = sceneTimeRanges(gTruth);
%
%   % Select a folder in the temp directory to write video scenes.
%   rootFolder = fullfile(tempdir, "videoScenes");
%
%   % Use label names as folder names.
%   folderNames = sceneLabels;
%
%   % Write video scenes to the folder videoScenes. Specify the subfolder
%   % names for each duration as the scene label names.
%   filenames = writeVideoScenes(gTruth, timeRanges, rootFolder, folderNames);
%
%   See also groundTruthDataSource, groundTruth, sceneTimeRanges, writeFrames,
%            gatherLabelData, objectDetectorTrainingData, pixelLabelTrainingData.

%   Copyright 2021-2024 The MathWorks, Inc.

    arguments
        % Required inputs.
        gTruth      {iValidateGroundTruth(gTruth)}
        timeRanges  {iValidateTimeRanges(timeRanges, gTruth)}
        rootFolder
        folderNames {iValidateFolderNames(folderNames,timeRanges, gTruth)}

        % Name-value pairs.
        nvp.NamePrefix   {iValidateNamePrefix(nvp.NamePrefix,gTruth)} = iGetImageSignalNames(gTruth)
        nvp.VideoProfile {mustBeA(nvp.VideoProfile, ["string","char"])} = "Motion JPEG AVI"
        nvp.Verbose      {vision.internal.inputValidation.validateLogical(nvp.Verbose,'Verbose')} = true
    end

    % Format user input into canonical sizes and types.
    rootFolder       = string(rootFolder);
    nvp.NamePrefix   = reshape(string(nvp.NamePrefix), 1, []);
    nvp.VideoProfile = string(iValidateVideoProfile(nvp.VideoProfile, gTruth));
    nvp.Verbose      = logical(nvp.Verbose);

    % Prepend root folder to folder names. 
    folderNames = iPrependRootFolder(rootFolder,folderNames);

    % Prepare the output folders for writing.
    cellfun(@iPrepareOutputLocations,folderNames);

    fileExtension = vision.internal.groundTruth.getFileExtensionFromVideoProfile(nvp.VideoProfile);
    % Generate the output filenames to write.
    filenames = prepareFilePathsForTimeRanges(gTruth, nvp.NamePrefix, folderNames, fileExtension);

    % Configure verbose printing and enable wait bar.
    printer = vision.internal.MessagePrinter.configure(nvp.Verbose);
    numVideos = iNumVideosToWrite(timeRanges);
    enableWaitBar = nvp.Verbose && isempty(dbstatus);
    wb = vision.internal.ConsoleWaitBar( ...
        numVideos, ...               % total number of iterations
        "Verbose",enableWaitBar, ... % whether to print or not
        "DisplayPeriod",2, ...       % refresh every 2 seconds
        "PrintElapsedTime",1, ...    % print elapsed time
        "PrintRemainingTime",1);     % print estimated time remaining

    % Force finish displaying wait bar when we exit this function
    % or if the user sends an interrupt signal.
    cleanUpObj = onCleanup(@() stop(wb));

    
    % Write video scenes.
    printer.printMessage('vision:groundTruth:WriteVideoScenesVerboseStart',...
        numVideos, numel(gTruth));
    
    start(wb);
    for i = 1:numel(gTruth)
        ts = timeRanges{i};
        signalDataSource = gTruth(i).DataSource;
        
        for k = 1:size(ts,1)
            
            % Get indices of the TimeStamps for each of the time ranges.
            indices = iGetTimestampIndicesForRange(ts(k,:), signalDataSource);
            
            filepath = filenames{i}(k);
            iWriteClip(gTruth(i), indices, filepath, nvp.VideoProfile, i);
            
            wb.update();
            
        end
    end
end

%--------------------------------------------------------------------------
function iWriteClip(gTruth, indices, filepath, videoProfile,gTruthIndex)
    writer = VideoWriter(filepath, videoProfile);
    writeFcn = iGetWriteFcn(writer,gTruth);
    writer.open();

    % Clean up writer object, if something is interrupted during the execution
    % of this function.
    cleanUpObj = onCleanup(@() writer.close());
    for k = 1:numel(indices)
        try
            frame = readFrame(gTruth,indices(k));
        catch readException
            msg = message('vision:groundTruth:WriteVideoScenesReadError',string(indices(k)),gTruthIndex);
            me = MException(msg);
            me = addCause(me,readException);
            throw(me);
        end

        try
            writeFcn(writer,frame);
        catch writeException
            msg = message('vision:groundTruth:WriteVideoScenesWriteError',string(indices(k)),gTruthIndex);
            me = MException(msg);
            me = addCause(me,writeException);
            throw(me)
        end
    end
end

%--------------------------------------------------------------------------
function fcn = iGetWriteFcn(writer,gTruth)
    switch string(writer.VideoFormat)
        case "Grayscale"
            fcn = @(writer,frame)iWriteFrameForGrayscale(writer,frame);
        case "Indexed"
            % On validation of input video profile we check if the source
            % is a video source, and if it is an indexed video format.
            colormap = double(gTruth.DataSource.Reader.Colormap);
            % normalize the colormap from the reader and set it on the VideoWriter.
            writer.Colormap =  colormap / 255;
            fcn = @(writer,frame)writeVideo(writer,frame.Data);
        otherwise
            fcn = @(writer,frame)writeVideo(writer,frame.Data);
    end
end

%--------------------------------------------------------------------------
function iWriteFrameForGrayscale(writer,frame)
    if ~ismatrix(frame.Data)
        frame.Data = rgb2gray(frame.Data);
    end
    writeVideo(writer, frame.Data);
end

%--------------------------------------------------------------------------
function indices = iGetTimestampIndicesForRange(timeRange, dataSource)
    timestamps = dataSource.TimeStamps;
    indices = find(timestamps >= timeRange(1) & timestamps <= timeRange(2));
end

%--------------------------------------------------------------------------
function iPrepareOutputLocations(locations)
for i = 1:numel(locations)
    if ~isfolder(locations(i))
        iCreateOutputLocation(locations(i));
    end
end
end

%--------------------------------------------------------------------------
function iCreateOutputLocation(location)
    [success,msg] = mkdir(location);
    if ~success
        error(message('vision:groundTruth:UnableToCreateFolder',location, msg));
    end
end

%--------------------------------------------------------------------------
function folderNames = iPrependRootFolder(rootFolder,folderNames)
    if isempty(rootFolder)
        rootFolder = '';
    end
    appendFcn = @(name)fullfile(rootFolder,string(name));
    folderNames = cellfun(appendFcn,folderNames,'UniformOutput', false);
end

%--------------------------------------------------------------------------
function iValidateTimeRanges(timeRanges,gTruth)
    numGtruth = numel(gTruth);
    timeRangesErrorMsg = 'vision:groundTruth:WriteVideoScenesTimeRangesSizeMismatch';
    iValidateTimeRangesOrFolderNamesCellSize(timeRanges,numGtruth,timeRangesErrorMsg);

    for ii = 1:numGtruth
        timeRange = timeRanges{ii};
        iValidateTimeRangeColumnSize(timeRange,ii);
    end
end

%--------------------------------------------------------------------------
function iValidateFolderNames(folderNames,timeRanges,gTruth)
    numGtruth = numel(gTruth);
    folderNamesErrorMsg = 'vision:groundTruth:WriteVideoScenesFolderNamesSizeMismatch';
    iValidateTimeRangesOrFolderNamesCellSize(folderNames,numGtruth,folderNamesErrorMsg);

    for ii = 1:numGtruth
        timeRange = timeRanges{ii};
        numLabels = size(timeRange,1);
        folderName = folderNames{ii};
        iValidateFolderNameSize(folderName, numLabels, ii);
    end
end

%--------------------------------------------------------------------------
function iValidateTimeRangeColumnSize(timeRange, index)
    classes = {'duration'};
    attributes = {'nonempty', 'nonnan', 'ncols', 2};
    try
        validateattributes(timeRange, classes, attributes);
    catch exc
        msg = message('vision:groundTruth:WriteVideoScenesInvalidTimeRange', string(index));
        me = MException(msg);
        me = addCause(me,exc);
        throw(me);
    end
end

%--------------------------------------------------------------------------
function iValidateFolderNameSize(folderName, numLabels, index)
    classes = {'categorical', 'string'};
    attributes = {'nonempty', 'size', [numLabels,1]};
    try
        validateattributes(folderName, classes, attributes);
    catch
        error(message('vision:groundTruth:WriteVideoScenesInvalidFolderNameSize', string(index)));
    end
end

%--------------------------------------------------------------------------
function iValidateTimeRangesOrFolderNamesCellSize(inp, numGtruth, errorMessage)
    classes = {'cell'};
    attributes = {'nonempty', 'size', [numGtruth,1]};
    try
        validateattributes(inp, classes, attributes);
    catch
        error(message(errorMessage));
    end
end

%--------------------------------------------------------------------------
function iValidateGroundTruth(gTruth)
    numGtruth = numel(gTruth);
    for ii = 1:numGtruth

        if ~istimetable(gTruth(ii).LabelData) || ~any(gTruth(ii).LabelDefinitions.Type == labelType.Scene)
            error(message('vision:groundTruth:SceneTimeRangesNoSceneLabels', string(ii)));
        end
    end
end

%--------------------------------------------------------------------------
function signalName = iGetImageSignalNamesFromGroundTruth(gTruth)
    dataSource = gTruth.DataSource;
    if ischar(dataSource)
        dataSource = string(dataSource);
    elseif iscell(dataSource)
        dataSource = string(dataSource{1});
    elseif isa(dataSource,'groundTruthDataSource')
        dataSource = string(dataSource.Source);
    else
        error('Unknown DataSource');
    end
    [~,signalName] = fileparts(dataSource);
end

%--------------------------------------------------------------------------
function signalNames = iGetImageSignalNames(gTruth)
    signalNames = arrayfun(@iGetImageSignalNamesFromGroundTruth,gTruth,'UniformOutput',false);
    signalNames = vertcat(signalNames{:});
end

%--------------------------------------------------------------------------
function iValidateNamePrefix(x,gTruth)
% Must be non-empty and cellstr or string.
signalNames = iGetImageSignalNames(gTruth);
groundTruth.validateIsStringOrCellstr(x,'NamePrefix');
groundTruth.validateStringValuesAreNotEmpty(x,'NamePrefix');
validateattributes(x,{'string','cell'},{'vector','numel',numel(signalNames)},...
    'writeVideoScenes','NamePrefix');
end

%--------------------------------------------------------------------------
function videoProfile = iValidateVideoProfile(videoProfile, gTruth)
% Must be one of the below values.
profiles = VideoWriter.getProfiles();
validValues = string({profiles.Name});
try
    videoProfile = validatestring(videoProfile,validValues);
catch exc
    if strcmpi(videoProfile,"MPEG-4") && ~any(validValues == "MPEG-4")
        error(message('vision:groundTruth:UnsupportedMPEG4InPlatform', string(videoProfile)));
    end
    throw(exc);
end

iValidateIndexedAVIProfile(videoProfile, gTruth);
end

%--------------------------------------------------------------------------
function total = iNumVideosToWrite(ts)
total = sum(cellfun(@(x)size(x,1),ts),'all');
end

%--------------------------------------------------------------------------
function iValidateIndexedAVIProfile(videoProfile, gTruth)
    if ~isequal(videoProfile, "Indexed AVI")
        return;
    end
    numGtruth = numel(gTruth);
    for ii = 1:numGtruth
        datasource = gTruth(ii).DataSource;
        indexed = isprop(datasource.Reader, 'VideoFormat'); % VideoFormat property must be present
        indexed = indexed && isequal(datasource.Reader.VideoFormat, 'Indexed'); % VideoFormat must be Indexed
        if ~indexed
            error(message('vision:groundTruth:IndexedAVIUnsupportedReader', string(ii)));
        end
    end
end
