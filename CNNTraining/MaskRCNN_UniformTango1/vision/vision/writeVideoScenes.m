function filenames = writeVideoScenes(gTruth, timeRanges, folderNames, varargin)
%writeVideoScenes Write video scenes to disk.
%
%   filenames = writeVideoScenes(gTruth, timeRanges, folderNames) write sequences
%   of ground truth source data corresponding to each input time range as separate
%   video files. Video files are written to the folders specified by folderNames.
%
%   The output filenames are full paths to the saved video scenes. filenames
%   is an M-by-1 cell array containing array of strings for each groundTruth
%   object, where M equals the number of elements in the gTruth vector.
%   The output filenames are automatically chosen as NamePrefix_<UID>.<EXT>,
%   where UID is a unique integer index for each written scene and EXT is the
%   video file extension supported by the VideoWriter. See name-value pair
%   arguments NamePrefix and VideoProfile for more information.
%
%   [...] = writeVideoScenes(gTruth,timeRanges,rootFolder,folderNames) specify the root
%   folder name to prepend to each of the folder names.
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
%                   timeRanges is an M-by-1 cell array for a groundTruth input or
%                   an M-by-N cell array for a groundTruthMultisignal input.
%                   M is  the number of elements in gTruth and N is the number of
%                   signals in gTruth.
%
%   folderNames     Specify the folder names for each duration to which to
%                   write the video scenes for each duration. Each folder name
%                   is a full path or a relative path from the current folder
%                   to write a video file for each duration. If a folder does
%                   not exist, it will be created. If files with the same file
%                   name exists in a folder, they will be overwritten, for example,
%                   when calling writeVideoScenes function twice with same
%                   input arguments.
%                   folderNames must be an M-by-1 cell array for a groundTruth input
%                   or an M-by-N cell array for a groundTruthMultisignal input.
%                   Each element in the cell array can be an array of strings or
%                   a categorical vector specifying the path to folder names for each
%                   of the time range.
%
%   rootFolder      Specify a root folder name to prepend to each of the
%                   folder names.
%
%   [...] = writeVideoScenes(...,Name,Value) specifies additional name-value
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
%                   Note: The profile "MPEG-4" is only supported on systems with 
%                         Windows 7 or later, or macOS 10.7 and later.
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
%   Example : Extract video scene labels for training a video classifier.
%   ---------------------------------------------------------------------
%   % For more information about extracting video scene labels for 
%   % training a video classifier, see <a href="matlab:helpview('vision','extractTraining')">Extract Training Data for Video Classification</a>.
%
%   See also groundTruthDataSource, groundTruth, sceneTimeRanges,
%            groundTruth/gatherLabelData, objectDetectorTrainingData, pixelLabelTrainingData.

%   Copyright 2021-2023 The MathWorks, Inc

narginchk(3,inf);

% Find rootFolder and folderNames based on nargin
% because rootFolder is optional and folderNames is required.

rootFolder = [];
nvp = {};
givenRootFolder = false;

if nargin > 3
    if mod(numel(varargin), 2) == 0 && iIsScalarText(varargin{1})
        nvp = varargin;
    else
        givenRootFolder = true;
        rootFolder = folderNames;
        folderNames = varargin{1};
        restVarargs = varargin(2:end);
        if ~isempty(restVarargs)
            nvp = restVarargs;
        end
    end
end

% Call with or without root folder input and get argument parameters, as
% arguments block only supports optional arguments as the last arguments.
if givenRootFolder
    params = iGetArgsWithRootFolder(gTruth,timeRanges,rootFolder,folderNames,nvp);
else
    % Call without root folder input and get argument parameters.
    params = iGetArgsWithoutRootFolder(gTruth,timeRanges,folderNames,nvp);
end

try
    filenames = writeSceneVideosForTimeRanges(params.gTruth, params.timeRanges, params.rootFolder, params.folderNames, params.nvp{:});
catch exc
    throw(exc);
end
end

%--------------------------------------------------------------------------
function args = iGetArgsWithRootFolder(gTruth,timeRanges,rootFolder,folderNames,nvp)
    arguments
        % Required inputs.
        gTruth      {mustBeNonempty,mustBeVector}
        timeRanges  {mustBeA(timeRanges, "cell"),mustBeNonempty}
        rootFolder  {iValidateRootFolder(rootFolder)}
        folderNames {mustBeA(folderNames, "cell"),mustBeNonempty}

        % Name-value pairs.
        nvp = {}
    end
    args = iAddToStruct(gTruth,timeRanges,rootFolder,folderNames,nvp);
end

%--------------------------------------------------------------------------
function args = iGetArgsWithoutRootFolder(gTruth,timeRanges,folderNames,nvp)
    arguments
        % Required inputs.
        gTruth      {mustBeNonempty,mustBeVector}
        timeRanges  {mustBeA(timeRanges, "cell"),mustBeNonempty}
        folderNames {mustBeA(folderNames, "cell"),mustBeNonempty}

        % Name-value pairs.
        nvp = {}
    end
    args = iAddToStruct(gTruth,timeRanges,[],folderNames,nvp);
end

%--------------------------------------------------------------------------
function params = iAddToStruct(gTruth, timeRanges, rootFolder, folderNames, nvp)
    params.gTruth = gTruth;
    params.timeRanges = timeRanges;
    params.rootFolder = rootFolder;
    params.folderNames = folderNames;
    params.nvp = nvp;
end

%--------------------------------------------------------------------------
function tf = iIsScalarText(inp)
    tf = ~isempty(inp) && (ischar(inp) && isrow(inp)) || (isstring(inp) && isscalar(inp));
end

%--------------------------------------------------------------------------
function iValidateRootFolder(rootFolder)
    classes = {'string','char'};
    attributes = {'nonempty','scalartext'};
    try
        validateattributes(rootFolder, classes, attributes);
    catch
        error(message('vision:groundTruth:WriteVideoScenesInvalidRootFolder'));
    end
end
