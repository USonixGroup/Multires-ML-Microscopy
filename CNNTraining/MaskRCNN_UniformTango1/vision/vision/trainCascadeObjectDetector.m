function trainCascadeObjectDetector(varargin)

%  Copyright 2012-2023 The MathWorks, Inc.

if nargin > 0
    [varargin{:}] = convertStringsToChars(varargin{:});
end

parser = parseInputs(varargin{:});

% Check if outputXMLFileName already exists, If 0 is returned, it means
% file already exists and user chose to abort training
if shouldExitBecauseOutputXMLFileExists(parser.Results.outputXMLFileName)
    return;
end

[filenameParams, trainerParams, cascadeParams, boostParams, ...
    featureParams, totalPosInstances] = populateTrainCascadeParams(parser);

% If the second argument is a string, run the resume procedure
if ischar(filenameParams.positiveInstances)
    % Reload the parameters from the interrupted session
    reloadedParameters = reloadInterruptedTrainingParameters(filenameParams);
    % Populate parameters for ocvTrainCascade
    filenameParams = reloadedParameters.filenameParams;
    trainerParams  = reloadedParameters.trainerParams;
    cascadeParams  = reloadedParameters.cascadeParams;
    boostParams    = reloadedParameters.boostParams;
    featureParams  = reloadedParameters.featureParams;    
    if isfield(reloadedParameters, 'negativeImages')        
        filenameParams.negativeImages = reloadedParameters.negativeImages;
    else
        error(message(...
            'vision:trainCascadeObjectDetector:cannotResumeFromOlderVersion'));
    end
    
    bboxes = getAllBoxes(filenameParams.positiveInstances);
    totalPosInstances = size(bboxes,1);
    
% Else normal procedure - Create the required temp files first
else  
    % Create temp folder where results will be saved
    % If function returns 0, it means temporary folder already existed and
    % user chose to not perform any training
    continueTraining = createTempXmlFolder(filenameParams);
    if continueTraining == 0
        return;
    end

    % This try-catch block will delete the temp folder in case the function
    % encounters an error anytime before calling ocvTrainCascade training
    % function. The temp folder may contain negatives description file,
    % positives vec file, and temp mat file
    try
        % If negativeImages is a char array, hence representing a folder, read
        % negative instances from a folder
        if ischar(filenameParams.negativeImagesFolder)
            %Create a negative instances from the negative images folder
            negativeImages = createNegativeInstancesFromFolder( ...
                filenameParams.negativeImagesFolder);
        % Else, it must be a cell array of image filenames. Read negative instances 
        % from these images
        else
            % Create negative instances from image filenames
            negativeImages = filenameParams.negativeImagesFolder;            
        end
    
        filenameParams.negativeImages = negativeImages;        
    
        % Create a positive instances vec file
        % First create a valid filename in the current folder
        filenameParams.positiveVecFilename = fullfile( ...
            filenameParams.tempXmlFoldername, 'positives.vec');   
                
        % Create vec file with the temporary name 
        % Note that createOcvVecFile takes the objectTrainingSize as 
        % [height, width]
        vision.internal.cascadeTrainer.createOcvVecFile(...
            filenameParams.positiveInstances, ...
            totalPosInstances,...            
            filenameParams.positiveVecFilename, ...
            cascadeParams.objectTrainingSize([2,1]));


        % Save the parameters to a mat file
        % in the current directory. This is used in case the user chooses to
        % 'resume' later in case of a crash.
        tempMatFilename = getTempMatFilename(filenameParams);
        save(tempMatFilename, 'filenameParams', 'trainerParams', 'cascadeParams', ...
            'boostParams', 'featureParams', 'negativeImages');
    catch e
        cleanUp(filenameParams);
        throw(e);
    end
        
end

assert(trainerParams.numPositiveSamples <= totalPosInstances);
fprintf('\n');
if isfield(cascadeParams, 'autoObjectTrainingSize') && ...
        cascadeParams.autoObjectTrainingSize
    disp(getString(message('vision:trainCascadeObjectDetector:autoObjectTrainingSize',...
        cascadeParams.objectTrainingSize(2), cascadeParams.objectTrainingSize(1))));
end
disp(getString(message('vision:trainCascadeObjectDetector:maxNumPositiveSamplesUsed', ...
    trainerParams.numPositiveSamples, totalPosInstances)));
disp(getString(message('vision:trainCascadeObjectDetector:maxNumNegativeSamplesUsed', ...
    trainerParams.numNegativeSamples)));
fprintf('\n');

% Run cascade detector OpenCV-mex code
% Use params obtained from parseInputs
ocvTrainCascade(filenameParams, trainerParams, cascadeParams, boostParams, ...
        featureParams);

% Copy the final xml file from tempXmlFolder to outputXmlFilename
copyTempXmlToFinalXml(filenameParams);

% Clean up all temp files and folders
cleanUp( filenameParams );

%
%==========================================================================
% Parse and check inputs
%==========================================================================
function parser = parseInputs(varargin)
% Check that the working directory is writable
[success, attributes] = fileattrib('.');
if ~success || attributes.system == 1 || ~attributes.UserWrite
    error(message('vision:trainCascadeObjectDetector:currentDirNotWriteable'));
end

% Parse the PV pairs
parser = inputParser;

parser.addRequired('outputXMLFileName', @checkOutputXmlFilename);
parser.addRequired('positiveInstances', @checkPositiveInstances);
parser.addOptional('negativeImages', '', @checkNegativeImages);
parser.addParameter('ObjectTrainingSize', 'Auto', ...
    @checkObjectTrainingSize);
parser.addParameter('NegativeSamplesFactor', 2, ...
    @checkNegativeSamplesFactor);
parser.addParameter('NumCascadeStages', 20, ...
    @checkNumCascadeStages);
parser.addParameter('FalseAlarmRate', 0.5, ...
    @checkFalseAlarmRate);
parser.addParameter('FeatureType', 'HOG', ...
    @checkFeatureType);
parser.addParameter('TruePositiveRate', .995, @checkTruePositiveRate);

% If second argument is a string, then we are in "resume" mode
% and the total number of input arguments must be equal to 2
if nargin > 2 && ischar(varargin{2})
    error(message('vision:trainCascadeObjectDetector:invalidResumeCommand'));
end

% Parse input
parser.parse(varargin{:});

%==========================================================================
% Populate the parameters to pass into C++ function ocvTrainCascade()
%==========================================================================
function [filenameParams, trainerParams, cascadeParams, boostParams, ...
    featureParams, totalPosInstances] = populateTrainCascadeParams(parser)

filenameParams.outputXmlFilename = parser.Results.outputXMLFileName;

% Name of the output folder where the trained cascade and intermediate
% cascades will be saved. A temporary mat file will also be saved here.
filenameParams.tempXmlFoldername = getXmlFoldername( filenameParams.outputXmlFilename);

filenameParams.positiveInstances = parser.Results.positiveInstances;

% If positiveInstances is a char array, don't populate any other parameters
if ischar(parser.Results.positiveInstances)
    trainerParams = [];
    cascadeParams = [];
    boostParams = [];
    featureParams =[];
    totalPosInstances = -1;
    return;

elseif istable(filenameParams.positiveInstances)
    validateattributes(filenameParams.positiveInstances, ...
        {'table'}, {'ncols', 2}, mfilename);
    filenameParams.positiveInstances.Properties.VariableNames{1} = ...
        'imageFilename';
    filenameParams.positiveInstances.Properties.VariableNames{2} = ...
        'objectBoundingBoxes';
    filenameParams.positiveInstances = ...
        table2struct(filenameParams.positiveInstances);

    % Wrap struct data in a datastore.
    [filenameParams.positiveInstances, bboxes] = wrapStructInADatastore(...
        filenameParams.positiveInstances);

elseif isstruct(filenameParams.positiveInstances)

    % Wrap struct data in a datastore.
    [filenameParams.positiveInstances, bboxes] = ...
        wrapStructInADatastore(filenameParams.positiveInstances);

elseif isDatastore(filenameParams.positiveInstances)
    % Copy and reset input datastore to prevent modifying input datastore,
    % which is a handle object.
    filenameParams.positiveInstances = copy(filenameParams.positiveInstances);
    reset(filenameParams.positiveInstances);

    % Attach validation transform to check the image and bounding box data
    % output from the datastore read method.
    filenameParams.positiveInstances = transform(filenameParams.positiveInstances, ...
        @checkTrainingDataDuringTraining, 'IncludeInfo', true);

    bboxes = getAllBoxes(filenameParams.positiveInstances);
end

% Size of training examples in pixels (all examples will be resized to this
% size)
if ischar(parser.Results.ObjectTrainingSize) % auto
    cascadeParams.objectTrainingSize = int32(determineObjectTrainingSizeWH(...
        bboxes));
    cascadeParams.autoObjectTrainingSize = true;
else
    % OpenCV defines size as [width, height]    
    cascadeParams.objectTrainingSize ...
        = int32(parser.Results.ObjectTrainingSize([2,1]));
end

% Else populate all other parameters
if isa(parser.Results.negativeImages, 'matlab.io.datastore.ImageDatastore')
    filenameParams.negativeImagesFolder = parser.Results.negativeImages.Files;
else
    filenameParams.negativeImagesFolder = parser.Results.negativeImages;
end

if strcmpi(parser.Results.FeatureType, 'HOG') && ...
        min(cascadeParams.objectTrainingSize) < 16
    error(message(...
        'vision:trainCascadeObjectDetector:objectTrainingSizeTooSmallForHOG'));
end

% Maximum desired false alarm rate at each stage
boostParams.falseAlarmRate           = parser.Results.FalseAlarmRate;

% Minimum desired true positive rate at each stage
boostParams.minHitRate               = parser.Results.TruePositiveRate;

% Number of cascade stages to train
trainerParams.numCascadeStages       = parser.Results.NumCascadeStages;

% Maximum number of positive samples used in each stage
totalPosInstances = size(bboxes,1);
minPositivesForFilteringOut = floor(1 / (1 - boostParams.minHitRate));
if minPositivesForFilteringOut >= totalPosInstances
    trainerParams.numPositiveSamples = totalPosInstances;
else
    trainerParams.numPositiveSamples = ...
        max(floor(totalPosInstances / ...
            (1 + (trainerParams.numCascadeStages - 1) * (1 - boostParams.minHitRate))),...
            minPositivesForFilteringOut);
end

% numNegativeSamples is numPositiveSamples times NegativeSamplesFactor. These
% many negative examples will be used for training each stage.
trainerParams.numNegativeSamples = parser.Results.NegativeSamplesFactor *...
    trainerParams.numPositiveSamples;
minNumSamples = 10;
if trainerParams.numPositiveSamples + trainerParams.numNegativeSamples ...
        <= minNumSamples
    error(message('vision:trainCascadeObjectDetector:notEnoughSamples', ...
        minNumSamples));
end

% Type of features to use HAAR/LBP/HOG
cascadeParams.featureType            = parser.Results.FeatureType;

% Call validatestring again and set featureName to the matched string 
% this will ensure that a partial matching featureName is now saved as its
% full value
cascadeParams.featureType = validatestring(cascadeParams.featureType,...
    {'Haar', 'LBP', 'HOG'});
% Change MATLAB input 'Haar' to C++ expected 'haar'
if strcmp(cascadeParams.featureType, 'Haar')
    cascadeParams.featureType = 'haar';
end

%--------------------------------------------------------------------------
% Other OpenCV parameters which are not exposed in the main interface
%--------------------------------------------------------------------------
% precalcValBufSize - Size of buffer for pre-calculated feature values in MB
trainerParams.precalcValBufSize       = 256;

% precalcIdxBufSize - Size of buffer for pre-calculated feature indices in MB
trainerParams.precalcIdxBufSize       = 256;

% Saves old format cascade for haar features - should be false usually
% Must be false if cascadeParams.featureType is HOG or LBP
trainerParams.oldFormatSave           = false;

% Type of stages to use (Only boosted classifier supported as of now)
% cascade stage type ('BOOST')
cascadeParams.stageType               = 'BOOST';

% boost parameters
% boosting type ('DAB','RAB', 'LB','GAB')
%  'DAB' - Discrete AdaBoost, 'RAB' - Real AdaBoost, 'LB' - LogitBoost,
%  'GAB' - Gentle AdaBoost
boostParams.boostingType              = 'GAB';

% Specifies whether weight trimming is to be used and its weight
boostParams.weightTrimRate            = .95;

% Maximum depth of weak tree - 1 means classifier is a stump
boostParams.maxDepth                  = 1;

% Maximum weak tree count. At each stage, at most these many weak trees will
% be learned in order to achieve the FalseAlarmRate for the stage.
boostParams.maxWeakCount              = 100;

% haarFeature parameters ('BASIC', 'CORE', 'ALL')
%  'BASIC' - Only upright features, 'ALL' - Uses upright along with 45 degree
%   rotated features
featureParams.mode                    = 'BASIC';

%==========================================================================
function tf = checkOutputXmlFilename(str)
validateattributes(str,...
    {'char'},...
    {'nonempty', 'nonsparse', 'vector'},...
    mfilename, 'outputXMLFileName', 1);
% Valid outputXmlFilename must end in '.xml'
if ~strcmpi( str(end-3:end), '.xml')
    error( message('vision:trainCascadeObjectDetector:invalidXmlExtension'));
end
% Valid outputXmlFilename must not have a folder name attached to it
% If '\' or '/' is found, throw error
if ~isempty(strfind(str,'/')) || ~isempty(strfind(str,'\'))
    error( message('vision:trainCascadeObjectDetector:invalidXmlFilepath'));
end
tf = true;

%==========================================================================
function tf = checkPositiveInstances(positiveInstances)
% Check if struct
validateattributes(positiveInstances,...
    {'struct', 'char', 'table',...
    'matlab.io.datastore.Datastore', 'matlab.io.Datastore'},...
    {'nonempty', 'nonsparse'},...
    mfilename, 'positiveInstances', 2);
% If positiveInstances is a struct
if isstruct(positiveInstances)
    %Check the fields
    if(~isfield( positiveInstances, 'imageFilename'))
        error( message('vision:trainCascadeObjectDetector:structFieldNotFound', ...
            'imageFilename'));
    end
    if(~isfield(positiveInstances, 'objectBoundingBoxes'))
        error( message('vision:trainCascadeObjectDetector:structFieldNotFound', ...
            'objectBoundingBoxes'));
    end
    % Else if it is a char array
elseif ischar(positiveInstances)
    % If input string does not exactly match 'resume', throw error
    if ~strcmp(positiveInstances, 'resume')
        error(message('vision:trainCascadeObjectDetector:invalidResumeString'));
    end

elseif isDatastore(positiveInstances)
    % Datastore training data may have optional label data. Check them if
    % they are present. Also permit empty boxes to be present in the
    % training data. Images without boxes are skipped during training. 
    opts.CheckLabels = 'optional';
    opts.AllowEmptyBoxes = true;
    vision.internal.inputValidation.checkGroundTruthDatastore(positiveInstances,opts);

    % The cascade object detector only supports one object class.
    out = preview(positiveInstances);
    if size(out,2) > 2
        cats = categories(out{1,3});
        if numel(cats) > 1
            error(message('vision:ObjectDetector:tooManyClasses'));
        end
    end
end
tf = true;

%==========================================================================
function tf = checkNegativeImages(negativeImages)
validateattributes(negativeImages,...
    {'char', 'cell', 'matlab.io.datastore.ImageDatastore'},...
    {'nonempty', 'nonsparse'},...
    mfilename, 'negativeImages', 3);
% negativeImages can be either a string or a cell array of strings or an
% imageDatastore. 
%
% N.B. We can only support imageDatastore and not "any datastore" as the
% c++ training code does not rely on the datastore to read the files. Here
% we are using an imageDatastore to simply hold a list of files.

% If it is a string, check that the folder exists
if ischar(negativeImages)
    if ~exist(negativeImages, 'dir')
        error(message('vision:trainCascadeObjectDetector:negativesFolderNotFound', ...
            negativeImages));
    end
% If it is an cell array, check that each element is a string and that each
% string is a valid file
else
    if isa(negativeImages, 'matlab.io.datastore.ImageDatastore')
        negativeImages = negativeImages.Files;
    end
    
    % If cell, it must a vector
    validateattributes(negativeImages, ...
        {'cell'}, ...
        {'vector'});
    
    if isempty(negativeImages)
        error(message('vision:trainCascadeObjectDetector:noNegativeImages'));
    end
    % Each element in the cell array must be a string
    disableImfinfoWarnings();
    for i = 1:length(negativeImages)
        if ~ischar(negativeImages{i})
            enableImfinfoWarnings();
            error(message('vision:trainCascadeObjectDetector:invalidNegativesName', ...
                 i));
        else        
            try                
                imfinfo(negativeImages{i});                
            catch
                enableImfinfoWarnings();
                error(message('vision:trainCascadeObjectDetector:cannotOpenImage',...
                    negativeImages{i}, i));
            end
        end
    end
    enableImfinfoWarnings();    
end
tf = true;

%==========================================================================
function tf = checkObjectTrainingSize(objectTrainingSize)
if ischar(objectTrainingSize)
    validatestring(objectTrainingSize, {'Auto'}, mfilename, ...
        'ObjectTrainingSize');
else
    validateattributes(objectTrainingSize,...
        {'numeric'},...
        {'nonempty', 'nonsparse', 'vector', 'integer', 'positive', 'numel', 2, ...
        '>', 3}...
        , mfilename, 'ObjectTrainingSize');
end
tf = true;

%==========================================================================
function tf = checkNegativeSamplesFactor(number)
validateattributes(number,...
    {'numeric'},...
    {'nonempty', 'scalar', 'positive', 'finite'}...
    , mfilename, 'NegativeSamplesFactor');
tf = true;

%==========================================================================
function tf = checkNumCascadeStages(number)
validateattributes(number,...
    {'numeric'},...
    {'nonempty', 'scalar', 'integer', 'positive'}...
    , mfilename, 'NumCascadeStages');
tf = true;

%==========================================================================
function tf = checkFalseAlarmRate(number)
validateattributes(number,...
    {'double', 'single'},...
    {'nonempty', 'scalar', 'real', 'positive', '<=', 1},...
    mfilename, 'StageFalseAlarmRate');
tf = true;

%==========================================================================
function tf = checkFeatureType(featureName)
validatestring(featureName,...
    {'Haar', 'LBP', 'HOG'},...
    mfilename, 'FeatureType');
tf = true;

%==========================================================================
function tf = checkTruePositiveRate(number)
validateattributes(number,...
    {'double', 'single'},...
    {'nonempty', 'scalar', 'real', 'positive', '<=', 1},...
    mfilename, 'StageFalseAlarmRate');
tf = true;

%==========================================================================
function instances = createNegativeInstancesFromFolder(imagesFoldername)

% createNegativeInstancesFromFolder Creates a struct array with information about 
%  image names for all images that exist in the folder called foldername
%  instances = createNegativeInstancesFromFolder(imagesFoldername)
%  Returns a cell array that contains image filenames 
%  All files in the folder that are readable with imread are included in the
%  cell array.

instances = [];
numImages = 0;

% Check if folder exists
if ~exist(imagesFoldername, 'dir')
    error( message('vision:trainCascadeObjectDetector:negativesFolderNotFound', ...
        imagesFoldername));
end

D = dir(imagesFoldername);

for i = 1:size(D,1)
    % If this is not . or ..
    if (D(i).name(1)~='.')
        % Read only valid file extension fileExt
        imgName = D(i).name;
        imgFullName = fullfile(imagesFoldername, imgName); 
        
        % Read image. If empty is returned, then the name is not a valid
        % image filename
        tempImg = vision.internal.cascadeTrainer.readImage(imgFullName);
        
        if ~isempty(tempImg)
            numImages = numImages+1;
            instances{numImages} = imgFullName; %#ok
        end
    end
end

% Throw error if number of images is 0
if numImages==0
    error(message('vision:trainCascadeObjectDetector:noNegativeImages'));
end

%==========================================================================
function [data, info] = checkTrainingDataDuringTraining(data, info)
% This function is applied as a transform to the training datastore and
% runs every time data is read from the datastore during training.

% Standardize data to be a cell array if needed.
data = iConvertToCellIfNeeded(data);

for i = 1:size(data,1)
   
    checkTrainingDataImages(data, info, i);
    checkTrainingDataBoxes(data, info, i);
    checkIfBoxesOutsideImageLimits(data, info, i);

    % N.B. Checking label data is not required during training as the
    % labels are not used. We only check labels once during datastore
    % validation. 
end

%==========================================================================
function checkTrainingDataImages(data, info, idx)
try
    % Image data must be stored in a scalar cell.
    validateattributes(data(idx,1),{'cell'},{'nonempty','scalar'});

    % Cell contents must be numeric.
    validateattributes(data{idx,1},{'double','single','uint8','uint16','logical'}, ...
        {'nonempty','real','nonsparse'});
catch
    throwDetailedError(...
        'vision:ObjectDetector:readOutputInvalidImageClass', info, idx)
end

% Image should be RGB or grayscale.
if ~any(ndims(data{idx,1}) == [2 3]) || ~any(size(data{idx,1},3) == [1 3])
    throwDetailedError(...
        'vision:ObjectDetector:readOutputInvalidRGBOrGrayscaleImage', info, idx)
end

%==========================================================================
function checkTrainingDataBoxes(data, info, idx)
try
    % Must be scalar cell.
    validateattributes(data(idx,2),{'cell'},{'nonempty','scalar'});

    % Cell contents must be real and non-sparse.
    validateattributes(data{idx,2},{'numeric'}, ...
        {'real', 'nonsparse', 'finite'});
catch
    throwDetailedError(...
        'vision:ObjectDetector:readOutputInvalidBboxFileInfo',info, idx);
end

if ~isempty(data{idx,2})
    % Boxes must be stored in a cell and cell contents must be M-by-4 matrices
    if size(data{idx,2},2) ~= 4
        throwDetailedError(...
            'vision:ObjectDetector:readOutputInvalidBboxFileInfo',info, idx);
    end

    try
        % Width and height of the boxes must be positive.
        bboxes = data{idx,2};
        mustBePositive(bboxes(:,3:4))
    catch
        throwDetailedError(...
            'vision:ObjectDetector:readOutputInvalidBboxFileInfo',info, idx);
    end
end

%==========================================================================
function checkIfBoxesOutsideImageLimits(data, info, idx)
img = data{idx,1};
boxes = data{idx,2};
if ~isempty(boxes)
    beyondXLimits = boxes(:,1) + boxes(:,3) - 1 > size(img,2);
    beyondYLimits = boxes(:,2) + boxes(:,4) - 1 > size(img,1);

    if any(boxes < 1,'all') || any(beyondXLimits) || any(beyondYLimits)
        throwDetailedError(...
            'vision:trainCascadeObjectDetector:cannotReadBoundingBox',info,idx);
    end
end

%==========================================================================
function throwDetailedError(msgID, info, dsRecordIdx)
exp = vision.internal.detector.ObjectDetectorDatastoreException(...
    msgID, info, dsRecordIdx);
throw(exp);

%==========================================================================
% Function that returns all the boxes in the datastore instances.
function bboxes = getAllBoxes(instances)

if isstruct(instances)
    bboxes = vertcat(instances(:).objectBoundingBoxes);
else

    % copy and reset datastore to prevent modifying input object.
    ds = copy(instances);
    reset(ds);

    % Apply a selection transform to read out just the box data.
    tds = transform(ds, @(data,info)deal(data(:,2),info), 'IncludeInfo', true);

    try
        % Read all the boxes out of the datastore.
        bboxes = readall(tds);
    catch ME
        % Strip away transform related error as that is an implementation
        % detail.
        throw(ME.cause{1})
    end
    % Combine all boxes together.
    bboxes = vertcat(bboxes{:});
    
end

if size(bboxes,1) == 0
    error( message('vision:trainCascadeObjectDetector:noPositiveExamples'));
end


%==========================================================================
function objectTrainingSizeWH = ...
    determineObjectTrainingSizeWH(bboxes)

ratios = bboxes(:, 3) ./ bboxes(:, 4);
r = median(ratios);
if r > 1
    h = 32;
    w = h * r;
else
    w = 32;
    h = w / r;
end

% OpenCV defines size as [width, height]
objectTrainingSizeWH = round([w, h]);

%==========================================================================
% Function that generates a filename for the temporary mat file where the
% training parameters are saved
function matFilename = getTempMatFilename( filenameParams)
    matFilename = fullfile(filenameParams.tempXmlFoldername, ...
       'parameters.mat');
    
%==========================================================================
% Function that returns a foldername derived from str by stripping out the
% '.xml' in the end
function foldername = getXmlFoldername(str)
foldername = str(1:end-4);
if isempty(foldername)
    error(message('vision:trainCascadeObjectDetector:invalidOutputFilename'));
end
foldername = sprintf('%s_trainCascadeTemp', foldername);

%==========================================================================
% Function that checks if output xml file already exists
% If output already exists, will prompt user to overwrite the existing
% file or exit training without making any changes.
% Returns 0 if user chooses to exit training, 1 if file did not exist, 2 if
% user chose to overwrite existing file
function result = shouldExitBecauseOutputXMLFileExists(outputXmlFilename)
% Check if the outputXmlFilename already exists
if exist(outputXmlFilename, 'file')
    % If it does, then issue prompt to user to overwrite file or return with
    % no changes.
    msg = message('vision:trainCascadeObjectDetector:xmlFileAlreadyExistsPrompt', ...
        outputXmlFilename);
    msg = [getString(msg) promptString()];
    while true
        disp(msg);
        reply = input('', 's');
        if isequal(reply,'1')
            % Delete existing xml file
            disp(getString(message(...
                'vision:trainCascadeObjectDetector:deletingXMLFile',...
                outputXmlFilename)));
            delete(outputXmlFilename);
            result = false;
            break;
        elseif isempty(reply) || isequal(reply,'2')
            result = true;
            break;
        end
    end
    % If outputXmlFile does not exist
else
    result = false;
end

%==========================================================================
% Function that creates a temporary folder where OpenCV will write the
% intermediate stage information
% If folder already exists, will prompt user to overwrite the existing
% folder or exit training without making any changes.
% Returns 0 if user chose to exit training, 1 if folder did not exist, 2 if
% user chose to overwrite existing folder
function result = createTempXmlFolder(filenameParams)
% Check if the tempXmlFolder already exists
if exist(filenameParams.tempXmlFoldername, 'dir')
    % If it does, then issue prompt to user to delete and restart or use
    % 'resume' option
    msg = message('vision:trainCascadeObjectDetector:tempFolderAlreadyExistsPrompt', ...
        filenameParams.outputXmlFilename, filenameParams.outputXmlFilename);
    msg = [getString(msg) promptString()];
    while true
        disp(msg);
        reply = input('', 's');
        
        if isequal(reply,'1')
            % Delete existing folder contents
            disp(getString(message(...
                'vision:trainCascadeObjectDetector:deletingTempDirectory')));
            filenamesToDelete = fullfile(filenameParams.tempXmlFoldername, '*');
            delete(filenamesToDelete);
            result = 1;
            return;
        elseif isempty(reply) || isequal(reply,'2')
            result = 0;
            return;
        end
    end
    % If tempXmlFolder does not exist, create a new one
else
    mkdir (filenameParams.tempXmlFoldername);
    result = 2;
end

%==========================================================================
% Function that cleans up the temp files and folder created during training
function cleanUp( filenameParameters )
% Delete the temp negative images description file, if it exists
if isfield(filenameParameters, 'negativeImagesDescFilename')
    if (exist(filenameParameters.negativeImagesDescFilename, 'file')==2)
        delete(filenameParameters.negativeImagesDescFilename);
    end
end

% Delete the temp vec file, if it exists
if isfield(filenameParameters, 'positiveVecFilename')
    if (exist(filenameParameters.positiveVecFilename, 'file')==2)
        delete(filenameParameters.positiveVecFilename);
    end
end

% Delete the temp mat file where parameters were saved, if it exists
tempMatFilename = getTempMatFilename(filenameParameters);
    if (exist(tempMatFilename, 'file')==2)
        delete(tempMatFilename);
    end
% Delete the tempXmlFolder that contains intermediate cascade stages
deleteFolder(filenameParameters.tempXmlFoldername);

%==========================================================================
% Function that deletes a folder after emptying it
function deleteFolder(foldername)
% Delete existing folder contents
success = rmdir(foldername, 's');
if ~success
    warning(message('vision:trainCascadeObjectDetector:cannotDeleteTempDir', foldername));
end

%==========================================================================
% Function that moves the final cascade xml file from tempXmlFolder to user
% specified outputXmlFilename
function copyTempXmlToFinalXml(params)
finalXmlFilename = fullfile(params.tempXmlFoldername, 'cascade.xml');
success = copyfile(finalXmlFilename, params.outputXmlFilename, 'f');
if ~success
    error(message('vision:trainCascadeObjectDetector:finalXmlWriteError', ...
        params.outputXmlFilename, finalXmlFilename));
end

%==========================================================================
% Function that loads an earlier saved training session. Checks if the 
% positiveInstances field in the params struct is 'resume'. If so, loads the
% earlier saved mat file with all parameters required for ocvTrainCascade 
function parameters = reloadInterruptedTrainingParameters(filenameParams)

% Check that temp xml folder exists
if(~exist(filenameParams.tempXmlFoldername, 'dir'))
    error(message('vision:trainCascadeObjectDetector:cannotFindResumeCascadeFolder', ...
        filenameParams.tempXmlFoldername));
end    
    
% Get temp mat filename
tempMatFilename = getTempMatFilename(filenameParams);
% If this file does not exist, throw error
if(~exist(tempMatFilename, 'file'))
    error(message('vision:trainCascadeObjectDetector:cannotFindTempMatFile', ...
        tempMatFilename));
% Else
else
    % Load the parameters mat file
    parameters = load(tempMatFilename);
    % Ensure that all required temporary files exist
    % Check that positive instances vec file exists
    if(~exist(parameters.filenameParams.positiveVecFilename, 'file'))
        error(message('vision:trainCascadeObjectDetector:cannotFindResumeVecFile', ...
            parameters.filenameParams.positiveVecFilename));
    end 
end

%------------------------------------------------------------------------
function disableImfinfoWarnings()
imfinfoWarnings('off');

%------------------------------------------------------------------------
function enableImfinfoWarnings()
imfinfoWarnings('on');

%------------------------------------------------------------------------
function imfinfoWarnings(onOff)
warnings = {'imageio:tifftagsread:badTagValueDivisionByZero',...
            'imageio:tifftagsread:numDirectoryEntriesIsZero',...
            'imageio:tifftagsread:tagDataPastEOF'};
for i = 1:length(warnings)
    warning(onOff, warnings{i});
end

%--------------------------------------------------------------------------
function tf = isDatastore(ds)
tf = isa(ds,'matlab.io.Datastore') || isa(ds,'matlab.io.datastore.Datastore');

%--------------------------------------------------------------------------
function [out, info] = structToTrainingData(data, info)
% Convert training data from a struct array into a two-column cell array
% where the first column holds image filenames and the second holds
% bounding boxes.
numImages = size(data,1);
out = cell(numImages,2);
for i = 1:numImages

    % Extract and validate filename.
    info.Filename = data{i}.imageFilename;
  
    % Read image.
    out{i,1} = readAndValidateImage(info.Filename);

    % Read box data.
    out{i,2} = data{i}.objectBoundingBoxes;

    checkIfBoxesOutsideImageLimits(out, info, i);
end

%--------------------------------------------------------------------------
function I = readAndValidateImage(filename)
I = vision.internal.cascadeTrainer.readImage(filename);

if isempty(I)
    error( message('vision:trainCascadeObjectDetector:cannotOpenPositiveImage', ...
        filename));
end

%--------------------------------------------------------------------------
function validateFilenameAndBoxes(s)
% Validate that current image name is valid. If error, catch it and
% throw error from catalog
for i = 1:numel(s)
    try
        validateattributes(s(i).imageFilename,...
            {'char'},...
            {'nonempty', 'nonsparse', 'vector'},...
            mfilename, 'outputXMLFileName', 1);
    catch e
        error( message('vision:trainCascadeObjectDetector:invalidImageFilename', i));
    end

    bboxes = s.objectBoundingBoxes;
    if ~isempty(bboxes)
        if (~ismatrix(bboxes)) || (size(bboxes, 2)~=4)
            error( message('vision:trainCascadeObjectDetector:invalidBoundingBoxes', ...
                i, s(i).imageFilename));
        end
    end
end

%--------------------------------------------------------------------------
function [ds, bboxes] = wrapStructInADatastore(s)
% Package a vector of struct data into an arrayDatastore and add the
% validation transform. All remaining code will process data using the
% datastore. This consolidates the code and avoids branches for
% different container types (datastore or table).
%
% Also return all the boxes, which are needed to compute additional
% training parameters.

% Count the bounding boxes before wrapping in a datastore to avoid
% reading images from datastore. Once wrapped in a datastore bypassing the
% image read operation is not possible.
bboxes = getAllBoxes(s);

validateFilenameAndBoxes(s);

% Wrap struct in a datastore.
s = reshape(s,[],1);
ds = arrayDatastore(s, 'IterationDimension', 1);
ds = transform(ds, @structToTrainingData, 'IncludeInfo', true);

% N.B. Table and struct data is validated before conversion into a
% datastore. This allows us to throw error messages that are related to
% struct and table input data. 

%--------------------------------------------------------------------------
function data = iConvertToCellIfNeeded(data)
if istable(data)
    data = table2cell(data);
elseif isnumeric(data)
    data = {data};
end

%--------------------------------------------------------------------------
function prompt = promptString()
% Prompt users to enter input in the command window when running in the
% live editor. This is required until INPUT is supported in live scripts.
if feature('LiveEditorRunning')
    prompt = message('vision:trainCascadeObjectDetector:promptChoiceLiveEditor');
else
    prompt = message('vision:trainCascadeObjectDetector:promptChoice');
end
prompt = getString(prompt);

% LocalWords:  grayscale nd Haar LBP Labeler visiondata datastores imds blds im bbox CVPR Lienhart
% LocalWords:  Kuranov Vadim Pisarevsky DAGM Dalal Triggs Ojala Pietikainen Maenpaa Multiresolution
% LocalWords:  ocv vec Writeable PV ncols HAAR haar precalc Buf Logit nonsparse Filepath foldername
% LocalWords:  imageio tifftagsread
