function [imds, pxds] = pixelLabelTrainingData(varargin)
%pixelLabelTrainingData Create training data for semantic segmentation from groundTruth.
%   pixelLabelTrainingData creates datastores that can be used to train a
%   semantic segmentation network from ground truth data. These datastores
%   can be used to both train and evaluate deep learning semantic
%   segmentation networks or classical segmentation methods using
%   trainNetwork, and evaluateSemanticSegmentation.
%
%   [imds, pxds] = pixelLabelTrainingData(gTruth) creates image and pixel
%   datastores from ground truth in gTruth. gTruth is either a scalar
%   groundTruth object, or an array of groundTruth objects. In case of an
%   array of groundTruth objects, the LabelDefinitions in each of them must
%   contain the same pixel label Names. imds is an imageDatastore
%   containing images extracted from the gTruth objects. imds contains
%   images which contain at least one class of pixel labels annotated, and
%   ignores images which are completely unannotated. pxds is a
%   pixelLabelDatastore with categorical matrices for corresponding
%   annotations marked on the images in imds.
%
%   [imds, pxds] = pixelLabelTrainingData(..., Name, Value) specifies
%   additional name-value pair arguments as described below:
% 
%   'SamplingFactor'    The sampling factor used to sub-sample images. A
%                       sampling factor N includes every Nth image in the
%                       ground truth data source that does not contain
%                       empty pixel labels. The sampling factor must be a
%                       numeric scalar or vector.
%
%                          * When the sampling factor is a scalar, the same
%                            sampling factor is applied to all ground truth
%                            data sources in gTruth.
%
%                          * When the sampling factor N is a vector,
%                            sampling factor N(k) is applied to the data
%                            source in gTruth(k).
%
%                       Default: 1
%
%   The name-value pair arguments below control the writing of image files.
%   Using these arguments are valid only when groundTruth objects are
%   created using a video file or a custom data source. They are also valid
%   for an array of groundTruth objects created using imageDatastores with
%   varied custom read functions, and images are written out only for
%   imageDatastores with custom read functions.
%
%   'WriteLocation'     A scalar string or character vector to specify a 
%                       folder location to which extracted image files are
%                       written. The specified folder must exist and have
%                       write permissions.
%
%                       Default: pwd(current working directory)
%
%   'ImageFormat'       A scalar string or character vector to specify the
%                       image file format used to write images. Supported
%                       formats include all those supported by imwrite.
%
%                       Default: 'png'
%
%   'NamePrefix'        A scalar string or character vector to specify the 
%                       prefix applied to output image file names. The
%                       image files are named as
%                       <name_prefix><source_number>_<image_number>.<image_format>.
%
%                       Default: strcat(sourceName, '_'), where sourceName
%                                is the name of the data source from which
%                                the image is extracted for video and
%                                custom data source, or 'datastore' for
%                                image datastore.
%
%   'Verbose'           Set true to display progress.
%
%                       Default: true
%
%
%   Notes
%   -----
%   - The 'WriteLocation', 'ImageFormat', 'NamePrefix', and 'Verbose'
%     name-value pairs are ignored when:
%       - The input  groundTruth object was created from an image sequence
%         data source.
%       - The array of input groundTruth objects all contain image
%         datastores use the same custom read function.
%       - Any of the input groundTruth objects containing datastores, use
%         the default read function.
%
%   - The function supports writing images using multiple MATLAB
%     workers. Enable parallel computing using the <a href="matlab:preferences('Computer Vision Toolbox')">preferences dialog</a>.
% 
%   - For custom data sources in groundTruth, when parallel computing is
%     enabled, the reader function is expected to work with a pool of
%     MATLAB workers to read images from the data source in parallel.
%
%   - Only labels corresponding to pixel labels are returned in 
%     pxds. Other labels are ignored. If the groundTruth object or array
%     does not contain any pixel label data, it throws an error.
%
%   - For an array of groundTruth objects with different Label Definitions
%     (same names but different PixelLabelID), the PixelLabelDatastore
%     created will merge them by name, and generate the right categorical
%     values when using the read method.
%
%   Example: Prepare data for evaluating a semantic segmentation algorithm
%   -----------------------------------------------------------------------
%   % Load a groundTruth object created with pixel labels annotated on a video
%   visiondataPath = fullfile(matlabroot, 'toolbox', 'vision', 'visiondata');
%   addpath(fullfile(visiondataPath, 'triangleImages'));
%   addpath(fullfile(visiondataPath, 'triangleImages', 'testLabels'));
%
%   loadedData = load(fullfile(visiondataPath, 'triangleImages', 'triangleGroundTruth.mat'));
% 
%   % Create an imageDataStore and pixelLabelDatastore from the video file and
%   % corresponding pixel labels.
%   [imds, pxdsTruth] = pixelLabelTrainingData(loadedData.gTruth, 'SamplingFactor', 5, 'WriteLocation', tempdir);
% 
%   % Check that every fifth frame is written to disk
%   imds.Files
%   
%   % Remove video and images from the path
%   rmpath(fullfile(visiondataPath, 'triangleImages'));
%   rmpath(fullfile(visiondataPath, 'triangleImages', 'testLabels'));
%
%   See also objectDetectorTrainingData, groundTruth, semanticseg,
%          pixelLabelDatastore, evaluateSemanticSegmentation.

%   Copyright 2018-2023 The MathWorks, Inc.

if nargin > 0
    [varargin{:}] = convertStringsToChars(varargin{:});
end
% Parse inputs,
[gTruth, samplingFactor, writeParams, idxStruct, labelDefs] = validateInputs(varargin{:});

% find sample indices for video/sequence based gTruths
sampleIndices = sampleDataSource(gTruth, labelDefs, samplingFactor);

% and write out images in specified directory.
imageNames = cell(size(gTruth));
imageNames(idxStruct.isVideoOrCustomOrDSSource) = vision.internal.trainingData.writeImages(gTruth(idxStruct.isVideoOrCustomOrDSSource), writeParams, sampleIndices(idxStruct.isVideoOrCustomOrDSSource));
imageNames(~idxStruct.isVideoOrCustomOrDSSource) = collectImages(gTruth(~idxStruct.isVideoOrCustomOrDSSource), sampleIndices(~idxStruct.isVideoOrCustomOrDSSource), idxStruct);

% Create a synchronized 'master-list' of images and corresponding pixel
% labels. labelDefAssignments map labelDefs to each label matrix in
% pixelList.
[imageList, pixelList, labelDefAssignments] = createCombinedImageAndPixelList(imageNames, gTruth, sampleIndices);

if idxStruct.useCustomReader
    % File extensions for the image files in the datastore
    [~,~,fileExtensions] = fileparts(imageList);
    fileExtensions = unique(fileExtensions);
    imds = imageDatastore(imageList, 'FileExtensions', fileExtensions);
    imds.ReadFcn = gTruth(1).DataSource.Source.ReadFcn;
else
    % Create imageDatastore from imageList cell array.    
    imds = vision.internal.customizedImageDatastore(imageList);
end

% Create pixelLabelDatastore from pixelList cell array.
params.IncludeSubfolders = false;
params.ReadSize = 1;
params.FileExtensions = string(iDefaultFileExtensions);
params.ReadFcn = iGetDefaultReadFcn;

pxds = matlab.io.datastore.PixelLabelDatastore.createFromDirectory(pixelList, labelDefs, labelDefAssignments, params);

end


%--------------------------------------------------------------------------
function sampleIndices = sampleDataSource(gTruth, labelDefs, samplingFactor)
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

sampleIndices = cellfun(@(data, defs, sf)computeSampleIndices(data, defs, sf), labelDatas, labelDefs, samplingFactor, 'UniformOutput', false);

if all(cellfun(@isempty, sampleIndices))
    % labelData contains saved images, but they do not contain any pixel
    % label annotations. Error out in this case.
    error(message('vision:pixelLabelTrainingData:NoPixelLabelsInGroundTruth'));
end

end


%--------------------------------------------------------------------------
function samples = computeSampleIndices(labelData, labelDefs, samplingFactor)
%computeSampleIndices compute indices to sample from non-empty label data
%
%   Inputs
%   ------
%   labelData       - table or timetable of label data from a single
%                     groundTruth object.
%   labelDefs       - label definition for the groundTruth Object
%   samplingFactor  - scalar greater than 1 representing the down sample
%                     factor.
%
%   Outputs
%   -------
%   samples         - row indices into label data to be sampled for
%                     training.
%

if ~isempty(labelDefs)
    labelData = labelData(:, 'PixelLabelData');

    % Find non-empty rows, which can be used for training.
    hasPixelLabelsFcn = @(rowData) hasPixelLabels(rowData, labelDefs.PixelLabelID);
    validIndices = find(rowfun(hasPixelLabelsFcn, labelData, ...
                   'ExtractCellContents', true, 'OutputFormat', 'uniform'));

    % Sample among these rows
    samples = validIndices( 1 : samplingFactor : end );
else
   samples = []; 
end
end


%--------------------------------------------------------------------------
function TF = hasPixelLabels(labelDataRow, labelIDs)
%hasPixelLabels checks if a pixel label matrix has any valid labelIDs
%
%   Inputs
%   ------
%   labelDataRow    - full path to the label matrix that contains pixel
%                     labels.
%   labelIDs        - scalar or vector of pixel label IDs.
%
%   Outputs
%   -------
%   TF              - true if any labelIDs is found in the label matrix, 
%                     false otherwise.
%
TF = false;
try
    im = imread(labelDataRow);
    for idx = 1:length(labelIDs)
        % labelIDs can be a cell array
        if iscell(labelIDs)
            labelID = labelIDs{idx};
        else
        % or a column vector
            labelID = labelIDs(idx,:);
        end
        if size(labelID, 2) == 3
        % labelID is a RGB triplet
            pixelMask =  im(:,:,1) == labelID(1) & ...
                         im(:,:,2) == labelID(2) & ...
                         im(:,:,3) == labelID(3);
            count = nnz(pixelMask);
            if count > 0
                TF = true;
                return;
            end                
        else
        % labelID is scalar grayscale value
            for lIdx = 1:length(labelID)
                if any(im(:) == labelID(lIdx))
                    TF = true;
                    return;
                end
            end 
        end
    end
catch
   % Do nothing: table entry is likely not an image, or doesn't have
   % pixel labels for selected labels.
end

end


%--------------------------------------------------------------------------
function imageNames = collectImages(gTruthCollection, sampleIndices, idxStruct)
%collectImages collect image names to be included in imageDatastore
%
%   Inputs
%   ------
%   gTruthCollection - Array of groundTruth objects containing image
%                      collections or image sequences (sources that don't 
%                      need to be written out to disk).
%   sampleIndices    - Indices at which the image sequence/collection needs
%                      to be sampled.
%
%   Outputs
%   -------
%   imageNames      - Cell array of full-file image names.
%
numSources   = numel(gTruthCollection);
imageNames  = cell(size(gTruthCollection));

if numSources==0
    return;
end

for n = 1 : numSources
    if idxStruct.useCustomReader || isa(gTruthCollection(n).DataSource.Source, 'matlab.io.datastore.ImageDatastore')
        allImageFileNames = gTruthCollection(n).DataSource.Source.Files(sampleIndices{n});
    else
        allImageFileNames = gTruthCollection(n).DataSource.Source(sampleIndices{n});
    end
    imageNames{n} = allImageFileNames;
end

end


%--------------------------------------------------------------------------
function [imageList, pixelList, labelDefAssignments] = createCombinedImageAndPixelList(imageNames, gTruth, sampleIndices)
%createCombinedImageAndPixelList create cell arrays of images and
%corresponding pixel labels, with label definition assignments to each.
%
%   Inputs
%   ------
%   imageNames     - Cell array of image file paths to be included in the
%                    generated image Datastore.
%   gTruth         - Array of groundTruth objects that contain label
%                    definitions and pixel label paths.
%   sampleIndices  - Indices at which the image sequence/collection needs
%                    to be sampled.
%
%   Outputs
%   -------
%   imageNames          - Cell array of full-file image names.
%   pixelList           - Cell array of full-file pixel label matrices.
%   labelDefAssignments - Numeric array matching groundTruth objects to
%                         pixel labels
%
imageList = {};
pixelList = {};
labelDefAssignments = [];
for setIdx = 1:length(imageNames)
    for imIdx = 1:length(imageNames{setIdx})
        imageList{end+1} = imageNames{setIdx}{imIdx}; %#ok<AGROW>
        pixelList{end+1} = gTruth(setIdx).LabelData.PixelLabelData{sampleIndices{setIdx}(imIdx)}; %#ok<AGROW>
        labelDefAssignments(end+1) = setIdx; %#ok<AGROW>
    end
end

assert(length(imageList) == length(pixelList));
end


%--------------------------------------------------------------------------
function [gTruth, samplingFactor, writeParams, idxStruct, labelDefs] = validateInputs(varargin)

inputs = parseInputs(varargin{:});

gTruth                  = inputs.gTruth;
samplingFactor          = inputs.SamplingFactor;
writeParams.Location    = inputs.WriteLocation;
writeParams.Prefix      = inputs.NamePrefix;
writeParams.Format      = inputs.ImageFormat;
writeParams.Verbose     = inputs.Verbose;
writeParams.UseParallel = inputs.UseParallel;

[gTruth,idxStruct, validGTruthIndices, samplingFactor] = ...
    vision.internal.trainingData.checkGroundTruthSources(gTruth, writeParams.Location, samplingFactor);

labelDefs = checkGroundTruthLabelDefinitions(gTruth, validGTruthIndices);

end


%--------------------------------------------------------------------------
function inputs = parseInputs(varargin)

% Defaults
samplingFactor  = 1;
writeLocation   = pwd;
imageFormat     = 'png';
namePrefix      = '';
verbose         = true;

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

parse(parser, varargin{:});

inputs = parser.Results;

if isscalar(inputs.SamplingFactor)
    % expand scalar for each gTruth
    inputs.SamplingFactor = repelem(inputs.SamplingFactor, 1, numel(inputs.gTruth));
end

validateattributes(inputs.SamplingFactor, {'numeric'},...
    {'vector','integer', 'positive', 'numel', numel(inputs.gTruth)}, ...
    mfilename, 'SamplingFactor');

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
function labelDefs = checkGroundTruthLabelDefinitions(gTruthArray, originalIndices)

labelDefs = cell(1, length(gTruthArray));
labelDefsName = [];

for n = 1 : length(gTruthArray)
    if n ~= originalIndices(n)
        % Skip for invalid groundTruth objects
        labelDefs{n} = [];
        continue
    end
    defs = gTruthArray(n).LabelDefinitions;
    defs(gTruthArray(n).LabelDefinitions.Type ~= labelType.PixelLabel, :) = [];
    if ~isempty(defs)
        % strip out Type column and only keep labelType.PixelLabel
        defs(:,2) = [];
        labelDefs{n} = defs;
        
        % Check if names from all gTruth objects are same
        if ~isempty(labelDefsName) && ~isempty(setdiff(labelDefsName, defs.Name))
            error(message('vision:pixelLabelTrainingData:InconsistentLabelDefinitions'))
        end
        
        labelDefsName = defs.Name;
    else
        error(message('vision:pixelLabelTrainingData:NoPixelLabelsInGroundTruth'))
    end
end

end

function defaultExtensions = iDefaultFileExtensions()
 i = imformats;
 defaultExtensions = strcat('.', [i.ext]);
end

function defaultReadFcn = iGetDefaultReadFcn()
% Get the default ReadFcn used by imageDatastore
 temp = imageDatastore({});
 defaultReadFcn = temp.ReadFcn;
end
