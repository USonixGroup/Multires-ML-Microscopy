function createOcvVecFile(dsTrain, totalNumInstances,...
    outputVecFilename, objectTrainingSize)
%function createOcvVecFile(positives, outputVecFilename, sampleWidth, ...
%  sampleHeight) writes an OpenCV style "vec" file to be used for training 
%  a cascade object detector.
%
%    dsTrain             datastore that contains image file names with 
%                        the associated bounding boxes.
% 
%    totalNumInstances   number of bounding boxes in all images, should be
%                        computed in trainCascadeObjectDetector.
%
%    outputVecFilename   name of the output vec file
%
%    objectTrainingSize  [sampleHeight, sampleWidth], the size to which all 
%                        samples will be resized

%   Copyright 2012-2021 The MathWorks, Inc.

vecFile = openOutputVecFile(outputVecFilename);

writeHeader(vecFile, totalNumInstances, objectTrainingSize);

% Copy and reset datastore to prevent modifying input object.
ds = copy(dsTrain);
reset(ds);

% Define transform to crop out patches and write to the vec file.
ds = transform(ds,...
    @(data,info)cropAndWritePatchesToVecFile(data,info,vecFile,objectTrainingSize),...
    'IncludeInfo',true);

try
    % Apply the transform and sum up total number of samples written.
    out = readall(ds);

catch exception
    % Close and delete vec file, then rethrow exception.
    fclose(vecFile.id);
    delete(vecFile.name);
    rethrow(exception.cause{1});
end

% Vec file creation complete. Close file.
fclose(vecFile.id);

% Sanity check. Must always pass.
samplesWritten = sum(vertcat(out{:,1}));
assert(samplesWritten == totalNumInstances);

% Issue warning when some images were not used during training.
missingBoxes = vertcat(out{:,2});
filenames    = vertcat(out{:,3});
issueWarningAboutUnusedImages(missingBoxes, filenames);

%==========================================================================
function [out, info] = cropAndWritePatchesToVecFile(...
    data, info, vecFile, objectTrainingSize)

numImages      = size(data,1);
missingBoxes   = false(numImages,1);
samplesWritten = zeros(numImages,1);
filenames      = strings(numImages,1);

for i = 1:numImages
    img = data{i,1};
    bboxes = data{i,2};

    if isempty(bboxes)
        missingBoxes(i) = true;
        samplesWritten(i) = 0;
        filenames(i) = findFileName(info(i),i);

    elseif ~isempty(img) 

        bboxes = round(bboxes);
      
        img = im2gray(img);
        
        for boundingBoxNum = 1:size(bboxes, 1)
            % Extract the sub-image specified by the bounding box
            imgPatch = cropROI(img, bboxes, boundingBoxNum);

            % Resize the sub-image to [sampleHeight sampleWidth]
            % Note that width and height must be resized to comply with
            % imresize convention.
            imgPatchResize = imresize(imgPatch, objectTrainingSize);

            % Write to image data to vec file
            writeImageInfoToVecFile(imgPatchResize, vecFile.id);
            samplesWritten(i) = samplesWritten(i) + 1;
        end
    end
end
out = {samplesWritten, missingBoxes, filenames};

%==========================================================================
% Create a new output vec file with validation
% Return a struct with fields 
%   name   the file name
%   id     the file id returned by fopen
% Error if unable to create the file
function vecFile = openOutputVecFile(outputVecFilename)
% trainCascadeObjectDetector.m has already checked to ensure that a file
% with this name does not already exist
vecFile.name = outputVecFilename;
vecFile.id = fopen(vecFile.name, 'w');
if(vecFile.id ==  -1)
    error(message('vision:trainCascadeObjectDetector:cannotWriteVecFile'));
end

%==========================================================================
% Write header information to the outputVecFile
function writeHeader(vecFile, totalNumInstances, objectTrainingSize)

fwrite(vecFile.id, totalNumInstances, 'int');
fwrite(vecFile.id, prod(objectTrainingSize), 'int');

% mysterious min/max values from OpenCV
tmp = int16(0);
fwrite(vecFile.id, tmp, 'short');
fwrite(vecFile.id, tmp, 'short');

%==========================================================================
% Crop the ROI specified by the bounding box and validate it
function imgPatch = cropROI(img, boxes, boundingBoxNum)

x      = boxes(boundingBoxNum, 1);
y      = boxes(boundingBoxNum, 2);
width  = boxes(boundingBoxNum, 3);
height = boxes(boundingBoxNum, 4);

% Crop the ROI specified by the bounding box
imgPatch = img(y:y+height-1, x:x+width-1);

%==========================================================================
% Write image intensity data in row-major format as short data type to the
% outputVecFile
function writeImageInfoToVecFile(img, outputVecFile)
% 0 separator
chartmp = uint8(0);
fwrite(outputVecFile, chartmp, 'uchar');
% image data
fwrite(outputVecFile, img', 'short');

%==========================================================================
function filename = findFileName(info,dsRecordIdx)
% Find the filename contained within the datastore read method info struct.
% This function assumes the info holds a field named 'Filename' which is
% used by file-based datastores like imageDatastore. If the filename cannot
% be found, return "".
if ~iscell(info)
    info = {info};
end

filename = "";
for i = 1:numel(info)
    s = info{i};
    for j = 1:numel(s)
        if isfield(s(j),'Filename')
            filename = string(s(j).Filename);
            break
        end
    end
end

if filename ~= ""
    filename = filename(dsRecordIdx);
end

%==========================================================================
function issueWarningAboutUnusedImages(missingBoxes, filenames)
% Extract the unused images and issue an informative warning about not
% using these images during training. 
unusedImages = filenames(missingBoxes);
if ~isempty(unusedImages)
    len = strlength(unusedImages);
    if all(len > 0)
        warning(message('vision:trainCascadeObjectDetector:removedImagesWithoutBoxes',...
            sprintf('%s\n',unusedImages)));
    else
        % We don't have file names to display as the datastore does not use
        % files (this can happen with a custom datastore).
        warning(message('vision:trainCascadeObjectDetector:removedImagesWithoutBoxesNoFilenames',...
            numel(unusedImages)));
    end
end

% LocalWords:  Vec vec datastores
