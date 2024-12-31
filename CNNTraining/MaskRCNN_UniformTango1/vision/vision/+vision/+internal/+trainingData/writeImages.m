%--------------------------------------------------------------------------
function imageNames = writeImages(gTruthVideoOrCustom, writeParams, sampleIndices)
%writeImages writes images from a groundTruth array to disk
%
%   Inputs
%   ------
%   gTruthVideo     - Array of groundTruth objects containing video or
%                     custom data sources.
%   writeParams     - Struct containing fields Location, Prefix and Format
%                     describing parameters used to write images.
%   sampleIndices   - Cell array of the same size as gTruthVideo containing
%                     indices to frames to be written to disk.
%
%   Outputs
%   -------
%   imageNames      - Cell array of full-file image names written to disk.
%

%   Copyright 2017-2024 The MathWorks, Inc.
numVideos   = numel(gTruthVideoOrCustom);
imageNames  = cell(size(gTruthVideoOrCustom));

if numVideos==0
    return;
end

addBackSlash = @(origStr)strrep(origStr,'\','\\');

% Create a message printer to print information about image write.
printer = vision.internal.MessagePrinter.configure(writeParams.Verbose);

printer.linebreak;
printer.printMessage('vision:trainingData:writeBegin', addBackSlash(writeParams.Location));
printer.linebreak;

% Setup parallel execution environment.
if writeParams.UseParallel
    partitionSize = setupParallelExecutionEnvironment();
end

% Write frames from each video.
dsCount = 0;
for n = 1 : numVideos
    dataSource = gTruthVideoOrCustom(n).DataSource;
    samples     = sampleIndices{n};
    numImages   = numel(samples);
    reader      = dataSource.Reader;

    imageFileNames = computeImageNames(dataSource.Source, samples, writeParams, n);

    if isa(reader, 'matlab.io.datastore.ImageDatastore')
        dsCount = dsCount + 1;
        name = strcat('datastore', num2str(dsCount));
        isCustomSource = false;
    else
        name = reader.Name;
        isCustomSource = dataSource.isCustomSource();
    end

    printer.printMessageNoReturn('vision:trainingData:writeSource', numImages, addBackSlash(name));
    printer.print('...');

    if writeParams.UseParallel
        % Write frames in parallel.
        numPartitions = ceil( numImages / partitionSize );
        parfor p = 1 : numPartitions
            indices = ((p-1)*partitionSize + 1) : min(p*partitionSize, numImages);
            readAndWriteFrames(reader, imageFileNames, samples, indices, isCustomSource);
        end
    else
        % Write frames sequentially. 
        indices = 1 : numImages;
        readAndWriteFrames(reader, imageFileNames, samples, indices, isCustomSource);
    end

    imageNames{n} = imageFileNames;

    printer.printMessageNoReturn('vision:trainingData:completed');
    printer.linebreak;
end
end

%--------------------------------------------------------------------------
function readAndWriteFrames(reader, imageFileNames, samples, indices, isCustomSource)
%writeImages writes images from a groundTruth array to disk
%
%   Inputs
%   ------
%   reader          - VideoReader, or custom source reader to read
%                     video/custom source image frames.
%   imageFileNames  - Cell array of strings to store image files.
%   samples         - Numeric vector of time instances at which the
%                     video/custom reader needs to read image frames.
%   indices         - Indices of frames to be written to disk.
%   isCustomSource  - true if custom source, false if not.

for idx = indices
    try
        % Read image
        if isa(reader, 'matlab.io.datastore.ImageDatastore')
            I = read(subset(reader, samples(idx)));
        else
            frame = reader.readFrameAtPosition(samples(idx));
            I = frame.Data;
        end
    catch baseME
        % Throw errors coming from VideoReader or CustomReader
        % object as if they are coming from this function. These
        % are errors related to reading frames.
        fprintf('\n');
        newME = baseME;
        if isCustomSource
            newCauseME = MException('vision:labeler:CustomReaderFunctionCallError',...
                vision.getMessage('vision:labeler:CustomReaderFunctionCallError', func2str(reader.Reader)));
            newME = addCause(newME,newCauseME);
        end
        
        throwAsCaller(newME)
    end
    
    % Write image
    imwrite(I, imageFileNames{idx}); 
end
end

%--------------------------------------------------------------------------
function imageFileNames = computeImageNames(source, samples, writeParams, sourceNumber)

folderLocation = writeParams.Location;

if isempty(writeParams.Prefix)
    
    if isa(source, 'matlab.io.datastore.ImageDatastore')
        namePrefix = strcat('datastore', '_', num2str(sourceNumber), '_');
    else
        % Default source name is the name of the video file.
        namePrefix = source;

        % Remove path and format from name.
        [~,namePrefix,~] = fileparts(namePrefix);
        namePrefix = strcat(namePrefix, '_', num2str(sourceNumber), '_');
    end
else
    namePrefix = strcat(writeParams.Prefix, '_', num2str(sourceNumber), '_');
end

indexString  = strrep(cellstr(num2str(samples(:))), ' ', '0');
formatString = strcat('.', writeParams.Format);
fileNames = strcat(namePrefix, indexString, formatString);

imageFileNames = fullfile( folderLocation, fileNames );

end

%--------------------------------------------------------------------------
function partitionSize = setupParallelExecutionEnvironment()
    % Create a pool
    pool = gcp('nocreate');
    if isempty(pool)
        pool = tryToCreateLocalPool();
    end
    
    if ~isempty(pool)
        partitionSize = pool.NumWorkers;
    else
        partitionSize = 16;
    end
end

%--------------------------------------------------------------------------
function pool = tryToCreateLocalPool()
defaultProfile = ...
    parallel.internal.settings.ProfileExpander.getClusterType(parallel.defaultProfile());

if(defaultProfile == parallel.internal.types.SchedulerType.Local)
    % Create the default pool (ensured local)
    pool = parpool;
else
    % Default profile not local   
    error(message('vision:vision_utils:noLocalPool', parallel.defaultProfile()));    
end
end
