%--------------------------------------------------------------------------
function [gTruth,idxStruct,validGTruthIndices,samplingFactor] = checkGroundTruthSources(gTruth, writeLocation, samplingFactor)
% Check ground truth sources.

%   Copyright 2017-2023 The MathWorks, Inc.
% Discard empty data sources
invalidGTruth = arrayfun(@(x)~x.hasValidDataSource(),gTruth);

if all(invalidGTruth)
    error( message( 'vision:trainingData:NoGroundTruthSources') )
end

gTruth(invalidGTruth) = [];
samplingFactor(invalidGTruth) = [];

validGTruth = ~invalidGTruth;
validGTruth(invalidGTruth) = [];
validGTruthIndices = find(validGTruth);

% Check if any sources contain ImageDatastores that need to be written out
[isDSToWrite, isDefaultDS, useCustomReader] = dataStoreToWrite(gTruth, validGTruthIndices);

% Check if any sources point to videos or custom data sources
isVideo = arrayfun(@(x)gTruth(x).DataSource.isVideoFileSource,validGTruthIndices);
isCustomSrc = arrayfun(@(x)gTruth(x).DataSource.isCustomSource,validGTruthIndices);

% Construct the struct with indices info
idxStruct.isVideoOrCustomOrDSSource = isVideo | isCustomSrc | isDSToWrite;
idxStruct.isDefaultDS = isDefaultDS;
idxStruct.useCustomReader = useCustomReader;

if any(idxStruct.isVideoOrCustomOrDSSource)
    vision.internal.inputValidation.checkWritePermissions(writeLocation);
end

end

%--------------------------------------------------------------------------
% Get datastore sources with default read functions, custom read functions
% and those that need images to be written out
function [isDSToWrite, isDefaultDS, useCustomReader] = dataStoreToWrite(gTruth, validIdx)

isDSSource = arrayfun(@(x)gTruth(x).DataSource.isImageDatastore,validIdx);

idxDS = find(isDSSource);
idxDefaultDS = arrayfun(@(x)...
    vision.internal.isDefaultImdsReadFcn(gTruth(x).DataSource.Source),idxDS);

% Datastores with custom read function
isDSToWrite = isDSSource;
isDSToWrite(idxDS(idxDefaultDS)) = 0;

% Datastores with default read function
isDefaultDS = xor(isDSSource,isDSToWrite);

% Output imageDatastore with custom read function if all gTruth objects
% have datastore sources with the same reader
useCustomReader = false;
if nnz(isDSToWrite) < numel(validIdx)
    return;
else
    for idx = 1:validIdx(1) - 1
        if ~isequal(gTruth(validIdx(idx)).DataSource.Source.ReadFcn, ...
                gTruth(validIdx(idx + 1)).DataSource.Source.ReadFcn)
            return;
        end
    end
    isDSToWrite(isDSToWrite) = 0;
    useCustomReader = true;
end

end
