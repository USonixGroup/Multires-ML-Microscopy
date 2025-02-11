function info = imageInformationFromDatastore(datastore, params)
% Returns average image info and other info, for a given groundTruth
% datastore.

% Copyright 2019 The MathWorks, Inc.

% Copy and reset the given datastore, so external state events are
% not reflected.
datastore = copy(datastore);
reset(datastore);

% For all the datastores, we have a isPartitionable method that depicts
% whether we can use partition and numpartition methods.
if isPartitionable(datastore) && params.UseParallel
    out = partitionAndGetInformation(datastore, params);
else
    out = readThroughAndGetInformation(datastore, params);
end
numImages = size(out.sizes, 1);
info.Sizes = out.sizes;
info.AverageImage = single(out.accum ./ numImages);

%--------------------------------------------------------------------------
function info = partitionAndGetInformation(datastore, params)
N = numpartitions(datastore,gcp);
infos = cell(N, 1);
parfor ii = 1:N
    subds = partition(datastore, N, ii);
    infos{ii} = readThroughAndGetInformation(subds, params);
end
infos = vertcat(infos{:});
info.sizes = [infos.sizes];
info.accum = sum([infos.accum]);

%--------------------------------------------------------------------------
function info = readThroughAndGetInformation(datastore, params)
accum = zeros(1,1,params.InputSize(3));
% figure out scaling for each image
k = 1;
isAnImageDatastore = isa(datastore, 'matlab.io.datastore.ImageDatastore');
if isAnImageDatastore
    numImages = numpartitions(datastore);
    if numImages > 1
        datastore.ReadSize = min(10, numImages);
    end
end

reset(datastore);
while hasdata(datastore)
    batch = read(datastore);
    batchSize = size(batch, 1);
    for i = 1:batchSize
        I = batch{i, 1};

        % Return size information for each image.
        [M, N, ~] = size(I);
        sz(k,:) = [M N];

        % Compute per channel mean
        accum = accum + vision.internal.cnn.utils.perChannelMean(I, params.InputSize);

        k = k + 1;
    end
end
info.sizes = sz;
info.accum = accum;
