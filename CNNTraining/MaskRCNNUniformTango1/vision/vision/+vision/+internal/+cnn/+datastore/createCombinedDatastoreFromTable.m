function ds = createCombinedDatastoreFromTable(tbl, miniBatchSize)
%CREATECOMBINEDDATASTOREFROMTABLE Create an imageDatastore and a
% boxLabelDatastore, that can be combined using combine() to return
% a CombinedDatastore that can emit images, boxes and labels as a
% Mx3 cell array on read methods.

% Copyright 2019 The MathWorks, Inc.
    if nargin == 1
        miniBatchSize = 1;
    end

    imds = imageDatastore(tbl{:, 1});
    bxds = boxLabelDatastore(tbl(:, 2:end));

    imds.ReadSize = miniBatchSize;
    bxds.ReadSize = miniBatchSize;

    ds = combine(imds, bxds);
end
