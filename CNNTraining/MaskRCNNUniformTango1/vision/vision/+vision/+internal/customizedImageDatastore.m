function out = customizedImageDatastore(fileNames, fileExtensions)
%customizedImageDatastore Create imageDatastore that can handle 
% file formats supported by IMREAD and DICOMREAD. Input fileNames is a cell
% array of image filenames as expected by imageDatastore.
%
%   See also matlab.io.datastore.ImageDatastore, datastore, mapreduce.

%   Copyright 2021 The MathWorks, Inc.

% We're using custom reader function to support DICOM when DICOM files are
% present. Otherwise, we'll leave the imageDatastore's default read
% function in place. When the read function is untouched, imageDatastore 
% uses a more efficient image reading procedure that prefetches the data. 
% It's not obvious that this is happening, but that's how it was 
% implemented in the internal of imageDatastore.

if any(contains(fileNames, '.dcm'))
    i = imformats;
    ext = strcat('.', [i.ext]);
    ext(end+1) = {'.dcm'}; % add DICOM

    out = imageDatastore(fileNames, 'ReadFcn',...
        @(x)vision.internal.readLabelerImages(x),...
        'FileExtensions', ext);
else
    if nargin < 2
        out = imageDatastore(fileNames);
    else
        out = imageDatastore(fileNames, 'FileExtensions',fileExtensions);
    end
end

