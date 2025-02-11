function data = readLabelerImages(filename)
%readLabelerImages Read file formats supported by IMREAD and DICOMREAD.
%
%   See also matlab.io.datastore.ImageDatastore, datastore, mapreduce.

%   Copyright 2021 The MathWorks, Inc.

% Turn off warning backtrace before calling imread
onState = warning('off', 'backtrace');
c = onCleanup(@() warning(onState));
N = 10;
% This implementation follows recommended pattern from the default readFcn
% of imageDatastore.
for iter = 1 : N
    try
        if isdicom(filename)
            [data, map] = dicomread(filename);
        else
            [data, map] = imread(filename);
        end
        if ~isempty(map)
            data = ind2rgb(data, map);
        end
        break;
    catch ME
        pause(10^-6);
        if iter == N
            throw(ME);
        end
    end
end