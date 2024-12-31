function tf = isDefaultImdsReadFcn(in)
% Return true if the input imageDatastore ReadFcn or function handle is the
% default read function, readDatastoreImage.

% Copyright 2021 The MathWorks, Inc.

persistent readFunctionPath
if isempty(readFunctionPath)
    imdsClassLocation = fileparts( which('matlab.io.datastore.ImageDatastore') );
    readFunctionPath = fullfile(imdsClassLocation, 'private', 'readDatastoreImage.m');
end

if isa(in,'function_handle')
    readFcn = functions(in);
else
    readFcn = functions(in.ReadFcn);
    
end
tf = isfield(readFcn, 'file') && isequal(readFcn.file, readFunctionPath);
tf = tf && isfield(readFcn, 'parentage') && isequal(readFcn.parentage, {'readDatastoreImage'});