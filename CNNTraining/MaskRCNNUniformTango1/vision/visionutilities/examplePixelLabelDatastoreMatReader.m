function data = examplePixelLabelDatastoreMatReader(filename)
% This function helps read data from a MAT file.
%
% data = examplePixelLabelDatastoreMatReader(filename) loads a MAT file and
% returns the first variable saved in that file.

% Copyright 2018 The MathWorks, Inc.
inp = load(filename);
f = fields(inp);
data=inp.(f{1});
end