function tf = isCodegen()
% Determine if MATLAB Coder is in use

%   Copyright 2015-2020 The MathWorks, Inc.

%#codegen

tf = ~isempty(coder.target);
