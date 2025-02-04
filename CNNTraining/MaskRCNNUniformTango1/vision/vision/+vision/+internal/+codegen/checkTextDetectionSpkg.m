function isSpkgInstalled = checkTextDetectionSpkg()
% Copyright 2023-2024 The MathWorks, Inc.

% Check if support package is installed for codegen version of
% detectTextCRAFT
breadcrumbFile = 'vision.internal.cnn.supportpackages.IsTextDetectionInstalled';
fullPath = which(breadcrumbFile);
if isempty(fullPath)
    isSpkgInstalled = 0;
else
    isSpkgInstalled = 1;
end
end