function isInstalled = ocrSpkgInstalled()
% Cache the existence of the support package

%   Copyright 2015-2020 The MathWorks, Inc.

%#codegen

spkgFile = coder.internal.const('vision.internal.ocr.isOCRSupportPackageInstalled');
if vision.internal.ocr.isCodegen()
    exst = ~isempty(vision.internal.codegen.which(spkgFile));
    isInstalled = logical(exst);
else
    isInstalled = logical(~isempty(which(spkgFile)));
end
