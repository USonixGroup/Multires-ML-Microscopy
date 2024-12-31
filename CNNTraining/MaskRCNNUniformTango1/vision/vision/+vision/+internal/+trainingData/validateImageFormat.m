%--------------------------------------------------------------------------
function TF = validateImageFormat(fmt)

%   Copyright 2017 The MathWorks, Inc.
try
    fmtStruct = imformats(fmt);
catch
    error(message('vision:trainingData:unsupportedImageFormat',fmt))
end
if isempty(fmtStruct)
    error(message('vision:trainingData:unsupportedImageFormat',fmt))
end

TF = true;
end