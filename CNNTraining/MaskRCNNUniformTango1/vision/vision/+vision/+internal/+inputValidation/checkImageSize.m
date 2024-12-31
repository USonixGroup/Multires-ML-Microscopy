%checkImageSize Check validity of image size from calibration object
%
%  See also estimateFisheyeParameters, estimateCameraParameters

% Copyright 2017 Mathworks, Inc.
function checkImageSize(I, expectedImageSize)

%#codegen

if ~isempty(expectedImageSize)
    wrongSize = ~isequal([size(I,1),size(I,2)], expectedImageSize);
    if isempty(coder.target)
        if wrongSize
            error(message('vision:calibrate:inconsistentImageSize'));
        end
    else
        coder.internal.errorIf(wrongSize,...
        'vision:calibrate:inconsistentImageSize');            
    end
end
