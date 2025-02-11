function code = convertKAZEDiffusionToOCVCode(method)
% -------------------------------------------------------------------------
% Convert diffusion method to opencv code
% -------------------------------------------------------------------------

%   Copyright 2017-2020 The MathWorks, Inc.

%#codegen
code = uint8(1); % initialization
switch method
    case 'region'
        code = uint8(1);
    case 'sharpedge'
        code = uint8(0);
    case 'edge'
        code = uint8(2);
end
end
