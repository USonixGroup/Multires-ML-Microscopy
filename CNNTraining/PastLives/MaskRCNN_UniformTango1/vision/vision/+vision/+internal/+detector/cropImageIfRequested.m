function Iroi = cropImageIfRequested(I, roi, usingROI, varargin)
% Crops out and returns the roi from I if usingROI is true. Otherwise, the
% original image is returned. 
%
% The roi should already be validated using vision.internal.detector.checkROI.

%   Copyright 2014-2020 The MathWorks, Inc.

%#codegen
if usingROI
    if isempty(varargin)
        is3D = false;
    else
        is3D = varargin{1};
    end
    Iroi = vision.internal.detector.cropImage(I, roi, is3D);
else
    % return original image. This is a no-op in codegen.    
    Iroi = I;    
end
