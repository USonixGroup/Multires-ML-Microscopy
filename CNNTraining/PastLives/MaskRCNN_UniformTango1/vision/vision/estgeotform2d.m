function [tform, inlierIndex, status] = estgeotform2d(matchedPoints1, matchedPoints2, transformType, varargin)

% Copyright 2022-2023 The MathWorks, Inc.

%#codegen

reportError = (nargout ~= 3);
is2D = true;

[tform, inlierIndex, status] = ...
    vision.internal.geotrans.algEstimateGeometricTransform(...
    matchedPoints1, matchedPoints2, transformType, reportError,...
    'estgeotform2d', is2D, varargin{:});

end