function varargout = bundleAdjustmentStructure(varargin)

% Copyright 2019-2023 The MathWorks, Inc.

%#codegen
nargoutchk(0, 3);
narginchk(4, 14);
[varargout{1:nargout}] = vision.internal.bundleAdjust.sparseBA(...
    'structure', mfilename, varargin{:});
