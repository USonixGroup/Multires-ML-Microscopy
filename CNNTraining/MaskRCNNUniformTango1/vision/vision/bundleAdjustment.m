function varargout = bundleAdjustment(varargin)

% Copyright 2015-2023 The MathWorks, Inc.

%#codegen

nargoutchk(0, 4);
narginchk(4, 18);
[varargout{1:nargout}] = ...
    vision.internal.bundleAdjust.sparseBA('full', mfilename, varargin{:});