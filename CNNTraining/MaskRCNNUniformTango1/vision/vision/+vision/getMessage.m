% Utility function for deriving UI strings from message catalog
function s = getMessage(id, varargin)

%   Copyright 2013-2020 The MathWorks, Inc.

  s = getString(message(id,varargin{:}));
end
