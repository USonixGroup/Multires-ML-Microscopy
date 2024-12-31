% This class is for internal use only and may change in the future.

% Copyright 2017 The MathWorks, Inc.
function uuid = getUniqueID()
    % getUniqueID returns a unique ID on each call, to be used in cases
    % where each item in a list needs to be assigned non-overlapping IDs. 
    uuid = char(matlab.lang.internal.uuid());
end