function cleanup(this, ~)
%CLEANUP Clean up the visual's HG components

%   Copyright 2022 The MathWorks, Inc.

if ishghandle(this.Axes)
    delete(this.Axes);
end

% [EOF]
