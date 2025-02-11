function errorIf(condition, messageID, varargin)

% Copyright 2024 The MathWorks, Inc.

%#codegen

    if isempty(coder.target)
        if condition
            error(message(messageID, varargin{:}));
        end
    else
        coder.internal.errorIf(condition, messageID, varargin{:});
    end

end