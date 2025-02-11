function objectDetectorBlockWrapper(func, varargin)
%

%   Copyright 2021 The MathWorks, Inc.

vision.internal.requiresNeuralToolbox(message('vision:ObjectDetectorBlock:TheObjectDetectorBlock').getString());
feval(func, varargin{:});

end
