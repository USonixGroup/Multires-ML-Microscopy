% availableWorldUnits Returns all available world units for camera calibration.

% Copyright 2022 The MathWorks, Inc.

function availableUnits = availableWorldUnits

    availableUnits = {vision.getMessage('vision:caltool:millimeters'),...
                      vision.getMessage('vision:caltool:centimeters'),...
                      vision.getMessage('vision:caltool:inches')};
end