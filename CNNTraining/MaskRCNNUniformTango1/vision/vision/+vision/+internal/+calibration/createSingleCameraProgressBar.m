% createSingleCameraProgressBar Create a progress bar for calibration

% Copyright 2017-2020 The MathWorks, Inc.
function progressBar = createSingleCameraProgressBar(isEnabled, parent)
messages = {'vision:calibrate:initialGuess', 'vision:calibrate:jointOptimization', ...
    'vision:calibrate:calibrationComplete'};
percentages = [0, 0.25, 1];
if nargin > 1 && ~isempty(parent) && ...
        isa((parent), 'matlab.ui.container.internal.AppContainer')
    progressBar = vision.internal.calibration.WebCalibrationProgressBar(isEnabled,...
        parent, messages, percentages);
else
    progressBar = vision.internal.calibration.CalibrationProgressBar(isEnabled,...
        messages, percentages);
end
