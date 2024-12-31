function requiresNeuralToolbox(myFunction)
% Verify that the Deep Learning Toolbox is available.

%   Copyright 2016-2022 The MathWorks, Inc.

% check if nnet is installed first.
try 
    toolboxdir('nnet');
catch
    exception = MException(message('vision:validation:nnetNotInstalled',myFunction));
    throwAsCaller(exception);
end

% check out a license. Request 2nd output to prevent message printing.
[isLicensePresent, ~] = license('checkout','Neural_Network_Toolbox');

if ~isLicensePresent
    exception = MException(message('vision:validation:nnetLicenseUnavailable',myFunction));
    throwAsCaller(exception);    
end
