function requiresStatisticsToolbox(myFunction)
% Verify that the Statistics and Machine Learning Toolbox is available.

%   Copyright 2014-2022 The MathWorks, Inc.

% check if stats is installed first.
try 
    toolboxdir('stats');
catch
    exception = MException(message('vision:validation:statsNotInstalled',myFunction));
    throwAsCaller(exception);
end

% check out a license. Request 2nd output to prevent message printing.
[isLicensePresent, ~] = license('checkout','Statistics_Toolbox');

if ~isLicensePresent
    exception = MException(message('vision:validation:statsLicenseUnavailable',myFunction));
    throwAsCaller(exception);    
end
