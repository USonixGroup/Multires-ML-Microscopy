function requiresROSToolbox(myFunction)
% Verify that the ROS Toolbox available.

%   Copyright 2019-2022 The MathWorks, Inc.

% check if toolbox is installed first.
try 
    toolboxdir('ros');
catch
    exception = MException(message('vision:validation:rosNotInstalled',myFunction));
    throwAsCaller(exception);
end

% check out a license. Request 2nd output to prevent message printing.
[isLicensePresent, ~] = license('checkout','Ros_Toolbox');

if ~isLicensePresent
    exception = MException(message('vision:validation:rosLicenseUnavailable',myFunction));
    throwAsCaller(exception);    
end
