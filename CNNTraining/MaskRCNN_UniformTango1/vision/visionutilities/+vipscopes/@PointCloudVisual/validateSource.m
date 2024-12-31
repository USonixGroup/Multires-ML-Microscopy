function varargout = validateSource(this, source)
%VALIDATESOURCE Validate the source

%   Copyright 2022 The MathWorks, Inc.

if nargin < 2
    source = this.Application.DataSource;
end

% Define the default outputs.
b         = true;
exception = MException.empty;

% If any of the inputs have complex data, return false.
if any(isInputComplex(source))
    b = false;
    id  = 'Spcuilib:scopes:ErrorComplexValuesNotSupported';
    msg = getString(message(id));
    exception = MException(id, msg);
    varargout = {b, exception};
    return;
end

% Get the MaxDimensions for all inputs
maxDims = getMaxDimensions(source);
nInputs = getNumInputs(source);

if numel(maxDims(1,:)) > 2
    if(maxDims(1,3) ~= 3)
        b = false;
       msg = getString(message('vision:block:invalidLocation'));
            exception = MException('vision:block:invalidLocation', msg);
    end
    this.IsOrganized = true;
else
    if(maxDims(1,2) ~= 3)
        b = false;
        msg = getString(message('vision:block:invalidLocation'));
            exception = MException('vision:block:invalidLocation', msg);
    end
    this.IsOrganized = false;
end

if nInputs == 2
    this.ColorProvided = true;
    sampleTimes = getSampleTimes(source);
    if numel(unique(sampleTimes)) ~= 1
        b = false;
        msg = getString(message('vision:block:PointCloudSampleTimeConflict'));
        exception = MException('vision:block:PointCloudSampleTimeConflict', msg);
    end

    if (maxDims(2,:) ~= maxDims(1,:))

        if(maxDims(2,:) ~= [1,3])
            b = false;
            
            msg = getString(message('vision:block:invalidColor'));
            exception = MException('vision:block:invalidColor', msg);
        end
    end

elseif nInputs == 1
    this.ColorProvided = false;

else
    b = false;
   msg = getString(message('vision:block:invalidLocation'));
            exception = MException('vision:block:invalidLocation', msg);
end

if nargout
    varargout = {b, exception};
elseif ~b
    throw(exception);
end

% [EOF]
