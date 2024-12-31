classdef selectStrongestValidation
    % Shared validation functions for selectStrongestBbox and selectStrongestBboxMulticlass.

    %  Copyright 2017-2021 The MathWorks, Inc.

    %#codegen
    %#ok<*EMCLS>
    %#ok<*EMCA>
    methods(Static)

        %==========================================================================
        function [ratioType, overlapThreshold, numStrongest] = parseOptInputs(varargin)
            persistent defaults
            if isempty(defaults)
                defaults = struct(...
                    'RatioType', 'Union',...
                    'OverlapThreshold', 0.5,...
                    'NumStrongest', inf);
            end
            iValidateNumNameValueArg(varargin{:});


            validNames = ["OverlapThreshold" "RatioType" "NumStrongest"];
            % Parse input
            name = varargin(1:2:end);
            val  = varargin(2:2:end);

            params = defaults;
            name = iValidateNames(name,validNames);
            for i = 1:numel(name)
                params.(name(i)) = val{i};
            end

            ratioType        = params.RatioType;
            overlapThreshold = params.OverlapThreshold;
            numStrongest     = params.NumStrongest;
        end

        %--------------------------------------------------------------------------
        function [ratioType, overlapThreshold, numStrongest] = ...
                parseOptInputsCodegen(varargin)

            defaults = struct(...
                'RatioType', 'Union',...
                'OverlapThreshold', 0.5,...
                'NumStrongest', inf);

            if ~isempty(varargin)
                % Set parser inputs
                params = struct( ...
                    'RatioType',             uint32(0), ...
                    'OverlapThreshold',      uint32(0), ...
                    'NumStrongest',          uint32(0));

                popt = struct( ...
                    'CaseSensitivity', false, ...
                    'StructExpand',    true);

                % Parse parameter/value pairs
                optarg = eml_parse_parameter_inputs(params, popt, varargin{:});

                ratioType = eml_get_parameter_value(optarg.RatioType, ...
                    defaults.RatioType, varargin{:});

                overlapThreshold  = eml_get_parameter_value(optarg.OverlapThreshold, ...
                    defaults.OverlapThreshold, varargin{:});

                numStrongest = eml_get_parameter_value(optarg.NumStrongest, ...
                    defaults.NumStrongest, varargin{:});

            else
                ratioType        = defaults.RatioType;
                overlapThreshold = defaults.OverlapThreshold;
                numStrongest     = defaults.NumStrongest;
            end
        end

        %==========================================================================
        function checkInputBboxAndScore(bbox, score,fname)
            % Validate the input box and score

            validateattributes(bbox,{'uint8', 'int8', 'uint16', 'int16', 'uint32', ...
                'int32', 'double', 'single'}, {'real','nonsparse','finite'}, ...
                fname, 'bbox', 1);

            validateattributes(score,{'uint8', 'int8', 'uint16', 'int16', 'uint32', ...
                'int32', 'double', 'single'}, {'real','nonsparse','finite','size',[NaN, 1]}, ...
                fname, 'score', 2);

            coder.internal.errorIf(~any(size(bbox,2) == [4 5]),...
                'vision:bbox:invalidBoxFormat');

            coder.internal.errorIf(size(bbox,1) ~= size(score,1),...
                'vision:visionlib:unmatchedBboxAndScore');

            coder.internal.errorIf(any(bbox(:,3)<=0) || any(bbox(:,4)<=0),...
                'vision:visionlib:invalidBboxHeightWidth');
        end

        %==========================================================================
        function checkRatioType(value)
            % Validate the input ratioType string
            coder.internal.errorIf( ~(ischar(value) || (isscalar(value) && isstring(value))),...
                'vision:selectStrongestBbox:invalidRatioType');

            list = {'union','min'};
            coder.internal.errorIf( ~any(strncmpi(list,value,1)), ...
                'vision:selectStrongestBbox:invalidRatioType');

        end

        %==========================================================================
        function checkOverlapThreshold(threshold,fname)
            % Validate 'OverlapThreshold'. Use custom checks for
            % performance.
            if isfloat(threshold) && ...
                    isscalar(threshold) && ...
                    ~isempty(threshold) && ...
                    isfinite(threshold) && ...
                    ~issparse(threshold) && ...
                    isreal(threshold) && ...
                    threshold >= 0 && ...
                    threshold <= 1
            else
                % Reuse validateattributes to throw error.
                validateattributes(threshold, {'single', 'double'}, {'nonempty', 'nonnan', ...
                    'finite', 'nonsparse', 'real', 'scalar', '>=', 0, '<=', 1}, ...
                    fname, 'OverlapThreshold');
            end
        end

        %==========================================================================
        function isvalid = isValidNumStrongestScalar(N)
            % N must be inf or a positive integer.
            % Check whether N is inf or a scalar integer value.
            N = N(1); % force scalar value.
            if isinf(N) || (isfinite(N) && ...
                    isreal(N) && ...
                    ~issparse(N) && ...
                    N > 0 && ...
                    N == floor(N))

                isvalid = true;
            else
                isvalid = false;
            end
        end

        %==========================================================================
        function isvalid = isValidNumStrongestVector(N)
            % N must be a vector of positive integers or inf.
            if isreal(N) && ~issparse(N) && ...
                    all(N(:) > 0) && all(N(:) == floor(N(:)))
                isvalid = true;
            else
                isvalid = false;
            end
        end

        %==========================================================================
        function checkNumStrongestScalar(N)
            % Validate 'NumStrongest', which can be inf or a positive
            % scalar integer. Use custom checks for performance.

            if isnumeric(N) && ~isempty(N) && isscalar(N)
                isvalid = vision.internal.detector.selectStrongestValidation.isValidNumStrongestScalar(N);
            else
                isvalid = false;
            end

            coder.internal.errorIf(~isvalid,...
                'vision:selectStrongestBbox:invalidNumStrongestScalar');

        end

        %==========================================================================
        function checkNumStrongestScalarOrVector(N)
            % Validate 'NumStrongest', which can be inf or a positive
            % scalar integer. Use custom checks for performance.

            if isnumeric(N) && ~isempty(N)
                if isscalar(N)
                    isvalid = vision.internal.detector.selectStrongestValidation.isValidNumStrongestScalar(N);
                elseif isvector(N)
                    isvalid = vision.internal.detector.selectStrongestValidation.isValidNumStrongestVector(N);
                else
                    isvalid = false;
                end
            else
                isvalid = false;
            end

            coder.internal.errorIf(~isvalid,...
                'vision:selectStrongestBbox:invalidNumStrongestScalarOrVector');
        end

    end

end

%--------------------------------------------------------------------------
function iValidateNumNameValueArg(varargin)
narginchk(0,6)
if mod(nargin,2) ~= 0
    narginchk(1,1); % throw not enough inputs error.
end
end

%--------------------------------------------------------------------------
function names = iValidateNames(inNames,expectedArgs)
idx = 1:numel(inNames);

for i = 1:numel(inNames)
    if ~(ischar(inNames{i}) || isstring(inNames{i}))
        error(message('vision:selectStrongestBbox:paramNameNotString'));
    end

    % Compare names with expected args. Error if name does not match
    % expected arg.
    ind = strncmpi(inNames{i},expectedArgs,strlength(inNames{i}));
    if ~any(ind)
        error(message('vision:selectStrongestBbox:invalidParamName'));
    end
    idx(i) = find(ind,1,'first');
end

names = expectedArgs(idx);

end
