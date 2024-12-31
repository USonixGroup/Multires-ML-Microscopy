classdef bagOfFeaturesValidation
% bagOfFeaturesValidation contains validation methods of bagOfFeatures
% common for both simulation and codegen
%
% Copyright 2022 The MathWorks, Inc.
%#codegen
    methods (Hidden, Static)
        %------------------------------------------------------------------
        function checkVocabularySize(vSize)

            validateattributes(vSize,{'double','single','uint32', ...
                                      'uint16', 'uint8'},{'scalar','nonempty','integer','>=',2},...
                               'bagOfFeatures', 'VocabularySize');

        end
        %------------------------------------------------------------------
        function checkTreeProperties(props)

            validateattributes(props,{'numeric'},...
                               {'vector','nonempty','integer','finite','numel',2},...
                               'bagOfFeatures', 'TreeProperties');

            % The number of levels must be >= 1.
            numLevels = props(1);
            validateattributes(numLevels,{'numeric'}, {'>=' 1}, ...
                               'bagOfFeatures', 'TreeProperties(1)');

            % The branching factor must be >= 2.
            branchingFactor = props(2);
            validateattributes(branchingFactor,{'numeric'}, {'>=' 2}, ...
                               'bagOfFeatures', 'TreeProperties(2)');

        end
        %------------------------------------------------------------------
        function checkStrongestFeatures(strongestFeatures)

            coder.internal.errorIf(~isequal(size(strongestFeatures), [1,1]), ...
                                   'vision:bagOfFeatures:expectedScalar', 'StrongestFeatures');
            validateattributes(strongestFeatures,{'numeric'}, ...
                               {'scalar','nonempty','real','positive','<=', 1}, ...
                               'bagOfFeatures', 'StrongestFeatures');

        end
        %------------------------------------------------------------------
        function checkGridStep(gridStep)

            validateattributes(gridStep,{'double','single','uint32', ...
                                         'uint16', 'uint8'},{'vector','nonempty','integer','positive'},...
                               'bagOfFeatures', 'GridStep');

            coder.internal.errorIf(numel(gridStep) > 2, 'vision:dims:twoElementVector','GridStep');

        end
        %------------------------------------------------------------------
        function checkBlockWidth(BlockWidth)

            validateattributes(BlockWidth,{'double','single','uint32', ...
                                           'uint16', 'uint8'},{'vector','nonempty','finite','>=', 32, 'real'},...
                               'bagOfFeatures', 'BlockWidth');

        end
        %------------------------------------------------------------------
        function supportsLocation = checkCustomExtractor(func)

            validateattributes(func, {'function_handle'}, {'scalar'}, ...
                               'bagOfFeatures', 'CustomExtractor');

            % get num args in/out. This errors out if func does not exist.
            numIn  = nargin(func);
            numOut = nargout(func);

            % functions may have varargin/out (i.e. anonymous functions)
            isVarargin  = (numIn  < 0);
            isVarargout = (numOut < 0);

            numIn  = abs(numIn);
            numOut = abs(numOut);

            % validate this API: [features, metric, location] = func(I)
            coder.internal.errorIf((~isVarargin && numIn ~= 1), ...
                                   'vision:bagOfFeatures:customInvalidNargin');

            coder.internal.errorIf((~isVarargout && numOut < 2), ...
                                   'vision:bagOfFeatures:customInvalidNargout');

            % check if custom extractor can output location data.
            if isVarargout || numOut == 3
                supportsLocation = true;
            else
                supportsLocation = false;
            end
        end
        %------------------------------------------------------------------
        function [numFeatures, featureLength] = checkCustomNumericFeatures(features)
        % Quick check on sizes and type of features and metric. Also
        % make sure the number of features is consistent with the
        % number of metrics.

            coder.internal.errorIf((~ismatrix(features)), ...
                                   'vision:bagOfFeatures:customInvalidFeatures');
            % features must be real floats (non-sparse).
            coder.internal.errorIf(~isreal(features) || isinteger(features) || issparse(features), ...
                                   'vision:bagOfFeatures:customFeaturesMustBeRealFloats');
            [numFeatures, featureLength] = size(features);

            coder.internal.errorIf(featureLength == 0, ...
                                   'vision:bagOfFeatures:customZeroFeatureLength');
        end
        %------------------------------------------------------------------
        function fcn = makeExtractorValidationFcn(featureType)
            if strcmp(featureType,'binaryFeatures')
                fcn = @vision.internal.bof.bagOfFeaturesValidation.checkCustomBinaryFeatures;
            else
                fcn = @vision.internal.bof.bagOfFeaturesValidation.checkCustomNumericFeatures;
            end

            % Create a function to use for validating custom feature
            % extractor outputs.
            fcn = @(x,y,varargin)vision.internal.bof.bagOfFeaturesValidation.checkCustomExtractorOutput(fcn,x,y,varargin{:});
        end
        %------------------------------------------------------------------
        function [featureLength, featureType, featureValFcn] = validateCustomExtractor(features, metrics)

        % Get feature type and create appropriate validation function.
            featureType   = class(features);
            featureValFcn = vision.internal.bof.bagOfFeaturesValidation.makeExtractorValidationFcn(featureType);

            % Validate features.
            featureLength = featureValFcn(features, metrics);

            % errors thrown by the extractor are reported directly to the
            % command window with a full stack to simplify debugging.
        end
        %------------------------------------------------------------------
        function [numFeatures, featureLength] = checkCustomBinaryFeatures(features)
        % Quick check on sizes and type of features and metric. Also
        % make sure the number of features is consistent with the
        % number of metrics.

            coder.internal.errorIf(~isa(features,'binaryFeatures'), ...
                                   'vision:bagOfFeatures:customInvalidFeatures');

            numFeatures = features.NumFeatures;
            featureLength = features.NumBits;

            coder.internal.errorIf(featureLength == 0, ...
                                   'vision:bagOfFeatures:customZeroNumBits');
        end
        %------------------------------------------------------------------
        function featureLength = checkCustomExtractorOutput(checkerFcn, features, featureMetrics, featureLocations)

            [numFeatures, featureLength] = checkerFcn(features);

            vision.internal.bof.bagOfFeaturesValidation.checkCustomExtractorMetrics(featureMetrics, numFeatures);

            if nargin > 3
                % Check location output.
                vision.internal.bof.bagOfFeaturesValidation.checkExtractorLocationOutput(featureLocations, numFeatures);
            end
        end
        %------------------------------------------------------------------
        function checkCustomExtractorMetrics(featureMetrics, numFeatures)

        % Validate feature metric output.
            numMetrics = numel(featureMetrics);
            coder.internal.errorIf(~iscolumn(featureMetrics), ...
                                   'vision:bagOfFeatures:customInvalidMetrics');

            coder.internal.errorIf(~isreal(featureMetrics) || isinteger(featureMetrics) || issparse(featureMetrics), ...
                                   'vision:bagOfFeatures:customMetricsMustBeRealFloats');

            % number of features and number of metrics must match
            coder.internal.errorIf(numFeatures ~= numMetrics, ...
                                   'vision:bagOfFeatures:customNumFeaturesNotEqNumMetrics');
        end
        %------------------------------------------------------------------
        function checkExtractorLocationOutput(featureLocations, numFeatures)
            coder.internal.errorIf(~ismatrix(featureLocations), ...
                                   'vision:bagOfFeatures:customInvalidLocations');

            [numLocations, N] = size(featureLocations);
            coder.internal.errorIf(N ~= 2, 'vision:bagOfFeatures:customInvalidLocations');

            coder.internal.errorIf(~isnumeric(featureLocations) || ~isreal(featureLocations) || issparse(featureLocations), ...
                                   'vision:bagOfFeatures:customLocationsMustBeRealNumeric');

            % cross validate number of features and locations
            coder.internal.errorIf(numFeatures ~= numLocations, ...
                                   'vision:bagOfFeatures:customNumFeaturesNotEqNumLocations');
        end
    end
end