classdef bagOfFeatures < vision.internal.EnforceScalarHandle
% Codegen implementation of bagOfFeatures
%
%#codegen

% Copyright 2022-2023 The MathWorks, Inc.

    properties(GetAccess = public, SetAccess = private)
        CustomExtractor
        NumVisualWords
        TreeProperties
        StrongestFeatures
        PointSelection
        GridStep
        BlockWidth
        Upright
    end

    properties (Hidden, GetAccess = public, SetAccess = protected, Dependent)
        VocabularySize
    end

    properties(Hidden, Access = protected)
        UsingCustomExtractor
        CustomFeatureLength
        CustomFeatureType
        ExtractorOutputsLocations
        Extractor
        ExtractorValidationFcn
    end

    properties(Hidden, Constant, Access = protected)
        ValidPointSelectionOptions = {'Grid','Detector'}
    end

    properties (Access = protected, Transient)
        VocabularyBuilder
    end

    properties (Access = protected)
        Encoder
    end

    properties (Dependent, Access = protected)
        VocabularyLevels

        VocabularyBranches
    end

    methods(Static)
        function props = matlabCodegenNontunableProperties(~)
        % Used for code generation
            props = {'UsingCustomExtractor', 'CustomFeatureType'};
        end
    end

    %----------------------------------------------------------------------
    % Set and get methods for dependent properties.
    %----------------------------------------------------------------------
    methods
        function v = get.VocabularySize(this)
            v = this.NumVisualWords;
        end

        function set.VocabularySize(this, v)
            this.NumVisualWords = v;
        end

        function v = get.VocabularyLevels(this)
            v = this.TreeProperties(1);
        end

        function v = get.VocabularyBranches(this)
            v = this.TreeProperties(2);
        end

        function set.VocabularyBranches(this, v)
            this.VocabularyBuilder.BranchingFactor = v;
        end
    end

    %----------------------------------------------------------------------
    methods (Access = public)

        %------------------------------------------------------------------
        % Constructor
        function this = bagOfFeatures(varargin)
            if nargin == 0

                % Just return empty object with all default configuration
                % settings. This required to serialize/deserialize objects
                % for parallel processing and testing.
                d = vision.internal.codegen.bof.bagOfFeatures.getDefaultSettings;
                this.setParams(d, d.UsingCustomExtractor, d.CustomFeatureType);
                this.NumVisualWords = 0; % this is actually an empty bag 

            elseif nargin==2 && strcmp(varargin{2}, "internal")
                return;
            else                
                % parse the inputs
                [imgStruct, usingCustomExtractor, featureType, params] = vision.internal.codegen.bof.bagOfFeatures.parseInputsCodegen(varargin{:});

                this.setParams(params, usingCustomExtractor, featureType);

                initializeVocabularyBuilder(this, params);

                [descriptors, scores] = this.extractDescriptors(imgStruct, params);

                % If needed, remove some of the descriptors
                descriptorSet = this.trimDescriptors(descriptors, scores);
                % Create the vocabulary
                vocabulary = createVocabulary(this, descriptorSet);
                % Initialize encoder
                initializeEncoder(this, vocabulary);
            end

        end % end of Constructor

        %------------------------------------------------------------------
        function [featureVector, varargout] = encode(this, in, varargin)

            nargoutchk(0,2);

            % Check if extractor supports visual word output
            coder.internal.errorIf(nargout == 2 && ~this.ExtractorOutputsLocations, ...
                                   'vision:bagOfFeatures:customNoLocations',...
                                   func2str(this.CustomExtractor));

            [params, sparseOut] = vision.internal.codegen.bof.bagOfFeatures.parseEncodeInputs(in, varargin{:});

            % Configure Encoder properties.
            this.Encoder.Normalization = params.Normalization;
            this.Encoder.SparseOutput  = sparseOut;

            numVarargout = nargout-1;

            if ~isnumeric(in)
                [featureVector, varargout{1:numVarargout}] = this.encodeImagesCell(in, sparseOut);
            else
                [featureVector, varargout{1:numVarargout}] = this.encodeSingleImage(in, sparseOut);
            end
        end

        %------------------------------------------------------------------
        function s = saveobj(this)
        % save properties into struct
            s.NumVisualWords            = this.NumVisualWords;
            s.TreeProperties            = this.TreeProperties;
            s.StrongestFeatures         = this.StrongestFeatures;
            s.GridStep                  = this.GridStep;
            s.BlockWidth                = this.BlockWidth;
            s.PointSelection            = this.PointSelection;
            s.CustomExtractor           = this.CustomExtractor;
            s.CustomFeatureType         = this.CustomFeatureType;
            s.UsingCustomExtractor      = this.UsingCustomExtractor;
            s.CustomFeatureLength       = this.CustomFeatureLength;
            s.Upright                   = this.Upright;
            s.ExtractorOutputsLocations = this.ExtractorOutputsLocations;
            s.Encoder                   = saveobj(this.Encoder);
            s.Version                   = 1;
        end
    end

    %======================================================================
    methods (Hidden, Static)
        % -----------------------------------------------------------------
        function this = loadobj(structBag)

            coder.internal.prefer_const(structBag);
            this = bagOfFeatures(structBag, "internal");

            hasCustomExtarctor = isfield(structBag, 'CustomExtractor');
            hasUpright = isfield(structBag, 'Upright');
            hasExtractOp = isfield(structBag, 'ExtractorOutputsLocations');
            if hasCustomExtarctor && hasUpright && hasExtractOp
                structBagNew = structBag;
                usingCustomExtractor = (structBag.UsingCustomExtractor & 1);
                if isa(structBag.CustomFeatureType, 'single')
                    customFeatureType = coder.internal.const('single');
                elseif isa(structBag.CustomFeatureType, 'double')
                    customFeatureType = coder.internal.const('double');
                else
                    customFeatureType = coder.internal.const('binaryFeatures');
                end
            else
                % set to defaults if loading old version of object

                defaults = bagOfFeatures.getDefaultSettings();
                structBagNew = structBag;

                structBagNew.UsingCustomExtractor      = defaults.UsingCustomExtractor;
                structBagNew.CustomExtractor           = defaults.CustomExtractor;
                structBagNew.CustomFeatureLength       = defaults.CustomFeatureLength;
                structBagNew.CustomFeatureType         = defaults.CustomFeatureType;
                structBagNew.Upright                   = defaults.Upright;
                structBagNew.ExtractorOutputsLocations = defaults.ExtractorOutputsLocations;
                usingCustomExtractor = coder.internal.const(defaults.UsingCustomExtractor & 1);
                customFeatureType = coder.internal.const(defaults.CustomFeatureType);
            end

            % Manage encoder creation for BoF saved in previous releases.
            objectIsSavedBeforeR2021b = ~isfield(structBagNew,'Version');
            if objectIsSavedBeforeR2021b

                % Add feature type information. Prior to 21b, only numeric
                % features were supported.
                structBagNew.CustomFeatureType = 'double';

                % Add vocabulary tree properties. Prior to 21b, only flat
                % hierarchies were supported.
                structBagNew.TreeProperties = [1 structBagNew.VocabularySize];
                structBagNew.NumVisualWords = structBagNew.VocabularySize;

                % Prior to 21b, the encoder was part of bagOfFeatures.
                % As of 21b, the encoder is encapsulated within an object
                % inside of bagOfFeatures using composition. Create the
                % encoder object using information saved from prior
                % releases.

                objectSavedPriorToR2015b = ~isfield(structBagNew,'KDTreeIndexState');
                if objectSavedPriorToR2015b
                    % Loading object prior to R2015b. Vocabulary must be
                    % transposed into row-major format.
                    vocabulary = structBagNew.Vocabulary';

                    % Prior to 15b the random state was not serialized. Use
                    % the current state.
                    randState = rng;
                else
                    vocabulary = structBagNew.Vocabulary;
                    randState  = structBagNew.KDTreeIndexState;
                end

                % Create the encoder.
                encoderFcn = @(x,y)vision.internal.bof.EncoderNumericFeaturesKDTree(x,y,0);
                vocabTree = vision.internal.bof.VocabularyTree();
                vocabTree.Words = vocabulary;
                this.Encoder = vision.internal.bof.EncoderVocabularyTree(vocabTree, encoderFcn);
            else
                % Load encoder if not empty. The Encoder may be empty when
                % the bagOfFeatures is getting created using parallel
                % workers.
                if ~isempty(structBagNew.Encoder)
                    % Load object using loadobj().
                    this.Encoder = vision.internal.bof.EncoderVocabularyTree.loadobj(structBagNew.Encoder);
                end
            end

            % Assign saved property values.
            this.setParams(structBagNew, usingCustomExtractor, customFeatureType);
        end

    end

    %======================================================================
    methods (Hidden, Access = {?invertedImageIndex})
        %------------------------------------------------------------------
        % Compute a histogram of occurences for given image features from
        % a single image. It turns the input into a feature vector
        %------------------------------------------------------------------
        function featureVector = encodeFeatures(this, featuresIn)

            vision.internal.inputValidation.checkFeatures(featuresIn, mfilename, 'features');

            % Configure Encoder properties.
            this.Encoder.Normalization = 'None';
            this.Encoder.SparseOutput  = true;

            % check if input feature type matches the one cached in BoF
            if isa(featuresIn, 'binaryFeatures')
                coder.internal.errorIf(~strcmp(this.CustomFeatureType,'binaryFeatures'), ...
                    'vision:bagOfFeatures:encodeInvalidBinaryFeatureType');

                coder.internal.errorIf(this.CustomFeatureLength ~= featuresIn.NumBits, ...
                    'vision:bagOfFeatures:encodeInvalidFeatureLength', ...
                        this.CustomFeatureLength, featuresIn.NumBits);
                features = featuresIn.Features;
            else
                if this.UsingCustomExtractor
                    coder.internal.errorIf(strcmp(this.CustomFeatureType,'binaryFeatures'), ...
                        'vision:bagOfFeatures:encodeInvalidNumericFeatureType');

                    featureLength = size(featuresIn,2);
                    coder.internal.errorIf(this.CustomFeatureLength ~= featureLength, ...
                        'vision:bagOfFeatures:encodeInvalidFeatureLength', ...
                        this.CustomFeatureLength, featureLength);
                end
                features = featuresIn;
            end

            fVector = this.Encoder.encode(features);
            % Sparse Output is always true
            featureVector = sparse(double(fVector));

        end
    end
    %======================================================================
    methods (Hidden, Static, Access = protected)
        %------------------------------------------------------------------
        % Returns default object settings
        %------------------------------------------------------------------
        function d = getDefaultSettings
            d.NumVisualWords            = 500;
            d.TreeProperties            = [1 500];
            d.StrongestFeatures         = 0.8;
            d.PointSelection            = 'Grid';
            d.GridStep                  = [8 8];
            d.BlockWidth                = [32 64 96 128];
            d.Verbose                   = true;
            d.CustomExtractor           = @vision.internal.codegen.bof.bagOfFeatures.makeUninitialized; % no-op function
            d.UsingCustomExtractor      = false;
            d.CustomFeatureLength       = 0;
            d.CustomFeatureType         = 'double';
            d.Upright                   = true;
            d.ExtractorOutputsLocations = true;
            d.UseParallel               = false;
        end

        %------------------------------------------------------------------
        function checkImageStruct(imgStruct)
            varName = 'imgStruct';

            validateattributes(imgStruct,...
                               {'struct'}, ...
                               {'nonempty'},...
                               varName);
            structFields = fieldnames(imgStruct);
            requiredStructFields = 'Images';
            optionalStructFields = 'Labels';
            hasLabelField = 0;
            % Add error Id for field missing
            coder.internal.assert(isequal(structFields{1}, requiredStructFields), ...
                                   'vision:bagOfFeatures:invalidFields', varName);
            if numel(structFields) == 2
                coder.internal.assert(isequal(structFields{2}, optionalStructFields), ...
                    'vision:bagOfFeatures:invalidFields', varName);
                hasLabelField = 1;
            else
                coder.internal.errorIf(numel(structFields) > 2, ...
                    'vision:bagOfFeatures:invalidFields', varName);
            end
            images = imgStruct.Images;
            validateattributes(images, {'cell'}, {'vector'}, 'bagOfFeatures', 'Images');
            % labels and images should be of same size
            if hasLabelField
                labels = imgStruct.Labels;
                validateattributes(labels, {'categorical'}, {'vector'}, 'bagOfFeatures', 'Labels');
            end
            for i=1:numel(images)
                vision.internal.inputValidation.validateImage(images{i},'images');
            end
        end

        %--------------------------------------------------------------------------
        function [imgStruct, usingCustomExtractor, featureType, params] = parseInputsCodegen(varargin)
            defaults = vision.internal.codegen.bof.bagOfFeatures.getDefaultSettings;
            imgStruct = varargin{1};
            vision.internal.codegen.bof.bagOfFeatures.checkImageStruct(imgStruct);

            % Define parser mapping struct
            pvPairs = struct( ...
                'VocabularySize',      uint32(0), ...
                'TreeProperties',      uint32(0), ...
                'StrongestFeatures',   uint32(0), ...
                'PointSelection',      uint32(0),...
                'GridStep',            uint32(0),...
                'BlockWidth',          uint32(0), ...
                'CustomExtractor',     uint32(0), ...
                'Upright',             uint32(0));

            % Specify parser options
            poptions = struct( ...
                'CaseSensitivity', false, ...
                'StructExpand',    true, ...
                'PartialMatching', true);

            % Parse PV pairs
            pstruct = coder.internal.parseParameterInputs(pvPairs, ...
                                                          poptions, varargin{2:end});
            % Extract inputs.GridStep
            vocabularySize     = coder.internal.getParameterValue(pstruct.VocabularySize, 500, varargin{2:end});
            treeProperties     = coder.internal.getParameterValue(pstruct.TreeProperties, [1 500], varargin{2:end});
            strongestFeatures  = coder.internal.getParameterValue(pstruct.StrongestFeatures, defaults.StrongestFeatures, varargin{2:end});
            pointSelection     = coder.internal.getParameterValue(pstruct.PointSelection, defaults.PointSelection, varargin{2:end});
            gridStep           = coder.internal.getParameterValue(pstruct.GridStep, defaults.GridStep, varargin{2:end});
            blockWidth         = coder.internal.getParameterValue(pstruct.BlockWidth, defaults.BlockWidth, varargin{2:end});
            customExtractor    = coder.internal.getParameterValue(pstruct.CustomExtractor, defaults.CustomExtractor, varargin{2:end});
            upright            = coder.internal.getParameterValue(pstruct.Upright, defaults.Upright, varargin{2:end});

            vision.internal.bof.bagOfFeaturesValidation.checkVocabularySize(vocabularySize);
            vision.internal.bof.bagOfFeaturesValidation.checkTreeProperties(treeProperties);
            vision.internal.bof.bagOfFeaturesValidation.checkStrongestFeatures(strongestFeatures);
            vision.internal.bof.bagOfFeaturesValidation.checkGridStep(gridStep);
            vision.internal.bof.bagOfFeaturesValidation.checkBlockWidth(blockWidth);
            vision.internal.inputValidation.validateLogical(upright,'Upright');

            params.ExtractorOutputsLocations = vision.internal.bof.bagOfFeaturesValidation.checkCustomExtractor(customExtractor);

            % Set property values to the correct type and format.
            params.VocabularySize    = double(vocabularySize);
            params.TreeProperties    = double(treeProperties);
            params.StrongestFeatures = double(strongestFeatures);
            % Handle partial strings
            [~,params.PointSelection]= vision.internal.codegen.bof.bagOfFeatures.checkAndAssignPointSelection(pointSelection);
            gridStep          = double(gridStep(:)');
            params.BlockWidth        = double(blockWidth(:)');
            params.CustomExtractor   = customExtractor;
            params.Upright           = logical(upright);
            % Set value of NumVisualWords based on TreeProperties. The
            % value set here may change based on the outcome of the
            % clustering step in createVocabulary.
            params.NumVisualWords = double(treeProperties(2)^treeProperties(1));
            params.Verbose = false;
            params.UseParallel = false;

            if vision.internal.codegen.bof.bagOfFeatures.wasSpecified(varargin,'VocabularySize')
                wasTreePropertiesSpecified = vision.internal.codegen.bof.bagOfFeatures.wasSpecified(varargin, 'TreeProperties');
                % VocabularySize and TreeProperties cannot be
                % specified together.
                coder.internal.errorIf(wasTreePropertiesSpecified, 'vision:bagOfFeatures:VocabularySizeAndHierarchyIncompatible');
                % Update TreeProperties to use one level with a
                % branching factor equal to VocabularySize.
                params.TreeProperties = [1 double(vocabularySize);];
            end

            if isequal(customExtractor, defaults.CustomExtractor)
                usingCustomExtractor = false;
            else
                usingCustomExtractor = true;
            end
            if usingCustomExtractor
                [features, metrics] = vision.internal.codegen.bof.bagOfFeatures.previewCustomExtractorOutput(imgStruct, customExtractor);

                featureType = class(features);
                [featureLength, ~] = vision.internal.bof.bagOfFeaturesValidation.validateCustomExtractor(features, metrics);

                % Cache the feature length for size checks while invoking
                % the custom extractor later on.
                params.CustomFeatureLength    = featureLength;
            else
                params.CustomFeatureLength    = 0;
                featureType = defaults.CustomFeatureType;
            end

            wasPointSelectionSpecified = vision.internal.codegen.bof.bagOfFeatures.wasSpecified(varargin,'PointSelection');
            wasGridStepSpecified       = vision.internal.codegen.bof.bagOfFeatures.wasSpecified(varargin,'GridStep');
            wasBlockWidthSpecified     = vision.internal.codegen.bof.bagOfFeatures.wasSpecified(varargin,'BlockWidth');
            wasUprightSpecified        = vision.internal.codegen.bof.bagOfFeatures.wasSpecified(varargin,'Upright');

            if usingCustomExtractor
                if wasPointSelectionSpecified || wasGridStepSpecified ...
                        || wasBlockWidthSpecified || wasUprightSpecified
                    coder.internal.warning('vision:bagOfFeatures:customInvalidParamCombo');
                end
            end

            % Scalar expand grid step
            if isscalar(gridStep)
                params.GridStep = [gridStep, gridStep];
            else
                params.GridStep = gridStep;
            end

            % Warn about ignored options
            if strcmp(params.PointSelection, 'Detector')
                if wasGridStepSpecified || wasBlockWidthSpecified
                    coder.internal.warning('vision:bagOfFeatures:paramsIgnored');
                end
            end

        end

        %------------------------------------------------------------------
        function [tf, ps] = checkAndAssignPointSelection(pointSelection)

            ps = validatestring(pointSelection,...
                                vision.internal.codegen.bof.bagOfFeatures.ValidPointSelectionOptions, mfilename, 'PointSelection');

            tf = true;
        end

        %------------------------------------------------------------------
        function tf = wasSpecified(userdata, name )
            tf = any(strcmp(userdata,name));
        end

        %------------------------------------------------------------------
        function [features, metrics] = previewCustomExtractorOutput(imgSet, extractor)
        % try the custom extractor on 1 image.
            img = imgSet.Images{1};

            [features, metrics] = extractor(img);
        end

        %------------------------------------------------------------------
        function validateEncodeInput(in)
            if isnumeric(in)
                if ~isempty(in) % allow empty inputs
                    vision.internal.inputValidation.validateImage(in,'I');
                end
            else
                validateattributes(in,{'cell'},{'vector', 'nonempty'},mfilename);
                for i=1:numel(in)
                    if ~isempty(in{i}) % allow empty inputs
                        vision.internal.inputValidation.validateImage(in{i},'I');
                    end
                end
            end
        end

        %------------------------------------------------------------------
        function [params, sparseOut] = parseEncodeInputs(varargin)

            imCell = varargin{1};
            % Define parser mapping struct
            pvPairs = struct( ...
                'Normalization',      uint32(0), ...
                'SparseOutput',      uint32(0));

            % Specify parser options
            poptions = struct( ...
                'CaseSensitivity', false, ...
                'StructExpand',    true, ...
                'PartialMatching', true);

            % Parse PV pairs
            pstruct = coder.internal.parseParameterInputs(pvPairs, ...
                                                          poptions, varargin{2:end});
            normalization     = coder.internal.getParameterValue(pstruct.Normalization, 'L2', varargin{2:end});
            sparseOutput     = coder.internal.getParameterValue(pstruct.SparseOutput, false, varargin{2:end});

            vision.internal.codegen.bof.bagOfFeatures.validateEncodeInput(imCell);
            vision.internal.inputValidation.validateLogical(sparseOutput, 'SparseOutput');

            % validate params
            str = validatestring(normalization,{'none','L2'},mfilename);

            sparseOut = coder.internal.const(sparseOutput & 1);
            
            % assign user data
            params.Normalization = str;
            params.Verbose       = false;
            params.UseParallel   = false;
        end

        %------------------------------------------------------------------
        function featureVector = allocateFeatureVector(m,n,isSparse)

        % Determine type for allocation
            if isSparse
                prototype = sparse(0);
            else
                prototype = single(0);
            end

            featureVector = zeros(m,n,'like',prototype);
        end

        %------------------------------------------------------------------
        function words = allocateVisualWords(n)
            wordIndex = zeros(0, 1);
            assignments = zeros(0, 2, 'single');
            vocabSize = 1;
            if n > 0
                words = vision.internal.visualWords(wordIndex, assignments, vocabSize);
                for i=2:n
                    words(i) = vision.internal.visualWords(wordIndex, assignments, vocabSize);
                end
            else
                words = vision.internal.visualWords.empty(wordIndex, assignments, vocabSize);
            end
        end

        %------------------------------------------------------------------
        function varargout = makeUninitialized(this)
        % Dummy function_handle for customExtractor function
            varargout{1} = 1;
        end
    end

    %======================================================================
    methods (Hidden, Access = protected)
        %------------------------------------------------------------------
        function setParams(this, params, usingCustomExtractor, featureType)

            this.NumVisualWords      = params.NumVisualWords;
            this.TreeProperties      = params.TreeProperties;
            this.StrongestFeatures   = params.StrongestFeatures;
            this.GridStep            = params.GridStep;
            this.BlockWidth          = params.BlockWidth;
            this.PointSelection      = params.PointSelection;
            this.Upright             = params.Upright;

            this.UsingCustomExtractor      = usingCustomExtractor;
            this.CustomFeatureType   = featureType;
            this.CustomExtractor           = params.CustomExtractor;
            this.ExtractorOutputsLocations = params.ExtractorOutputsLocations;

            if this.UsingCustomExtractor
                % Use the custom extractor.
                this.CustomFeatureLength = params.CustomFeatureLength;

                % Create validation function based on feature type.
                this.ExtractorValidationFcn = vision.internal.bof.bagOfFeaturesValidation.makeExtractorValidationFcn(this.CustomFeatureType);

                % Function handles with same workspace as the object cannot be returned
                % Use different classes to denote different extractors
                if strcmp(this.CustomFeatureType,'binaryFeatures')
                    this.Extractor = double(0);
                else
                    this.Extractor = single(0);
                end
            else
                % Use the default SURF extractor
                this.Extractor = uint8(0);
            end
        end

        %------------------------------------------------------------------
        % Invoke the custom extractor.
        %------------------------------------------------------------------
        function [features, featureMetrics, varargout] = invokeCustomExtractor(this, img, extractor)

            [initialFeatures, initialFeatureMetrics, varargout{1:nargout-2}] = extractor(img);

            featureLength = this.ExtractorValidationFcn(initialFeatures, initialFeatureMetrics, varargout{:});

            % Error if the feature length changes from the one we have
            % cached.
            coder.internal.errorIf(this.CustomFeatureLength ~= featureLength, ...
                                   'vision:bagOfFeatures:customInvalidFeatureLength', ...
                                   this.CustomFeatureLength, featureLength);

            % cast custom features and metric to single for clustering.
            if ~strcmp(this.CustomFeatureType,'binaryFeatures')
                features = single(initialFeatures);
            else
                features = initialFeatures;
            end

            featureMetrics = single(initialFeatureMetrics);

            if nargout > 2
                % cast location data to single
                varargout{1} = single(varargout{1});
            end
        end

        %------------------------------------------------------------------
        % Invoke custom extractor and return underlying binary features as
        % a numeric matrix. This simplifies collecting features across
        % images.
        %------------------------------------------------------------------
        function  [features, featureMetrics, varargout] = ...
                invokeAndExtractUnderlyingBinaryFeatures(this, img, extractor)
            [features, featureMetrics, varargout{1:nargout-2}] = this.invokeCustomExtractor(img,extractor);
            features = features.Features;
        end

        %------------------------------------------------------------------
        function initializeVocabularyBuilder(this, params)

        % Choose the a vocabulary build and encoder based on the feature type.
            if strcmp(this.CustomFeatureType,'binaryFeatures')
                vocabBuilder = vision.internal.bof.VocabularyBuilderKMedians;
                createEncoderFcn = @vision.internal.bof.EncoderBinaryFeatures;
            else
                vocabBuilder = vision.internal.bof.VocabularyBuilderApproximateKMeans;
                createEncoderFcn = @(x,y)vision.internal.bof.EncoderNumericFeaturesKDTree(x,y,0);
            end

            % Set parameters for vocabulary creation approximate
            % K-means. Literal parameter values are tuned for robust
            % results across a variety of data sets.
            vocabBuilder.NumTrials     = 1;
            vocabBuilder.MaxIterations = 100;
            vocabBuilder.Threshold     = 0.0001;
            vocabBuilder.UseParallel   = params.UseParallel;

            % Create a hierarchical vocabulary builder.
            this.VocabularyBuilder = vision.internal.bof.VocabularyBuilderHierarchicalClustering(...
                vocabBuilder, createEncoderFcn, ...
                params.TreeProperties(1), ... % number of levels
                params.TreeProperties(2), ...
                params.Verbose);
        end

        %------------------------------------------------------------------
        function [descriptors, scores] = extractDescriptors(this, imgStruct, params)

            [descriptors, scores] = this.extractDescriptorsFromStruct(imgStruct, params);
        end

        %------------------------------------------------------------------
        function [groupedDescriptors, groupedScores] = extractDescriptorsFromStruct(this, imgStruct, params)
            numImages   = numel(imgStruct.Images);

            categoryIndices = zeros(1, numImages);
            if ~isfield(imgStruct, 'Labels') || isempty(imgStruct.Labels)
                categoryIndices = repelem(1,numImages);
            else
                % Use categorical to get category indices for grouping
                % descriptors and scores.
                c = categorical(imgStruct.Labels);

                % check if any of the labels were left undefined, which can
                % happen if files are added incrementally to the image
                % datastore.
                undefined = isundefined(c);
                if all(undefined)
                    % manually set category index
                    categoryIndices = repelem(1,numImages);

                elseif any(undefined)
                    % Disallow partially undefined labels
                    coder.internal.errorIf(any(undefined), 'vision:bagOfFeatures:labelsUndefined')

                else
                    categoryIndices = double(c);
                end

            end
            descriptors = coder.nullcopy(cell(numImages, 1));
            scores      = coder.nullcopy(cell(numImages, 1));
            extractor = this.getExtractorFunc(this.Extractor);
            for i = 1:numImages
                img = imgStruct.Images{i};

                [tempDescriptors, tempScores] = ...
                    extractor(img);
                descriptors{i} = tempDescriptors;
                scores{i} = tempScores;
            end
            if ~isfield(imgStruct, 'Labels') || isempty(imgStruct.Labels)
                numCategories = 1;
            else
                numCategories      = numel(unique(imgStruct.Labels));
            end
            groupedDescriptors = cell(numCategories,1);
            groupedScores      = cell(numCategories,1);

            for i = 1:numCategories
                groupedDescriptors{i} = zeros(coder.ignoreConst(0), coder.ignoreConst(0), class(descriptors{i}));
                groupedScores{i} = zeros(coder.ignoreConst(0), coder.ignoreConst(0), class(scores{i}));
            end
            % Group descriptors by labels.
            for i = 1:numImages
                groupedDescriptors{categoryIndices(1, i)} = [groupedDescriptors{categoryIndices(1, i)}; descriptors{i}];
                groupedScores{categoryIndices(1, i)} = [groupedScores{categoryIndices(1, i)}; scores{i}];
            end
        end

        %------------------------------------------------------------------
        % Return the image descriptors and their scores.
        %
        % When the PointSelection method is 'Grid', the scores are computed
        % using the variance of the SURF descriptors. For more details,
        % refer to figure 3 in:
        %
        %   Herbert Bay, Andreas Ess, Tinne Tuytelaars, Luc Van Gool "SURF:
        %   Speeded Up Robust Features", Computer Vision and Image
        %   Understanding (CVIU), Vol. 110, No. 3, pp. 346-359, 2008
        %
        %------------------------------------------------------------------
        function [descriptors, scores, locations] = extractDescriptorsFromImage(this, image)

            if ismatrix(image) % convert color images to grayscale
                grayImage = image;
            else
                grayImage = rgb2gray(image);
            end

            points = this.determineExtractionPoints(grayImage);

            % Use upright SURF features.
            descriptors = extractFeatures(grayImage, points, 'Upright', this.Upright);

            if strcmpi(this.PointSelection,'Grid')
                if isempty(descriptors)
                    scores = zeros(0, 1, 'single');
                else
                    scores = var(descriptors,[],2);
                end
            else
                scores = points.Metric;
            end

            locations = points.Location;
        end

        %------------------------------------------------------------------
        function points = determineExtractionPoints(this,grayImage)

            if strcmpi(this.PointSelection, 'Grid')
                points = this.createSURFPointsGrid(grayImage);
            else
                points = detectSURFFeatures(grayImage, 'MetricThreshold', 800);
            end
        end

        %------------------------------------------------------------------
        function points = createSURFPointsGrid(this, grayImage)

            startIdx = round(this.GridStep/2);
            stopIdx  = fliplr(size(grayImage));

            rangeX = startIdx(1):this.GridStep(1):stopIdx(1);
            rangeY = startIdx(2):this.GridStep(2):stopIdx(2);

            if isempty(rangeX) || isempty(rangeY)

                coder.internal.warning('vision:bagOfFeatures:noGridPoints');
                loc = zeros(coder.ignoreConst(0),coder.ignoreConst(2));
                points = SURFPoints(loc);
            else

                [X, Y] = meshgrid(rangeX, rangeY);

                % SURFPoint scale values, specified as multiples of 1.6
                scales = single(1.6 * this.BlockWidth/32);

                % create SURFPoints object for the grid
                locations = repmat([X(:) Y(:)], numel(scales),1);
                scales    = repmat(scales, numel(X),1);

                points = SURFPoints(locations, 'Scale', scales(:));

            end
        end

        %------------------------------------------------------------------
        function trimmedDescriptors = trimDescriptors(this, ...
                                                      descriptorCell, scores)

            numScores = size(scores, 1);
            numFeatures = zeros(1, numScores);
            for i = 1:numScores
                numFeatures(i) = size(scores{i},1);
            end
            % Determine smallest number of features in any of the sets.
            % That will determine the maximum number of features to use
            % while forming the vocabulary
            [maxNumFeatures, ~] = min(numFeatures);

            % Based on percentage, figure out how many features to keep
            numToKeep = round(maxNumFeatures*this.StrongestFeatures);

            numSets = length(descriptorCell);
            trimmedDescriptors = zeros(numSets*numToKeep, ...
                                       size(descriptorCell{1}, 2), 'like', descriptorCell{1});
            insertIdx = 1:numToKeep;
            for i=1:numSets
                setScores      = scores{i};
                setDescriptors = descriptorCell{i};

                % limit the features to only the strong ones
                [~, sortIdx] = sort(setScores ,'descend');

                topScoresIdx = sortIdx(1:numToKeep,:);

                setDescriptors = setDescriptors(topScoresIdx,:);
                trimmedDescriptors(insertIdx,:) = setDescriptors;
                insertIdx = insertIdx+numToKeep;
            end
        end

        %------------------------------------------------------------------
        function vocabulary = createVocabulary(this, descriptors)

            numDescriptors = size(descriptors, 1);

            desiredVocabularySize = this.VocabularyBranches^this.VocabularyLevels;

            if this.VocabularyLevels == 1
                % Check whether we have enough data to build the desired
                % vocabulary. For a vocabulary hierarchy with one level, we
                % can adjust the user's desired vocabulary size to one that
                % fits the data.
                K = min(numDescriptors, desiredVocabularySize);

                coder.internal.errorIf(K == 0, 'vision:bagOfFeatures:notEnoughFeatures',...
                                       string(desiredVocabularySize));

                if numDescriptors < desiredVocabularySize
                    coder.internal.warning('vision:bagOfFeatures:reducingVocabSize', ...
                                           K, string(desiredVocabularySize));

                    % Update the branching factor in the TreeProperties.
                    this.VocabularyBuilder.BranchingFactor = K;

                end

            else % More than one level.


                % Not enough data to build the a vocabulary of the
                % desired size. For a hierarchical tree, the number of
                % levels or the branching factor needs to be reduced.
                % Issue an error to let the user decide.
                coder.internal.errorIf(numDescriptors < desiredVocabularySize, ...
                                       'vision:bagOfFeatures:notEnoughFeatures',...
                                       string(desiredVocabularySize));

            end

            % Build the vocabulary.
            vocabulary = this.VocabularyBuilder.create(descriptors);

            % Update the vocabulary size based on what we able to learn
            % from the data.
            this.NumVisualWords = vocabulary.maxWords;

            if this.VocabularyLevels > 1 && this.NumVisualWords < desiredVocabularySize
                coder.internal.warning('vision:bagOfFeatures:reducingVocabTreeSize',...
                                       string(this.NumVisualWords));
            end

            % Update the number of vocabulary levels and branching factor.
            % Given the potential to have unbalanced trees, the hierarchy
            % is reported using maximum values.
            this.TreeProperties = [vocabulary.numLevels vocabulary.maxBranchingFactor];
        end

        %------------------------------------------------------------------
        function initializeEncoder(this, vocabulary)
            isBinaryFeature = strcmp(this.CustomFeatureType,'binaryFeatures');
            if isBinaryFeature
                encoderFcn = @(x,y)vision.internal.bof.EncoderBinaryFeatures(x,y);
            else
                encoderFcn = @(x,y)vision.internal.bof.EncoderNumericFeaturesKDTree(x,y,0);
            end
            this.Encoder = vision.internal.codegen.bof.EncoderVocabularyTree(...
                vocabulary, encoderFcn);
        end

        %------------------------------------------------------------------
        function [features, varargout] = encodeImagesCell(this, imgCell, sparseOut)

            numImages = numel(imgCell);

            features = vision.internal.codegen.bof.bagOfFeatures.allocateFeatureVector(numImages, this.NumVisualWords, sparseOut);

            numVarargout = nargout-1;
            % Get the extractor
            if numVarargout == 1
                words    = vision.internal.codegen.bof.bagOfFeatures.allocateVisualWords(numImages);
                for j = 1:numImages
                    img = imgCell{j};
                    [features(j,:), words(j)]  = this.encodeSingleImage(img, sparseOut);
                end

                varargout{1} = words;
            else
                for j = 1:numImages
                    img = imgCell{j};
                    features(j,:)  = this.encodeSingleImage(img, sparseOut);
                end
            end

        end

        %------------------------------------------------------------------
        function [featureVector, varargout] = encodeSingleImage(this, img, sparseOut)

        % Get the extractor
            extractor = this.getExtractorFunc(this.Extractor);
            if nargout == 2
                [features,~,locations] = extractor(img);
                [fVector, varargout{1}] = this.Encoder.encode(features, locations);
            else
                features = extractor(img);
                [fVector] = this.Encoder.encode(features);
            end
            if sparseOut
                featureVector = sparse(double(fVector));
            else
                featureVector = fVector;
            end
        end

        %------------------------------------------------------------------
        function extractorOut = getExtractorFunc(this, extractor)
            if isa(extractor, 'double')
                extractorOut = @(img)this.invokeAndExtractUnderlyingBinaryFeatures(img, this.CustomExtractor);
            elseif isa(extractor, 'single')
                extractorOut = @(img)this.invokeCustomExtractor(img, this.CustomExtractor);
            else
                extractorOut = @(img)this.extractDescriptorsFromImage(img);
            end
        end
    end
end
