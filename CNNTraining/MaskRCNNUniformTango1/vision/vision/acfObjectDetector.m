classdef acfObjectDetector < vision.internal.EnforceScalarHandle
    %

    % Copyright 2016-2024 The MathWorks, Inc.

    %#codegen
    %#ok<*EMCLS>
    %#ok<*EMCA>

    properties(Access = public)
        ModelName
    end

    properties(SetAccess = private, Dependent)
        ObjectTrainingSize
        NumWeakLearners
    end

    properties(Access = protected)
        Classifier
        TrainingOptions
        TrainingOptionsNonTunable
    end

    %======================================================================
    methods
        function [bboxes, scores ] = detect(this, I, varargin)
            useSharedLibrary = coder.internal.preferMATLABHostCompiledLibraries();

            if iIsDatastore(I) && isempty(coder.target)
                nargoutchk(0,1);

                % Copy and reset datastore to prevent modification of
                % datastore handle object.
                ds = copy(I);
                reset(ds);

                params = acfObjectDetector.parseDetectInputs(ds, ...
                    this.TrainingOptions.ModelSize, varargin{:});

                this = updateTrainingOptions(this, params);

                % Define a transform on the datastore to run the detector on
                % each image in the datastore.
                tds = transform(ds, @(data,info)detectionTransform(this, data, info, params),...
                    'IncludeInfo', true);

                % Read all the detection results.
                try
                    data = readall(tds);
                catch ME
                    % Strip away the transform related error and throw the
                    % underlying cause as the use of a transform is an
                    % implementation detail.
                    throw(ME.cause{1});
                end
                % Pack results into a table.
                bboxes = table(data(:,1), data(:,2), ...
                    'VariableNames', {'Boxes','Scores'});

            else
                if isempty(coder.target) || useSharedLibrary  % simulation or host codegen path

                    if isempty(coder.target)
                        params = acfObjectDetector.parseDetectInputs(I, ...
                            this.TrainingOptions.ModelSize, varargin{:});
                    else
                        params = acfObjectDetector.parseDetectInputs_cg(I, ...
                            this.TrainingOptions.ModelSize, varargin{:});
                    end

                    this = updateTrainingOptions(this, params);

                    Iroi = vision.internal.detector.cropImageIfRequested(I, ...
                        params.ROI, params.UseROI);

                    [bboxes, scores] = computePyramidAndDetect(this, Iroi);

                else  % portable codegen path

                    [params,ROI,UseROI,threshold] = acfObjectDetector.parseDetectInputsCGPortable(I, this.TrainingOptions.ModelSize, varargin{:});
                    Iroi = vision.internal.detector.cropImageIfRequested(I, ROI, UseROI);

                    this.TrainingOptions.MinSize         = params.MinSize;
                    this.TrainingOptions.MaxSize         = params.MaxSize;
                    this.TrainingOptions.WindowStride    = params.WindowStride;
                    this.TrainingOptions.NumScaleLevels  = params.NumScaleLevels;

                    coder.internal.prefer_const(threshold);

                    [bboxes, scores] = vision.internal.codegen.acf.computePyramidAndDetect(Iroi, this.TrainingOptions, this.TrainingOptionsNonTunable, this.Classifier, threshold);
                end

                [bboxes, scores] = postprocessDetections(this, Iroi, bboxes, scores, params);
            end
        end
    end

    %======================================================================
    % Constructor
    %======================================================================
    methods (Hidden)
        function this = acfObjectDetector(classifier, params)
            this.checkClassifierInput(classifier);
            this.checkParametersInput(params);

            this.ModelName = params.ModelName;
            this.Classifier = classifier;
            this.TrainingOptions = params;

            % construct non-tunable training options for portable codegen
            % to generate faster code

            useSharedLibrary = coder.internal.preferMATLABHostCompiledLibraries();
            if ~isempty(coder.target) && ~useSharedLibrary
                coder.const(classifier);
                this.TrainingOptionsNonTunable = struct( ...
                    'ChannelPadding', params.ChannelPadding,...
                    'ModelSize', params.ModelSize,...
                    'ModelSizePadded', params.ModelSizePadded,...
                    'Lambdas', params.Lambdas,...
                    'Shrink', params.Shrink,...
                    'PreSmoothColor', params.PreSmoothColor,...
                    'gradient', params.gradient,...
                    'hog', params.hog);
            end
        end
    end

    methods(Hidden)
        %------------------------------------------------------------------
        % Returns feature points in a struct
        %------------------------------------------------------------------
        function s = toStruct(obj)
            s = struct( ...
                'ModelName',obj.ModelName,...
                'ObjectTrainingSize' ,obj.ObjectTrainingSize,...
                'NumWeakLearners' ,obj.NumWeakLearners,...
                'Classifier', obj.Classifier,...
                'TrainingOptions', obj.TrainingOptions);
            s.TrainingOptions.Threshold = -1;
            s.TrainingOptions.MinSize = s.TrainingOptions.ModelSize;
            s.TrainingOptions.MaxSize = s.TrainingOptions.ModelSize;
            s.TrainingOptions.WindowStride = double(0);
            s.TrainingOptions.NumScaleLevels = double(0);
            s.TrainingOptions.ROI = uint32([0 0 0 0]);
            s.TrainingOptions.UseROI = false;
        end
    end

    %======================================================================
    % Get/Set Properties
    %======================================================================
    methods
        function sz = get.ObjectTrainingSize(this)
            sz = this.TrainingOptions.ModelSize;
        end

        function num = get.NumWeakLearners(this)
            num = size(this.Classifier.child, 2);
        end

        function set.ModelName(this, value)
            if isa(value, 'string')
                validateattributes(value,{'string'}, {'nonempty','scalar'});
            else
                validateattributes(value,{'char'}, {'nonempty'});
            end
            this.ModelName = value;
        end
    end

    %======================================================================
    methods(Hidden)
        function cls = getClassifier(this)
            cls = this.Classifier;
        end

        function params = getTrainingOptions(this)
            params = this.TrainingOptions;
        end
    end

    %======================================================================
    methods(Access = private)

        function [detections, info] = detectionTransform(this, data, info, params)
            % Standardize data to cell array in case it is a numeric value or a
            % table.

            if isempty(coder.target)
                data = iCheckDatastoreOutput(data, info, 1);
                data = iConvertToCellIfNeeded(data);

                % Allocate output cell array.
                numImages  = size(data,1);
                detections = cell(numImages, 2);

                % Apply the detector to each image.
                for i = 1:numImages
                    I = data{i,1};

                    try
                        acfObjectDetector.checkImage(I);

                        acfObjectDetector.checkImageAndModelSizes(...
                            size(I,[1 2]), this.TrainingOptions.ModelSize, params);
                    catch ME
                        excp = vision.internal.detector.ObjectDetectorDatastoreException(...
                            ME, info, i);
                        throw(excp);
                    end

                    Iroi = vision.internal.detector.cropImageIfRequested(I, ...
                        params.ROI, params.UseROI);

                    [bboxes, scores] = computePyramidAndDetect(this, Iroi);

                    [bboxes, scores] = postprocessDetections(this, Iroi, bboxes, scores, params);

                    detections{i,1} = bboxes;
                    detections{i,2} = scores;
                end
            else
                detections = [];
            end
        end

        %------------------------------------------------------------------
        function [bboxes, scores] = computePyramidAndDetect(this, I)
            % Compute scale-space pyramid of channels.
            P = vision.internal.acf.computePyramid(I, this.TrainingOptions);

            % Apply the detector on feature pyramid.
            [bboxes, scores] = vision.internal.acf.detect(P, ...
                this.Classifier, this.TrainingOptions);
        end

        %------------------------------------------------------------------
        function [bboxes, scores] = postprocessDetections(~, I, bboxes, scores, params)
            bboxes = round(bboxes);

            % Channels are padded during detection. This may lead to negative box
            % positions. Clip boxes to ensure they are within image boundary.
            bboxes = vision.internal.detector.clipBBox(bboxes, size(I));

            if params.SelectStrongest
                [bboxes, scores] = selectStrongestBbox(bboxes, scores, ...
                    'RatioType', 'Min', 'OverlapThreshold', 0.65);
            end

            bboxes(:,1:2) = vision.internal.detector.addOffsetForROI(...
                bboxes(:,1:2), params.ROI, params.UseROI);
        end

        %------------------------------------------------------------------
        function this = updateTrainingOptions(this, params)
            this.TrainingOptions.Threshold       = params.Threshold;
            this.TrainingOptions.MinSize         = params.MinSize;
            this.TrainingOptions.MaxSize         = params.MaxSize;
            this.TrainingOptions.WindowStride    = params.WindowStride;
            this.TrainingOptions.NumScaleLevels  = params.NumScaleLevels;
        end

    end
    %======================================================================
    % Save/Load
    %======================================================================
    methods(Hidden)
        function s = saveobj(this)
            s.ModelName       = this.ModelName;
            s.Classifier      = this.Classifier;
            s.TrainingOptions = this.TrainingOptions;
            s.Version         = 1.0;
        end
    end

    methods(Static, Hidden)
        function this = loadobj(s)
            this = acfObjectDetector(s.Classifier, s.TrainingOptions);
            % By default, the model name is set by the training function.
            % The users may change it to their preferred name.
            this.ModelName = s.ModelName;
        end
    end


    %======================================================================
    % Parameter validation routines.
    %======================================================================
    methods(Static, Hidden, Access = protected)
        function params = parseDetectInputs(I, modelSize, varargin)

            validateattributes(I,...
                {'numeric','matlab.io.Datastore', 'matlab.io.datastore.Datastore'}, ...
                {}, mfilename, 'I');

            if isnumeric(I)
                acfObjectDetector.checkImage(I);
                imageSize = size(I);
            else
                data = preview(I);
                data = iConvertToCellIfNeeded(data);
                imageFromDS = data{1,1};
                acfObjectDetector.checkImage(imageFromDS);
                imageSize = size(imageFromDS);
            end

            params = acfObjectDetector.parseDetectParameters(imageSize(1:2), modelSize, varargin{:});
        end

        %------------------------------------------------------------------
        function params = parseDetectParameters(imageSize, modelSize, varargin)
            p = inputParser;
            p.addOptional('ROI', zeros(0,4));
            p.addParameter('Threshold', -1, ...
                @(x)validateattributes(x,{'numeric'},...
                {'nonempty','nonsparse','scalar','real','finite'}, ...
                mfilename, 'Threshold'));
            p.addParameter('WindowStride', 4, ...
                @(x)validateattributes(x,{'numeric'},...
                {'nonempty','nonsparse','real','positive','scalar','integer'},...
                mfilename, 'WindowStride'));
            p.addParameter('NumScaleLevels', 8, ...
                @(x)validateattributes(x,{'numeric'},...
                {'nonempty','nonsparse','scalar','real','integer','positive'},...
                mfilename,'NumScaleLevels'));
            p.addParameter('MinSize', modelSize);
            p.addParameter('MaxSize', imageSize);
            p.addParameter('SelectStrongest', true, ...
                @(x)vision.internal.inputValidation.validateLogical(x,'SelectStrongest'));

            p.parse(varargin{:});
            userInput = p.Results;

            wasMinSizeSpecified = ~ismember('MinSize', p.UsingDefaults);
            wasMaxSizeSpecified = ~ismember('MaxSize', p.UsingDefaults);

            if wasMinSizeSpecified
                vision.internal.detector.ValidationUtils.checkMinSize(userInput.MinSize, modelSize, mfilename);
            else
                % Set min size to model training size if not user specified.
                userInput.MinSize = modelSize;
            end

            if wasMaxSizeSpecified
                vision.internal.detector.ValidationUtils.checkMaxSize(userInput.MaxSize, modelSize, mfilename);
                % note: default max size set above in inputParser to size(I)
            end

            if wasMaxSizeSpecified && wasMinSizeSpecified
                % Cross validate min and max size
                coder.internal.errorIf(any(userInput.MinSize >= userInput.MaxSize) , ...
                    'vision:ObjectDetector:minSizeGTMaxSize');
            end

            userInput.UseROI = ~ismember('ROI', p.UsingDefaults);

            acfObjectDetector.checkImageAndModelSizes(imageSize, modelSize, userInput, wasMinSizeSpecified);

            % Set user input to expected type
            params.ROI              = double(userInput.ROI);
            params.UseROI           = logical(userInput.UseROI);
            params.Threshold        = double(userInput.Threshold);
            params.MinSize          = double(userInput.MinSize);
            params.MaxSize          = double(userInput.MaxSize);
            params.SelectStrongest  = logical(userInput.SelectStrongest);
            params.WindowStride     = double(userInput.WindowStride);
            params.NumScaleLevels   = double(userInput.NumScaleLevels);
            params.WasMinSizeSpecified = logical(wasMinSizeSpecified);
        end

        %------------------------------------------------------------------
        function checkImage(I)
            if ismatrix(I)
                vision.internal.inputValidation.validateImage(I, 'I', 'grayscale');
            else
                vision.internal.inputValidation.validateImage(I, 'I', 'rgb');
            end
        end

        %------------------------------------------------------------------
        function checkImageAndModelSizes(imageSize, modelSize, params, wasMinSizeSpecified)
            if nargin == 3
                wasMinSizeSpecified = params.WasMinSizeSpecified;
            end

            if params.UseROI
                vision.internal.detector.checkROI(params.ROI, imageSize);
                if ~isempty(params.ROI)
                    sz = params.ROI([4 3]);
                    vision.internal.detector.ValidationUtils.checkImageSizes(sz, params, wasMinSizeSpecified, ...
                        modelSize, ...
                        'vision:ObjectDetector:ROILessThanMinSize', ...
                        'vision:ObjectDetector:ROILessThanModelSize');
                else
                    validateattributes(params.ROI, {'numeric'}, ...
                        {'numel', 4, 'vector'});
                end

            else
                vision.internal.detector.ValidationUtils.checkImageSizes(imageSize, params, wasMinSizeSpecified, ...
                    modelSize, ...
                    'vision:ObjectDetector:ImageLessThanMinSize', ...
                    'vision:ObjectDetector:ImageLessThanModelSize');
            end
        end

        %------------------------------------------------------------------
        % Issue warning if sz < min size or sz < model size.
        %------------------------------------------------------------------
        function checkImageSizes(sz, userInput, wasMinSizeSpecified, modelSize, minSizeID, modelID)
            if wasMinSizeSpecified
                if any(sz < userInput.MinSize)
                    warning(message(minSizeID, ...
                        acfObjectDetector.printSizeVector(sz),...
                        acfObjectDetector.printSizeVector(userInput.MinSize)));
                end
            else
                if any(sz < modelSize)
                    warning(message(modelID, ...
                        acfObjectDetector.printSizeVector(sz),...
                        acfObjectDetector.printSizeVector(modelSize)));
                end
            end
        end

        %------------------------------------------------------------------
        function vstr = printSizeVector(v)
            vstr = sprintf('[%d %d]',v(1),v(2));
        end

        %------------------------------------------------------------------
        function checkSize(sz, name)
            validateattributes(sz,{'numeric'},...
                {'nonempty', 'nonsparse', 'real', 'finite', 'integer', 'positive', 'size', [1,2]},...
                mfilename, name);
        end

        % -----------------------------------------------------------------
        % Check Training Options Parameters value
        % -----------------------------------------------------------------
        function checkClassifierInput(classifier)

            coder.internal.errorIf(~isfield(classifier, 'fids'),...
                'vision:acfObjectDetector:InvalidClassifier');
            coder.internal.errorIf(~isfield(classifier, 'thrs'),...
                'vision:acfObjectDetector:InvalidClassifier');
            coder.internal.errorIf(~isfield(classifier, 'child'),...
                'vision:acfObjectDetector:InvalidClassifier');
            coder.internal.errorIf(~isfield(classifier, 'hs'),...
                'vision:acfObjectDetector:InvalidClassifier');
            coder.internal.errorIf(~isfield(classifier, 'treeDepth'),...
                'vision:acfObjectDetector:InvalidClassifier');

            validateattributes(classifier.fids, {'uint32'}, ...
                {'2d', 'real', 'nonsparse'}, mfilename);
            sz = size(classifier.fids);
            validateattributes(classifier.thrs, {'single'}, ...
                {'2d', 'real', 'nonsparse', 'size', sz}, mfilename);
            validateattributes(classifier.child, {'uint32'}, ...
                {'2d', 'real', 'nonsparse', 'size', sz}, mfilename);
            validateattributes(classifier.hs, {'single'}, ...
                {'2d', 'real', 'nonsparse', 'size', sz}, mfilename);
        end

        % -----------------------------------------------------------------
        % Check Training Options Parameters value
        % -----------------------------------------------------------------
        function checkParametersInput(params)

            coder.internal.errorIf(~isfield(params, 'ModelName'),...
                'vision:acfObjectDetector:InvalidParameter');
            coder.internal.errorIf(~isfield(params, 'ModelSize'),...
                'vision:acfObjectDetector:InvalidParameter');
            coder.internal.errorIf(~isfield(params, 'ModelSizePadded'),...
                'vision:acfObjectDetector:InvalidParameter');
            coder.internal.errorIf(~isfield(params, 'ChannelPadding'),...
                'vision:acfObjectDetector:InvalidParameter');
            coder.internal.errorIf(~isfield(params, 'NumApprox'),...
                'vision:acfObjectDetector:InvalidParameter');
            coder.internal.errorIf(~isfield(params, 'Shrink'),...
                'vision:acfObjectDetector:InvalidParameter');
            coder.internal.errorIf(~isfield(params, 'SmoothChannels'),...
                'vision:acfObjectDetector:InvalidParameter');
            coder.internal.errorIf(~isfield(params, 'PreSmoothColor'),...
                'vision:acfObjectDetector:InvalidParameter');
            coder.internal.errorIf(~isfield(params, 'NumUpscaledOctaves'),...
                'vision:acfObjectDetector:InvalidParameter');
            coder.internal.errorIf(~isfield(params, 'gradient'),...
                'vision:acfObjectDetector:InvalidParameter');
            coder.internal.errorIf(~isfield(params, 'hog'),...
                'vision:acfObjectDetector:InvalidParameter');
            coder.internal.errorIf(~isfield(params, 'Lambdas'),...
                'vision:acfObjectDetector:InvalidParameter');
            coder.internal.errorIf(~isfield(params, 'NumStages'),...
                'vision:acfObjectDetector:InvalidParameter');
            coder.internal.errorIf(~isfield(params, 'NegativeSamplesFactor'),...
                'vision:acfObjectDetector:InvalidParameter');
            coder.internal.errorIf(~isfield(params, 'MaxWeakLearners'),...
                'vision:acfObjectDetector:InvalidParameter');
        end
    end

    methods(Static, Hidden, Access = protected)
        function results = parseDetectInputs_cg(I, modelSize, varargin)

            % 'pvPairs' is storing the initialized indices of each of
            % the valid parameter names. The value of each field only
            % needs to be a scalar uint32 so 'pvPairs' satisfies the
            % requirement to be the first argument in
            % eml_parse_parameter_inputs.
            pvPairs = struct( ...
                'Threshold',       uint32(0), ...
                'WindowStride',    uint32(0), ...
                'NumScaleLevels',  uint32(0), ...
                'MinSize',         uint32(0), ...
                'MaxSize',         uint32(0), ...
                'SelectStrongest', uint32(1),...
                'UseROI' ,         uint32(0) );

            popt = struct( ...
                'CaseSensitivity', false, ...
                'StructExpand',    true, ...
                'PartialMatching', true);

            % Getting the defaults values
            defaults = acfObjectDetector.getDefaultParameters(size(I), modelSize);

            % Check and get the optional input argument ROI
            if ~isempty(coder.target)
                imgSize = size(I);
                [M, N, ~] = size(I);
                if (mod(nargin,2) == 1 )
                    if ( ~ischar(varargin{1}) )
                        validateattributes(varargin{1}, {'numeric'}, ...
                            {'real', 'nonsparse', 'finite', 'numel', 4}, mfilename, 'roi');
                        vision.internal.detector.checkROI(varargin{1}, size(I));
                        ROI  = uint32(varargin{1});
                        UseROI = true;
                        paramValStartIdx = 2;
                    else
                        paramValStartIdx = 1;
                        ROI = uint32([1 1 imgSize([2 1])]);
                        UseROI = false;
                    end
                else
                    paramValStartIdx = 1;
                    ROI = uint32([1 1 imgSize([2 1])]);
                    UseROI = false;
                end
            end

            optarg = eml_parse_parameter_inputs(pvPairs, popt, varargin{paramValStartIdx:end});

            results.ROI = uint32(ROI);
            results.UseROI = UseROI;

            results.Threshold = eml_get_parameter_value(optarg.Threshold, ...
                defaults.Threshold, varargin{paramValStartIdx:end});

            results.WindowStride = eml_get_parameter_value(optarg.WindowStride, ...
                defaults.WindowStride, varargin{paramValStartIdx:end});

            results.NumScaleLevels = eml_get_parameter_value(optarg.NumScaleLevels, ...
                defaults.NumScaleLevels, varargin{paramValStartIdx:end});

            results.MinSize = eml_get_parameter_value(optarg.MinSize, ...
                defaults.MinSize, varargin{paramValStartIdx:end});

            results.MaxSize = eml_get_parameter_value(optarg.MaxSize, ...
                defaults.MaxSize, varargin{paramValStartIdx:end});

            results.SelectStrongest = eml_get_parameter_value(optarg.SelectStrongest, ...
                defaults.SelectStrongest, varargin{paramValStartIdx:end});

            % Validate all PV pairs
            acfObjectDetector.checkThresholdParam(results.Threshold);
            acfObjectDetector.checkNumScaleLevelsParam(results.NumScaleLevels);
            acfObjectDetector.checkWindowStrideParam(results.WindowStride);
            acfObjectDetector.checkSize(results.MinSize, 'MinSize');
            acfObjectDetector.checkSize(results.MaxSize, 'MaxSize');
            acfObjectDetector.checkSelectStrongestParam(results.SelectStrongest);

            % Validate user input Image
            acfObjectDetector.checkImage(I);

            % Validate user given min Size
            if(results.MinSize == defaults.MinSize)
                wasMinSizeSpecified = false;
            else
                wasMinSizeSpecified = true;
                vision.internal.detector.ValidationUtils.checkMinSize(results.MinSize, modelSize, mfilename);
            end

            % Validate user given max Size
            if(results.MaxSize == defaults.MaxSize)
                wasMaxSizeSpecified = false;
            else
                wasMaxSizeSpecified = true;
                vision.internal.detector.ValidationUtils.checkMaxSize(results.MaxSize, modelSize, mfilename);
            end

            if wasMaxSizeSpecified && wasMinSizeSpecified
                % Cross validate min and max size
                coder.internal.errorIf(any(results.MinSize >= results.MaxSize) , ...
                    'vision:ObjectDetector:minSizeGTMaxSize');
            end

            % Validate ROI
            if results.UseROI
                vision.internal.detector.checkROI(results.ROI, size(I));
                if ~isempty(results.ROI)
                    sz = results.ROI([4 3]);
                    vision.internal.detector.ValidationUtils.checkImageSizes(sz, results, wasMinSizeSpecified, ...
                        modelSize, ...
                        'vision:ObjectDetector:ROILessThanMinSize', ...
                        'vision:ObjectDetector:ROILessThanModelSize');
                end
            else
                vision.internal.detector.ValidationUtils.checkImageSizes([M N], results, wasMinSizeSpecified, ...
                    modelSize, ...
                    'vision:ObjectDetector:ImageLessThanMinSize', ...
                    'vision:ObjectDetector:ImageLessThanModelSize');
            end
        end

        % -----------------------------------------------------------------
        % parse inputs to detect for portable codegen
        % -----------------------------------------------------------------
        function [results,ROI,UseROI,threshold] = parseDetectInputsCGPortable(I, modelSize, varargin)
            % 'pvPairs' is storing the initialized indices of each of
            % the valid parameter names. The value of each field only
            % needs to be a scalar uint32 so 'pvPairs' satisfies the
            % requirement to be the first argument in
            % eml_parse_parameter_inputs.
            pvPairs = struct( ...
                'Threshold',       uint32(0), ...
                'WindowStride',    uint32(0), ...
                'NumScaleLevels',  uint32(0), ...
                'MinSize',         uint32(0), ...
                'MaxSize',         uint32(0), ...
                'SelectStrongest', uint32(1),...
                'UseROI' ,         uint32(0) );

            popt = struct( ...
                'CaseSensitivity', false, ...
                'StructExpand',    true, ...
                'PartialMatching', true);

            % Getting the defaults values
            defaults = acfObjectDetector.getDefaultParameters(size(I), modelSize);

            % Check and get the optional input argument ROI
            if ~isempty(coder.target)
                imgSize = size(I);
                [M, N, ~] = size(I);
                if (mod(nargin,2) == 1 )
                    if ( ~ischar(varargin{1}) )
                        validateattributes(varargin{1}, {'numeric'}, ...
                            {'real', 'nonsparse', 'finite', 'numel', 4}, mfilename, 'roi');
                        vision.internal.detector.checkROI(varargin{1}, size(I));
                        ROI  = uint32(varargin{1});
                        UseROI = true;
                        paramValStartIdx = 2;
                    else
                        paramValStartIdx = 1;
                        ROI = uint32([1 1 imgSize([2 1])]);
                        UseROI = false;
                    end
                else
                    paramValStartIdx = 1;
                    ROI = uint32([1 1 imgSize([2 1])]);
                    UseROI = false;
                end
            end

            optarg = eml_parse_parameter_inputs(pvPairs, popt, varargin{paramValStartIdx:end});

            results.ROI = uint32(ROI);
            results.UseROI = UseROI;

            results.Threshold = eml_get_parameter_value(optarg.Threshold, ...
                defaults.Threshold, varargin{paramValStartIdx:end});

            results.WindowStride = eml_get_parameter_value(optarg.WindowStride, ...
                defaults.WindowStride, varargin{paramValStartIdx:end});

            results.NumScaleLevels = eml_get_parameter_value(optarg.NumScaleLevels, ...
                defaults.NumScaleLevels, varargin{paramValStartIdx:end});

            results.MinSize = eml_get_parameter_value(optarg.MinSize, ...
                defaults.MinSize, varargin{paramValStartIdx:end});

            results.MaxSize = eml_get_parameter_value(optarg.MaxSize, ...
                defaults.MaxSize, varargin{paramValStartIdx:end});

            results.SelectStrongest = eml_get_parameter_value(optarg.SelectStrongest, ...
                defaults.SelectStrongest, varargin{paramValStartIdx:end});

            % Validate all PV pairs
            acfObjectDetector.checkThresholdParam(results.Threshold);
            acfObjectDetector.checkNumScaleLevelsParam(results.NumScaleLevels);
            acfObjectDetector.checkWindowStrideParam(results.WindowStride);
            acfObjectDetector.checkSize(results.MinSize, 'MinSize');
            acfObjectDetector.checkSize(results.MaxSize, 'MaxSize');
            acfObjectDetector.checkSelectStrongestParam(results.SelectStrongest);

            % Validate user input Image
            acfObjectDetector.checkImage(I);

            % Validate user given min Size
            if(results.MinSize == defaults.MinSize)
                wasMinSizeSpecified = false;
            else
                wasMinSizeSpecified = true;
                vision.internal.detector.ValidationUtils.checkMinSize(results.MinSize, modelSize, mfilename);
            end

            % Validate user given max Size
            if(results.MaxSize == defaults.MaxSize)
                wasMaxSizeSpecified = false;
            else
                wasMaxSizeSpecified = true;
                vision.internal.detector.ValidationUtils.checkMaxSize(results.MaxSize, modelSize, mfilename);
            end

            if wasMaxSizeSpecified && wasMinSizeSpecified
                % Cross validate min and max size
                coder.internal.errorIf(any(results.MinSize >= results.MaxSize) , ...
                    'vision:ObjectDetector:minSizeGTMaxSize');
            end

            % Validate ROI
            if results.UseROI
                vision.internal.detector.checkROI(results.ROI, size(I));
                if ~isempty(results.ROI)
                    sz = results.ROI([4 3]);
                    vision.internal.detector.ValidationUtils.checkImageSizes(sz, results, wasMinSizeSpecified, ...
                        modelSize, ...
                        'vision:ObjectDetector:ROILessThanMinSize', ...
                        'vision:ObjectDetector:ROILessThanModelSize');
                end
            else
                vision.internal.detector.ValidationUtils.checkImageSizes([M N], results, wasMinSizeSpecified, ...
                    modelSize, ...
                    'vision:ObjectDetector:ImageLessThanMinSize', ...
                    'vision:ObjectDetector:ImageLessThanModelSize');
            end

            UseROI = results.UseROI;
            ROI = results.ROI;
            threshold = results.Threshold;
        end

        % -----------------------------------------------------------------
        % Check threshold value
        % -----------------------------------------------------------------
        function checkThresholdParam(Threshold)
            validateattributes(Threshold,{'numeric'},...
                {'nonempty', 'nonsparse', 'scalar', 'real', 'finite'}, ...
                mfilename, 'Threshold');
        end

        % -----------------------------------------------------------------
        % Check numscalesvalue value
        % -----------------------------------------------------------------
        function checkNumScaleLevelsParam(NumScaleLevels)
            validateattributes(NumScaleLevels,{'numeric'},...
                {'nonempty','nonsparse', 'scalar', 'real', 'integer', 'positive'},...
                mfilename, 'NumScaleLevels');
        end

        % -----------------------------------------------------------------
        % Check windowStride value
        % -----------------------------------------------------------------
        function checkWindowStrideParam(WindowStride)
            validateattributes(WindowStride,{'numeric'},...
                {'nonempty', 'nonsparse', 'real', 'positive', 'scalar', 'integer'},...
                mfilename, 'WindowStride');
        end

        % -----------------------------------------------------------------
        % Check selectStrongest
        % -----------------------------------------------------------------
        function checkSelectStrongestParam(SelectStrongest)
            vision.internal.inputValidation.validateLogical(SelectStrongest, 'SelectStrongest');
        end
    end

    methods(Static)

        % -----------------------------------------------------------------
        % Set defaults values for input name value pair arguments
        % -----------------------------------------------------------------
        function defaults = getDefaultParameters(imgSize, modelSize)
            M = imgSize(1);
            N = imgSize(2);
            defaults = struct( ...
                'Threshold',       double(-1), ...
                'WindowStride',    double(4), ...
                'NumScaleLevels',  double(8), ...
                'MinSize',         double(modelSize), ...
                'MaxSize',         [M N],...
                'SelectStrongest', true,...
                'useROI' ,         false );
        end

        % -----------------------------------------------------------------
        % Add nontunable properties for codegen
        % -----------------------------------------------------------------
        function props = matlabCodegenSoftNontunableProperties(~)
            props = {'Classifier', 'TrainingOptionsNonTunable'};
        end
    end
end

%--------------------------------------------------------------------------
function tf = iIsDatastore(ds)
tf = isa(ds,'matlab.io.Datastore') || isa(ds,'matlab.io.datastore.Datastore');
end

%--------------------------------------------------------------------------
function data = iConvertToCellIfNeeded(data)
if istable(data)
    data = table2cell(data);
elseif isnumeric(data)
    data = {data};
end
end

%--------------------------------------------------------------------------
function data = iCheckDatastoreOutput(data, info, dsRecordIdx)
if ~isnumeric(data) && ~iscell(data) && ~istable(data)
    excp = vision.internal.detector.ObjectDetectorDatastoreException(...
        'vision:ObjectDetector:invalidDetectionDatastoreReadOutput', info, dsRecordIdx);
    throw(excp);
end
end
% LocalWords:  ACF visiondata datastores imds blds acf bboxes Labeler rcnn adaboost truecolor
% LocalWords:  grayscale roi Bbox bbox nonsparse sz fids thrs hs Upscaled pv numscalesvalue
