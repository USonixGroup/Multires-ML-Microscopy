classdef SIFTPointsImpl < vision.internal.FeaturePointsImpl %#codegen

    properties (Dependent = true)
        %Scale Array of keypoint scales
        Scale;
        %Orientation Array of keypoint orientations
        Orientation;
        %Octave Array of keypoint octaves
        Octave;
        %Layer Array of keypoint layers
        Layer;
    end

    % Internal properties that are accessible only indirectly through
    % dependent properties
    properties (Access = protected)
        pScale           = ones(0,1,'single');
        pOrientation     = ones(0,1,'single');
        pOctave          = ones(0,1,'int32');
        pLayer           = ones(0,1,'int32');
    end

    methods % Accessors for Dependent properties

        function this = SIFTPointsImpl(varargin)
            if nargin > 0
                inputs = parseInputs(this, varargin{:});
                validate(this,inputs);
                this = configure(this,inputs);
            end
        end

        function strongest = selectStrongest(this, N)
            %selectStrongest Return N points with strongest metrics
            %
            %   strongest = selectStrongest(points, N) keeps N
            %   points with strongest metrics.
            %
            %   Example
            %   -------
            %   % create object holding 50 points
            %   points = SIFTPoints(ones(50,2), 'Metric', 0.11:0.01:0.6);
            %   % keep 2 strongest features
            %   points = selectStrongest(points, 2)

            strongest = selectStrongest@vision.internal.FeaturePointsImpl(this,N);
        end

        %------------------------------------------------------------------
        function that = selectUniform(this, N, imageSize)
            % selectUniform Return a uniformly distributed subset of feature points
            %   pointsOut = selectUniform(pointsIn, N, imageSize) keeps N
            %   points with the strongest metrics approximately uniformly
            %   distributed throughout the image. imageSize is a 2-element
            %   or 3-element vector containing the size of the image.
            %
            %   Example - Select a uniformly distributed subset of features
            %   -----------------------------------------------------------
            %   % Read in the image
            %   im = imread('yellowstone_left.png');
            %
            %   % Detect and display SIFT features
            %   points1 = detectSIFTFeatures(rgb2gray(im));
            %   subplot(1, 2, 1);
            %   imshow(im);
            %   hold on
            %   plot(points1);
            %   hold off
            %   title('Original points');
            %
            %   % Select a uniformly distributed subset of points
            %   numPoints = 100;
            %   points2 = selectUniform(points1, numPoints, size(im));
            %   subplot(1, 2, 2);
            %   imshow(im);
            %   hold on
            %   plot(points2);
            %   hold off
            %   title('Uniformly distributed points');
            %
            %   See also detectSIFTFeatures, matchFeatures, vision.PointTracker,
            %       estimateFundamentalMatrix

            that = selectUniform@vision.internal.FeaturePointsImpl(this, N, imageSize);
        end

        function this = set.Scale(this, in)
            this.checkForResizing(in);
            this.checkScale(in);
            this.pScale = single(in);
        end

        function out = get.Scale(this)
            out = this.pScale;
        end

        function this = set.Orientation(this, in)
            this.checkForResizing(in);
            this.checkOrientation(in);
            this.pOrientation = single(in);
        end

        function out = get.Orientation(this)
            out = this.pOrientation;
        end

        function this = set.Octave(this, in)
            this.checkForResizing(in);
            this.checkOctave(in);
            this.pOctave = int32(in);
        end

        function out = get.Octave(this)
            out = this.pOctave;
        end

        function this = set.Layer(this, in)
            this.checkForResizing(in);
            this.checkLayer(in);
            this.pLayer = int32(in);
        end

        function out = get.Layer(this)
            out = this.pLayer;
        end
    end
    methods (Access = protected)
        function inputs = parseInputs(~, varargin)

            defaults = struct(...
                'Scale',           single(1.6),...
                'Metric',          single(0),...
                'Orientation',     single(0),...
                'Octave',          int32(0),...
                'Layer',           int32(0));

            if isempty(coder.target)
                % Parse the PV pairs
                parser = inputParser;

                parser.addRequired('Location');
                parser.addParameter('Scale',           defaults.Scale);
                parser.addParameter('Metric',          defaults.Metric);
                parser.addParameter('Orientation',     defaults.Orientation);
                parser.addParameter('Octave',          defaults.Octave);
                parser.addParameter('Layer',           defaults.Layer);

                % Parse input
                parser.parse(varargin{:});

                inputs = parser.Results;

            else % codegen

                pvPairs = struct(...
                    'Location',        uint32(0),...
                    'Metric',          uint32(0),...
                    'Scale',           uint32(0),...
                    'Orientation',     uint32(0),...
                    'Octave',          uint32(0),...
                    'Layer',           uint32(0));

                properties =  struct( ...
                    'CaseSensitivity', false, ...
                    'StructExpand',    true, ...
                    'PartialMatching', false);

                inputs.Location = single(varargin{1});

                optarg = eml_parse_parameter_inputs(pvPairs, properties, varargin{2:end});

                inputs.Scale  = eml_get_parameter_value(optarg.Scale , ...
                    defaults.Scale, varargin{2:end});

                inputs.Metric = eml_get_parameter_value(optarg.Metric, ...
                    defaults.Metric, varargin{2:end});

                inputs.Orientation = eml_get_parameter_value(optarg.Orientation , ...
                    defaults.Orientation, varargin{2:end});

                inputs.Octave = eml_get_parameter_value(optarg.Octave , ...
                    defaults.Octave, varargin{2:end});

                inputs.Layer = eml_get_parameter_value(optarg.Layer , ...
                    defaults.Layer, varargin{2:end});

            end
        end

        function this = configure(this, inputs)

            this = configure@vision.internal.FeaturePointsImpl(this,inputs);

            n = size(this.Location,1);

            this.pScale           = coder.nullcopy(zeros(n,1,'single'));
            this.pMetric          = coder.nullcopy(zeros(n,1,'single'));
            this.pOrientation     = coder.nullcopy(zeros(n,1,'single'));
            this.pOctave          = coder.nullcopy(zeros(n,1,'int32'));
            this.pLayer           = coder.nullcopy(zeros(n,1,'int32'));

            this.pScale(:) = ...
                vision.internal.FeaturePointsImpl.assignValue(single(inputs.Scale),n);

            this.pMetric(:) = ...
                vision.internal.FeaturePointsImpl.assignValue(single(inputs.Metric),n);

            this.pOrientation(:) = ...
                vision.internal.FeaturePointsImpl.assignValue(single(inputs.Orientation),n);

            this.pOctave(:) = ...
                vision.internal.FeaturePointsImpl.assignValue(single(inputs.Octave),n);

            this.pLayer(:) = ...
                vision.internal.FeaturePointsImpl.assignValue(single(inputs.Layer),n);

        end

        function validate(this, inputs)

            validate@vision.internal.FeaturePointsImpl(this,inputs);

            this.checkScale(inputs.Scale);
            this.checkMetric(inputs.Metric);
            this.checkOrientation(inputs.Orientation);
            this.checkOctave(inputs.Octave);
            this.checkLayer(inputs.Layer);

            numPts = size(inputs.Location,1);

            % All parameters must have the same number of elements or be a scalar
            vision.internal.FeaturePointsImpl.validateParamLength(numel(inputs.Scale), 'Scale', numPts);
            vision.internal.FeaturePointsImpl.validateParamLength(numel(inputs.Metric), 'Metric', numPts);
            vision.internal.FeaturePointsImpl.validateParamLength(numel(inputs.Orientation), 'Orientation', numPts);
            vision.internal.FeaturePointsImpl.validateParamLength(numel(inputs.Octave), 'Octave',  numPts);
            vision.internal.FeaturePointsImpl.validateParamLength(numel(inputs.Layer), 'Layer',  numPts);
        end

        %--------------------------------------------------------------------------

        function checkScale(this, scale)
            validateattributes(scale, {'numeric'},...
                {'positive','nonnan', 'finite', 'nonsparse', 'vector'},...
                class(this),'Scale');
        end

        function checkMetric(this, metric)
            validateattributes(metric, {'numeric'},...
                {'nonnan', 'finite', 'nonsparse', 'real', 'vector',...
                '>=',0.0,'<=',1.0}, class(this),'Metric');
        end

        function checkOctave(this,in)
            validateattributes(in, {'numeric'},...
                {'nonnegative','integer', 'vector', 'nonsparse', 'finite'}, ...
                class(this),'Octave');
        end

        function checkLayer(this,in)
            validateattributes(in, {'numeric'},...
                {'integer', 'vector', 'nonsparse', 'finite','>=',0.0}, ...
                class(this),'Layer');
        end
    end

    methods(Hidden)
        %------------------------------------------------------------------
        % Set Orientation values. Used to update orientation values after
        % feature extraction.
        %------------------------------------------------------------------
        function this = setOrientation(this, orientation)
            this.checkForResizing(orientation);
            this.checkOrientation(orientation);
            this.pOrientation = single(orientation);
        end
    end
end

%  In order for method help to work properly for subclasses, this classdef
%  file cannot have a comment block at the top, so the following remark and
%  copyright/version information are provided here at the end. Please do
%  not move them.

% Copyright 2021-2024 The MathWorks, Inc.
