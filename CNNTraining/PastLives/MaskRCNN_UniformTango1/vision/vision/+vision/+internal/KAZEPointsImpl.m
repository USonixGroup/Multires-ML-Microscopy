classdef KAZEPointsImpl < vision.internal.FeaturePointsImpl

    % Copyright 2017-2024 The MathWorks, Inc.

    %#codegen
    properties (Dependent = true)
        %Scale Array of point scales
        Scale;
        %Orientation Array of feature orientations
        Orientation;
    end

    % Internal properties that are accessible only indirectly through
    % dependent properties
    properties (Access = protected)
        pDiffusion      = 'region';
        pNumOctaves     = uint8(1);
        pNumScaleLevels = uint8(3);
        pScale          = ones(0,1,'single');
        pOrientation    = ones(0,1,'single');
        pLayerID        = ones(0,1,'int32');
    end

    methods % Accessors for Dependent properties
        function this = KAZEPointsImpl(varargin)
            if nargin > 0
                inputs = parseInputs(this, varargin{:});
                validate(this, inputs);
                this = configure(this, inputs);
            end
        end

        function strongest = selectStrongest(this, N)
            strongest = selectStrongest@vision.internal.FeaturePointsImpl(this,N);
        end

        %------------------------------------------------------------------
        function that = selectUniform(this, N, imageSize)
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
            this = setOrientation(this,in);
        end

        function out = get.Orientation(this)
            out = this.pOrientation;
        end

    end

    methods (Access = protected)
        function inputs = parseInputs(~, varargin)
            defaults = struct(...
                'Metric',         single(0),...
                'Scale',          single(1.6),...
                'Orientation',    single(0), ...
                'LayerID',        int32(1), ...
                'Diffusion',      'region', ...
                'NumOctaves',     uint8(1), ...
                'NumScaleLevels', uint8(3));

            if isempty(coder.target)
                % Parse the PV pairs
                parser = inputParser;

                parser.addRequired('Location');
                parser.addParameter('Metric',         defaults.Metric);
                parser.addParameter('Scale',          defaults.Scale);
                parser.addParameter('Orientation',    defaults.Orientation);
                parser.addParameter('LayerID',        defaults.LayerID);
                parser.addParameter('Diffusion',      defaults.Diffusion);
                parser.addParameter('NumOctaves',     defaults.NumOctaves);
                parser.addParameter('NumScaleLevels', defaults.NumScaleLevels);

                % Parse input
                parser.parse(varargin{:});
                inputs = parser.Results;
            else % codegen

                % 'pvPairs' is storing the initialized indices of each of
                % the valid parameter names. The value of each field only
                % needs to be a scalar uint32 so 'pvPairs' satisfies the
                % requirement to be the first argument in
                % eml_parse_parameter_inputs.
                pvPairs = struct(...
                    'Location',        uint32(0),...
                    'Metric',          uint32(0),...
                    'Scale',           uint32(0),...
                    'Orientation',     uint32(0),...
                    'LayerID',         uint32(0),...
                    'Diffusion',       uint32(0),...
                    'NumOctaves',      uint32(0),...
                    'NumScaleLevels',  uint32(0));

                properties =  struct( ...
                    'CaseSensitivity', false, ...
                    'StructExpand',    true, ...
                    'PartialMatching', false);

                inputs.Location = single(varargin{1});

                optarg = eml_parse_parameter_inputs(pvPairs, properties, varargin{2:end});

                inputs.Metric = eml_get_parameter_value(optarg.Metric, ...
                    defaults.Metric, varargin{2:end});

                inputs.Scale  = eml_get_parameter_value(optarg.Scale , ...
                    defaults.Scale, varargin{2:end});

                inputs.Orientation = eml_get_parameter_value(optarg.Orientation , ...
                    defaults.Orientation, varargin{2:end});

                inputs.LayerID = eml_get_parameter_value(optarg.LayerID, ...
                    defaults.LayerID, varargin{2:end});

                inputs.Diffusion = eml_get_parameter_value(optarg.Diffusion, ...
                    defaults.Diffusion, varargin{2:end});

                inputs.NumOctaves = eml_get_parameter_value(optarg.NumOctaves, ...
                    defaults.NumOctaves, varargin{2:end});

                inputs.NumScaleLevels = eml_get_parameter_value(optarg.NumScaleLevels, ...
                    defaults.NumScaleLevels, varargin{2:end});

            end
        end

        function this = configure(this, inputs)
            this = configure@vision.internal.FeaturePointsImpl(this,inputs);

            n = size(this.Location,1);

            this.pScale           = coder.nullcopy(zeros(n,1,'single'));
            this.pOrientation     = coder.nullcopy(zeros(n,1,'single'));
            this.pScale(:) = ...
                vision.internal.FeaturePointsImpl.assignValue(single(inputs.Scale),n);
            this.pOrientation(:) = ...
                vision.internal.FeaturePointsImpl.assignValue(single(inputs.Orientation),n);


            % Initialize layer IDs so the numbers of octaves and scale
            % levels can be set. Once actual layer IDs are assigned, the
            % numbers of octaves and scale levels cannot be changed.
            this.pLayerID         = coder.nullcopy(zeros(n,1,'int32'));
            this.pLayerID(:) = ...
                vision.internal.FeaturePointsImpl.assignValue(int32(0),n);

            % checking and validation done within each of the following
            this = this.setDiffusion(inputs.Diffusion);
            this = this.setNumOctaves(inputs.NumOctaves);
            this = this.setNumScaleLevels(inputs.NumScaleLevels);
            this = this.setLayerID(inputs.LayerID);
        end

        function validate(this, inputs)
            validate@vision.internal.FeaturePointsImpl(this,inputs);

            this.checkScale(inputs.Scale);
            this.checkOrientation(inputs.Orientation);
            numPts = size(inputs.Location,1);

            % All parameters must have the same number of elements or be a scalar
            vision.internal.FeaturePointsImpl.validateParamLength(numel(inputs.Scale), 'Scale', numPts);
            vision.internal.FeaturePointsImpl.validateParamLength(numel(inputs.Orientation), 'Orientation', numPts);
        end
        %--------------------------------------------------------------------------
        function checkScale(this, scale)
            validateattributes(scale, {'numeric'},...
                {'nonnan', 'finite', 'nonsparse','real','vector','>=',1.6},...
                class(this),'Scale');
        end

        function checkOrientation(this, orientation)
            validateattributes(orientation, {'numeric'},...
                {'nonnan', 'finite', 'nonsparse','real','vector'},...
                class(this),'Orientation');
        end

        function checkLayerID(this, layerIndex)
            nO = this.pNumOctaves;
            nS = this.pNumScaleLevels;
            nL = nO*nS;
            validateattributes(layerIndex, {'numeric'},...
                {'nonnan', 'finite', 'nonsparse','nonnegative', ...
                'integer','vector','>=',1,'<=',nL-2},...
                class(this),'LayerID');
        end

        function validString = checkDiffusion(this, in)
            validStrings = {'region', 'sharpedge', 'edge'};
            validString = validatestring(in, validStrings, class(this), 'Diffusion');
        end

        function checkNumOctaves(this, in)
            vision.internal.errorIfNotFixedSize(in, 'NumOctaves');
            validateattributes(in, {'numeric'},...
                {'scalar','>=', 1, 'real','nonsparse','integer'},...
                class(this), 'NumOctaves');
        end

        function checkNumScaleLevels(this, in)
            vision.internal.errorIfNotFixedSize(in, 'NumScaleLevels');
            validateattributes(in, {'numeric'},...
                {'scalar','>=', 3, 'real','nonsparse','integer'},...
                class(this), 'NumScaleLevels');
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

        function this = setupLayerID(this, layerIndex)
            n = size(this.Location, 1);
            this.checkLayerID(layerIndex);
            vision.internal.FeaturePointsImpl.validateParamLength(numel(layerIndex), 'LayerID', n);
            this.pLayerID = coder.nullcopy(zeros(n,1,'int32'));
            this.pLayerID(:) = ...
                vision.internal.FeaturePointsImpl.assignValue(int32(layerIndex),n);
        end

        function this = setDiffusion(this, in)
            in = this.checkDiffusion(in);
            this.pDiffusion = in;
        end

        function out = getDiffusion(this)
            out = this.pDiffusion;
        end

        function this = setNumOctaves(this, in)
            if all(this.pLayerID == 0)
                this.checkNumOctaves(in);
                this.pNumOctaves = uint8(in);
            else
                errmsgId = 'vision:KAZEPoints:cannotChangeOctaves';
                if isempty(coder.target)
                    error(message(errmsgId));
                else
                    coder.internal.error(errmsgId);
                end
            end
        end

        function out = getNumOctaves(this)
            out = uint8(this.pNumOctaves);
        end

        function this = setNumScaleLevels(this, in)
            if all(this.pLayerID == 0)
                this.checkNumScaleLevels(in);
                this.pNumScaleLevels = uint8(in);
            else
                errmsgId = 'vision:KAZEPoints:cannotChangeScaleLevels';
                if isempty(coder.target)
                    error(message(errmsgId));
                else
                    coder.internal.error(errmsgId);
                end

            end
        end

        function out = getNumScaleLevels(this)
            out = uint8(this.pNumScaleLevels);
        end

        function this = setLayerID(this, in)
            this = setupLayerID(this,in);
        end

        function out = getLayerID(this)
            out = this.pLayerID;
        end
    end
end
