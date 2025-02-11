classdef ORBPointsImpl < vision.internal.FeaturePointsImpl %#codegen
    properties (Dependent = true)
        % Scale Array of point scales
        Scale;
        % Orientation Array of feature orientations
        Orientation;
    end
    
    % Internal properties that are accessible only indirectly through
    % dependent properties
    properties (Access = protected)
        pNumLevels      = uint8(8);
        pScaleFactor    = single(1.2);
        pPatchSize      = int32(31);
        pScale          = ones(0, 1, 'single');
        pOrientation    = ones(0, 1, 'single');
    end
    
    
    methods % Accessors for Dependent properties
        
        function this = ORBPointsImpl(varargin)
            if nargin > 0
                inputs = parseInputs(this, varargin{:});
                validate(this, inputs);
                this = configure(this, inputs);
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
            %   points = ORBPoints(ones(50,2), 'Metric', 1:50);
            %   % keep 2 strongest features
            %   points = selectStrongest(points, 2)
            strongest = selectStrongest@vision.internal.FeaturePointsImpl(this,N);
        end
        
        %------------------------------------------------------------------
        function that = selectUniform(this, N, imageSize)
            % selectUniform Return a uniformly distributed subset of
            % feature points
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
            %   % Detect and display ORB features
            %   points1 = detectORBFeatures(rgb2gray(im));
            %   subplot(1, 2, 1)
            %   imshow(im)
            %   hold on
            %   plot(points1)
            %   hold off
            %   title('Original points')
            %
            %   % Select a uniformly distributed subset of points
            %   numPoints = 100;
            %   points2 = selectUniform(points1, numPoints, size(im));
            %   subplot(1, 2, 2)
            %   imshow(im)
            %   hold on
            %   plot(points2);
            %   hold off
            %   title('Uniformly distributed points')
            
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
            this = setOrientation(this, in);
        end
        
        function out = get.Orientation(this)
            out = this.pOrientation;
        end
        
    end
    
    methods (Access = protected)
        function inputs = parseInputs(~, varargin)
            defaults = struct(...
                'Metric',         single(0),...
                'Scale',          single(1),...
                'Orientation',    single(0), ...
                'NumLevels',      uint8(8), ...
                'PatchSize',      int32(31),...
                'ScaleFactor',    single(1.2));
            
            if isempty(coder.target)
                % Parse the Name-Value pairs
                parser = inputParser;
                
                parser.addRequired('Location');
                parser.addParameter('Metric',         defaults.Metric);
                parser.addParameter('Scale',          defaults.Scale);
                parser.addParameter('Orientation',    defaults.Orientation);
                parser.addParameter('NumLevels',      defaults.NumLevels);
                parser.addParameter('ScaleFactor',    defaults.ScaleFactor);
                parser.addParameter('PatchSize',      defaults.PatchSize);
                
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
                    'NumLevels',       uint32(0),...
                    'PatchSize',       uint32(0),...
                    'ScaleFactor',     uint32(0));
                
                properties =  struct( ...
                    'CaseSensitivity', false, ...
                    'StructExpand',    true, ...
                    'PartialMatching', false);
                
                inputs.Location = varargin{1};
                
                optarg = eml_parse_parameter_inputs(pvPairs, properties, varargin{2:end});
                
                inputs.Metric = eml_get_parameter_value(optarg.Metric, ...
                    defaults.Metric, varargin{2:end});
                
                inputs.Scale  = eml_get_parameter_value(optarg.Scale , ...
                    defaults.Scale, varargin{2:end});
                
                inputs.Orientation = eml_get_parameter_value(optarg.Orientation , ...
                    defaults.Orientation, varargin{2:end});
                
                inputs.NumLevels = eml_get_parameter_value(optarg.NumLevels, ...
                    defaults.NumLevels, varargin{2:end});
                
                inputs.ScaleFactor = eml_get_parameter_value(optarg.ScaleFactor, ...
                    defaults.ScaleFactor, varargin{2:end});
                
                inputs.PatchSize = eml_get_parameter_value(optarg.PatchSize, ...
                    defaults.PatchSize, varargin{2:end});
                
            end
        end
        
        function this = configure(this, inputs)
            this = configure@vision.internal.FeaturePointsImpl(this, inputs);
            this.Location = single(inputs.Location);
            
            n = size(this.Location,1);
            
            this.pScale           = coder.nullcopy(zeros(n, 1, 'single'));
            this.pOrientation     = coder.nullcopy(zeros(n, 1, 'single'));
            this.pScale(:) = ...
                vision.internal.FeaturePointsImpl.assignValue(single(inputs.Scale), n);
            this.pOrientation(:) = ...
                vision.internal.FeaturePointsImpl.assignValue(single(inputs.Orientation), n);
            
            % checking and validation done within each of the following
            this = this.setNumLevels(inputs.NumLevels);
            this = this.setScaleFactor(inputs.ScaleFactor);
            this = this.setPatchSize(inputs.PatchSize);            
        end
        
        function validate(this, inputs)
            validate@vision.internal.FeaturePointsImpl(this, inputs);
            
            this.checkScale(inputs.Scale);
            this.checkOrientation(inputs.Orientation);
            numPts = size(inputs.Location, 1);
            
            % All parameters must have the same number of elements or be a scalar
            vision.internal.FeaturePointsImpl.validateParamLength(numel(inputs.Scale), 'Scale', numPts);
            vision.internal.FeaturePointsImpl.validateParamLength(numel(inputs.Orientation), 'Orientation', numPts);
        end
        %--------------------------------------------------------------------------
        function checkScale(this, scale)
            validateattributes(scale, {'numeric'},...
                { 'real', 'nonsparse','vector', 'nonnan', 'finite', ...
                '>=', 1}, class(this), 'Scale');
        end
        
        function checkOrientation(this, orientation)
            validateattributes(orientation, {'numeric'},...
                {'real', 'nonsparse','vector', 'nonnan', 'finite'},...
                class(this), 'Orientation');
        end
        
        function checkNumLevels(this, in)
            vision.internal.errorIfNotFixedSize(in, 'NumLevels');
            validateattributes(in, {'numeric'},...
                {'real', 'nonsparse','scalar', 'nonnan', 'finite',...
                'integer', '>=', 1}, class(this), 'NumLevels');
        end
        
        function checkScaleFactor(this, in)
            vision.internal.errorIfNotFixedSize(in, 'ScaleFactor');
            validateattributes(in, {'numeric'},...
                {'real', 'nonsparse','vector', 'nonnan', 'finite',...
                '>', 1}, class(this), 'ScaleFactor');
        end
        
        function checkPatchSize(this, in)
            vision.internal.errorIfNotFixedSize(in, 'PatchSize');
            validateattributes(in, {'numeric'},...
                {'real', 'nonsparse','vector', 'nonnan', 'finite',...
                '>=', 31}, class(this), 'PatchSize');
        end
        
    end
    
    methods(Hidden)
        
        function this = setOrientation(this, orientation)
            this.checkForResizing(orientation);
            this.checkOrientation(orientation);
            this.pOrientation = single(orientation);
        end
        
        function this = setNumLevels(this, in)
            this.checkNumLevels(in);
            this.pNumLevels = uint8(in);
        end
        
        function out = getNumLevels(this)
            out = uint8(this.pNumLevels);
        end
        
        function this = setScaleFactor(this, in)
            this.checkScaleFactor(in);
            this.pScaleFactor = single(in);
            
        end
        
        function out = getScaleFactor(this)
            out = single(this.pScaleFactor);
        end
        
        function this = setPatchSize(this, in)
            this.checkPatchSize(in);
            this.pPatchSize = in;
        end
        
        function out = getPatchSize(this)
            out = this.pPatchSize;
        end
    end
end

%  In order for method help to work properly for subclasses, this classdef
%  file cannot have a comment block at the top, so the following remark and
%  copyright/version information are provided here at the end. Please do
%  not move them.

% Copyright 2018 The MathWorks, Inc.