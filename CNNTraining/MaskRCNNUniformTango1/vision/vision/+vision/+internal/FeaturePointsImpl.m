classdef FeaturePointsImpl
    %

    % Copyright 2013-2024 The MathWorks, Inc.
    
    %#codegen
    
    properties (Access='public', Dependent = true)
        %Location Array of [x y] point coordinates
        Location;
        %Metric Value indicating feature's strength
        Metric;
    end
    
    properties (SetAccess='private', GetAccess='public', Dependent = true)
        %Count Number of stored interest points
        Count;
    end
    
    properties (Access='protected')
        pLocation = ones(0,2,'single');
        pMetric   = ones(0,1,'single');
    end
    
    
    methods
        %------------------------------------------------------------------
        function this = FeaturePointsImpl(varargin)
            if nargin > 0
                inputs = parseInputs(this, varargin{:});
                validate(this,inputs);
                this = configure(this,inputs);
            end
        end
        
        %------------------------------------------------------------------
        function strongest = selectStrongest(this, N)
            
            validateattributes(N, {'numeric'}, {'scalar', 'integer', ...
                'positive'}, class(this));
            
            coder.varsize('idx', [inf, 1]);
            idx = coder.nullcopy(zeros(size(this.pMetric), 'like', this.pMetric));
            
            [~, idx(:,1)] = sort(this.pMetric,'descend');
            
            if N > length(idx)
                NN = length(idx);
            else
                NN = N;
            end
            
            if isSimMode()
                % Use this's subsref implementation
                s.type = '()';
                s.subs = {idx(1:NN)};
                strongest = subsref(this,s);
            else
                strongest = select(this, idx(1:NN));
            end
        end
        
        %------------------------------------------------------------------
        function pointsOut = selectUniform(this, numPoints, imageSize)
            
            validateattributes(numPoints, {'numeric'}, {'scalar', 'integer', ...
                'positive', 'nonsparse', 'real'}, class(this));
            
            validateattributes(imageSize, {'numeric'}, {'vector', 'integer', ...
                'positive', 'nonsparse'}, class(this));
            if numel(imageSize) > 3
                validateattributes(imageSize, {'numeric'}, {'vector', 'integer', ...
                    'positive', 'numel', 2}, class(this));
            end
            
            coder.varsize('metric', [inf, 1]);
            coder.varsize('origIdx', [1, inf]);
            coder.varsize('points', [inf, 2]);
            coder.varsize('idxOut', [inf, 1]);
            coder.varsize('idx', [inf, 1]);
            coder.varsize('idxNum', [1, inf]);
            
            imageSize = imageSize([2,1]);
            points = this.Location;
            
            metric = this.Metric;
            origIdx = 1:this.Count;
            
            idxOut = coder.nullcopy(zeros(size(this.pMetric), 'like', this.pMetric));
            
            if numPoints > length(idxOut)
                NN = length(idxOut);
            else
                NN = double(numPoints);
            end
            
            first = 1;
            if isSimMode() && ~isa(this.Location, 'gpuArray')
                idxNum = visionSelectUniformPoints(double(points), imageSize,...
                    double(metric), NN);
                idxNum = sort(idxNum);
                idx = false(size(origIdx));
                idx(idxNum) = true;
            else
                idx = selectPoints(points, imageSize, metric, NN);
                idxNum = origIdx(idx);
            end
            idxOut(first:numel(idxNum)) = idxNum';
            first = numel(idxNum) + 1;
            
            while(first <= NN)
                origIdx = origIdx(~idx);
                points = points(~idx, :);
                metric = metric(~idx);
                n = NN - (first - 1);
                
                if isSimMode() && ~isa(this.Location, 'gpuArray')
                    idxNum = visionSelectUniformPoints(double(points), imageSize,...
                        double(metric), n);
                    idx = false(size(points,1), 1);
                    idx(idxNum) = true;
                    idxNum = origIdx(idx);
                else
                    idx = selectPoints(points, imageSize, metric, n);
                    idxNum = origIdx(idx);
                end
                
                idxOut(first:first+numel(idxNum)-1) = idxNum';
                first = first+numel(idxNum);
            end
            
            if isSimMode()
                s.type = '()';
                s.subs = {idxOut(1:NN)};
                pointsOut = subsref(this, s);
            else
                pointsOut = select(this, idxOut(1:NN));
            end
        end
        
        %------------------------------------------------------------------
        % Note:  NUMEL is not overridden because it interferes with the
        %        desired operation of this object. FeaturePoints is a scalar
        %        object which pretends to be a vector. NUMEL is used during
        %        subsref operations and therefore needs to represent true
        %        number of elements for the object, which is always 1.
        %------------------------------------------------------------------
        function out = length(this)
            out = this.Count;
        end
        
        %------------------------------------------------------------------
        function out = isempty(this)
            out = this.Count == 0;
        end
        
        %-------------------------------------------------------------------
        function ind = end(this,varargin)
            %END Last index in indexing expression for FeaturePoints
            %   end(V,K,N) is called for indexing expressions involving the
            %   FeaturePoints vector V when END is part of the K-th index out of
            %   N indices. For example, the expression V(end-1,:) calls the
            %   FeaturePoints vector's END method with END(V,1,2).
            %
            %   See also end
            
            if isempty(varargin) || varargin{1} == 1
                ind = this.Count;
            else
                ind = 1;
            end
        end
        %------------------------------------------------------------------
        % overloading the size functionality - g3343170
        %------------------------------------------------------------------
        function varargout = size(this, varargin)
                % size(obj, 1, 2, 3, ...) -> error
                % size(obj, vector)       -> error
                narginchk(1,2);
                if ~isempty(varargin)
                    validateattributes(varargin{1},{'numeric'},...
                        {'positive','scalar','real','finite','nonsparse'},...
                        'size');
                end
                
                % Use the builtin function to validate the inputs and
                % outputs.
                switch nargout
                    case 0
                        % size(obj)       :  ans = [this.Count 1]
                        % size(obj, 1)    :  ans = this.Count
                        % size(obj, 2)    :  ans = 1
                        % size(obj, d > 2):  ans = 1
                        [varargout{1:nargout}] = ...
                            builtin('size', this, varargin{:});
                        if isempty(varargin)
                            % size(obj)
                            varargout{1}(1) = this.Count;
                        elseif isscalar(varargin) && varargin{1} ~= 1
                            % size(obj, 2), size(obj,n) n~=1 = 1
                            varargout{1} = 1;
                           
                        else
                            % size(obj, 1)
                            varargout{1} = this.Count;
                        end
                        
                    case 1
                        % D = size(obj)       :  D = [this.Count, 1]
                        % n = size(obj, 1)    :  n = this.Count
                        % m = size(obj, 2)    :  m = 1
                        % p = size(obj, d > 2):  p = 1
                        n = builtin('size', this, varargin{:});
                        if isempty(varargin)
                            % D = size(obj);
                            varargout{1} = [this.Count, 1];
                        elseif isscalar(varargin) && varargin{1} ~= 1
                            % m = size(obj, 2);
                            % p = size(obj, d > 3);
                            varargout{1} = n;
                        else
                            % n = size(obj, 1);
                            varargout{1} = this.Count;
                        end
                        
                    case 2
                        % [n, m] = size(obj);
                        % [n, m] = size(obj, d) --> issues error
                        [n, ~] = builtin('size', this, varargin{:});
                        varargout{1} = this.Count;
                        varargout{2} = n;
                        
                    otherwise
                        % [n, m, p, ...] = size(obj)
                        % [n, m, p, ...] = size(obj, d) ---> issues error
                        %  p, ... are always 1
                        [n, ~, varargout{3:nargout}] = ...
                            builtin('size', this, varargin{:});
                        varargout{1} = this.Count;
                        varargout{2} = n;
                end
        end

        %-----------------------------------------------
        function this = set.Location(this, in)
            this.checkForResizing(in);
            this.checkLocation(in);
            this.pLocation(:) = single(in);
        end
        function out = get.Location(this)
            out = this.pLocation;
        end
        %------------------------------------------------
        function this = set.Metric(this, in)
            this.checkForResizing(in);
            this.checkMetric(in);
            this.pMetric(:) = single(in);
        end
        function out = get.Metric(this)
            out = this.pMetric;
        end
        %-------------------------------------------------
        function out = get.Count(this)
            out = size(this.Location,1);            
        end
        
    end
    methods(Access = protected)
        
        function inputs = parseInputs(~, varargin)
            
            if isSimMode()
                % Parse the PV pairs
                parser = inputParser;
                parser.addRequired('Location')
                parser.addParameter('Metric', single(0));
                
                % Parse input
                parser.parse(varargin{:});
                
                inputs = parser.Results;
                
            else
                defaultsNoVal = struct('Metric', uint32(0));
                
                properties = struct( ...
                    'CaseSensitivity', false, ...
                    'StructExpand',    true, ...
                    'PartialMatching', false);
                
                inputs.Location = single(varargin{1});
                
                defaults = vision.internal.FeaturePointsImpl.getParameterDefaults();
                
                optarg = eml_parse_parameter_inputs(defaultsNoVal, properties, varargin{2:end});
                
                inputs.Metric = (eml_get_parameter_value( ...
                    optarg.Metric, defaults.Metric, varargin{2:end}));
                
            end
        end
        
        function validate(this, inputs)
            
            this.checkLocation(inputs.Location);
            this.checkMetric(inputs.Metric);
            
            numPts = size(inputs.Location,1);
            % Parameters must have the same number of elements or be a scalar
            vision.internal.FeaturePointsImpl.validateParamLength(numel(inputs.Metric), 'Metric', numPts);
            
        end
        
        function checkForResizing(this, in)
            % Prevent resizing of public properties
            coder.internal.errorIf(size(in,1) ~= this.Count, ...
                'vision:FeaturePoints:cannotResizePoints', class(this));
        end
        
        %------------------------------------------------------------------
        function this = configure(this,inputs)
            
            if ~isSimMode()
                if eml_is_const(size(inputs.Location))
                    eml_invariant(all(size(inputs.Location) > 0) , ...
                        eml_message('vision:FeaturePoints:constSizeEmpty'));
                end
            end
            
            n = size(inputs.Location,1);
            
            % If either location or metric is a gpuArray then store both as
            % gpuArrays.
            if isa(inputs.Metric, 'gpuArray') || isa(inputs.Location, 'gpuArray')
                prototype = zeros(0,1,'single','gpuArray');
            else
                % built-in type/codegen code path
                prototype = zeros(0,1,'single');
            end
            
            this.pLocation = coder.nullcopy(zeros(n,2,'like',prototype));
            
            this.pLocation = single(inputs.Location);
            
            this.pMetric = coder.nullcopy(zeros(n,1,'like',prototype));
            
            this.pMetric(:) = ...
                vision.internal.FeaturePointsImpl.assignValue(single(inputs.Metric),n);
            
        end
        
        function checkLocation(this, location)
            
            if isa(location, 'gpuArray')
                checkGPULocation(this, location);
            else
                validateattributes(location, {'numeric'}, {'nonnan', ...
                    'finite', 'nonsparse', 'real', 'positive', 'size',[NaN,2]}, ...
                    class(this)); %#ok<*EMCA>
            end
        end
        
        function checkGPULocation(this, location)
            validateattributes(location, {'numeric'}, {'nonnan', ...
                'finite','real'}, ...
                class(this)); %#ok<*EMCA>
            
            % positive
            if any(location(:) <= 0)
                validateattributes(-1, {'numeric'}, {'positive'}, class(this));
            end
            
            % size
            if size(location,2) ~= 2
                validateattributes(ones(3,3),{'numeric'},{'size',[NaN,2]},class(this));
            end
        end
        
        function checkMetric(this, metric)
            checkParam(metric, class(this),'Metric');
        end
        
        function checkScale(this, scale)
            checkParam(scale,class(this),'Scale');
        end
        
        function checkOrientation(this, orientation)
            checkParam(orientation, class(this),'Orientation');
        end
    end
    
    methods (Static, Access = protected)
        
        function validateParamLength(numelParam, paramName, numPts)
            coder.internal.errorIf(~(numelParam == 1 || numelParam == numPts), ...
                'vision:FeaturePoints:invalidParamLength', paramName);
        end
        
        function defaults = getParameterDefaults()
            defaults = struct('Metric', single(0));
        end
        
        function v = assignValue(x,N)
            % copy x into v, expanding scalars if needed
            coder.inline('always');
            v = coder.nullcopy(ones(N,1,'like',x));
            if isscalar(x)
                v = repmat(x(:),N,1);
            else
                v = x(:);
            end
        end
    end
end

function checkParam(in,fname,pname)
validateattributes(in, {'numeric'},...
    {'nonnan', 'finite', 'nonsparse','real','vector'},...
    fname, pname);
end

%--------------------------------------------------------------------------
function pointsIdx = selectPoints(points, imageSize, metric, numPoints)
if numPoints == 1
    [~, numericIdx] = max(metric);
    pointsIdx = false(size(points, 1), 1);
    pointsIdx(numericIdx) = true;
    return;
end

aspectRatio = imageSize(1) / imageSize(2);
h = max(floor(sqrt(numPoints / aspectRatio)), 1);
w = max(floor(h * aspectRatio), 1);

nBins = [w, h];
gridStep = imageSize ./ (nBins + 1);

binIdx = zeros(nBins);

for i = 1:size(points, 1)
    whichBin = min([floor(points(i, :) ./ gridStep) + 1; nBins]);
    idx = binIdx(whichBin(1), whichBin(2));
    if idx < 1 || metric(idx) < metric(i)
        binIdx(whichBin(1), whichBin(2)) = i;
    end
end
numericIdx = binIdx(binIdx > 0);
numericIdx = numericIdx(:);
pointsIdx = false(size(points, 1), 1);
pointsIdx(numericIdx) = true;
end

%--------------------------------------------------------------------------
function tf = isSimMode()
tf = isempty(coder.target);
end
