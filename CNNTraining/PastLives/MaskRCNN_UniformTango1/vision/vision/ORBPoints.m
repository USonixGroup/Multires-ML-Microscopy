classdef ORBPoints < vision.internal.FeaturePoints & vision.internal.ORBPointsImpl

    %  Copyright 2018-2023 The MathWorks, Inc.
    
    methods(Access=public, Static, Hidden)
        function name = matlabCodegenRedirect(~)
            name = 'vision.internal.ORBPoints_cg';
        end
    end
    
    methods (Access='public')
        function this = ORBPoints(varargin)
            this = this@vision.internal.ORBPointsImpl(varargin{:});
        end
        
        %-------------------------------------------------------------------
        function varargout = plot(this, varargin)
            
            nargoutchk(0,1);
            
            this.PlotScaleFactor = 0.5 * single(this.pPatchSize) ; % to plot circle of diameter equal to the patchsize.
            supportsScaleAndOrientation = true;
            
            h = plot@vision.internal.FeaturePoints(this, ...
                supportsScaleAndOrientation, varargin{:});
            
            if nargout == 1
                varargout{1} = h;
            end
        end
    end
    
    methods (Access = 'public', Hidden = true)
        %-------------------------------------------------------------------
        function this = append(this,varargin)
            %append Appends additional ORB points
            indexS = this.Count + 1;
            inputs = ORBPoints(varargin{:});
            checkCompatibility(this, inputs);
            indexE = indexS + size(inputs.Location,1) - 1;
            
            this.pLocation(indexS:indexE, 1:2)     = inputs.Location;
            this.pScale(indexS:indexE, 1)          = inputs.Scale;
            this.pMetric(indexS:indexE, 1)         = inputs.Metric;
            this.pOrientation(indexS:indexE, 1)    = inputs.Orientation;
        end
    end
    
    methods (Access='protected')
        %-------------------------------------------------------------------
        % Copy data for subsref. This method is used in subsref
        function this = subsref_data(this, option)
            this = subsref_data@vision.internal.FeaturePoints(this, option);
            
            % Scale and Orientation are Mx1 matrices. When
            % the indices for sub-referencing is a 1-D array, we explicitly
            % specify the size for the second dimension.
            if length(option.subs) == 1
                option.subs{2} = 1;
            end
            
            this.pScale           = subsref(this.pScale, option);
            this.pOrientation     = subsref(this.pOrientation, option);
        end
        
        %-------------------------------------------------------------------
        % Copy data for subsasgn. This method is used in subsasgn
        function this = subsasgn_data(this, option, in)
            this = subsasgn_data@vision.internal.FeaturePoints(this, option, in);

            if isempty(in)
                this.pScale = ...
                    subsasgn(this.pScale, option, in);
                this.pOrientation = ...
                    subsasgn(this.pOrientation, option, in);
            else
                % Scale and orientation are both Mx1 vectors
                if length(option.subs) == 1
                    option.subs{2} = 1;
                end
            
                this.pScale = ...
                    subsasgn(this.pScale, option, in.pScale);
                this.pOrientation = ...
                    subsasgn(this.pOrientation, option, in.pOrientation);
            end
        end
        %------------------------------------------------------------------
        % Concatenate data for vertcat. This method is used in vertcat.
        function obj = vertcatObj(varargin)
            obj = varargin{1};
            for i=2:nargin
                obj.pNumLevels    = obj.checkCompatibility(varargin{i});
                obj.pLocation    = [obj.pLocation; varargin{i}.pLocation];
                obj.pMetric      = [obj.pMetric  ; varargin{i}.pMetric];
                obj.pScale       = [obj.pScale   ; varargin{i}.pScale];
                obj.pOrientation = [obj.pOrientation ; varargin{i}.pOrientation];
            end
        end
    end
    
    methods (Access='private')
        function NumLevels = checkCompatibility(this, scndObj)
            if (this.pScaleFactor ~= scndObj.pScaleFactor)
                error(message('vision:points:diffScaleFactor'));
            else
                NumLevels = max(this.pNumLevels, scndObj.pNumLevels);
            end
        end
    end
end

% LocalWords: OpenCV
