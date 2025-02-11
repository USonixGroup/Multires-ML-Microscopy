classdef KAZEPoints < vision.internal.FeaturePoints & vision.internal.KAZEPointsImpl

    % Copyright 2017-2023 The MathWorks, Inc.

    methods(Access=public, Static, Hidden)
       function name = matlabCodegenRedirect(~)
         name = 'vision.internal.KAZEPoints_cg';
       end
    end
    
    methods (Access='public')
        function this = KAZEPoints(varargin)
            this = this@vision.internal.KAZEPointsImpl(varargin{:});
        end

       %-------------------------------------------------------------------
       function varargout = plot(this, varargin)

           nargoutchk(0,1);

           supportsScaleAndOrientation = true;
           this.PlotScaleFactor = 6;

           h = plot@vision.internal.FeaturePoints(this, ...
               supportsScaleAndOrientation, varargin{:});

           if nargout == 1
               varargout{1} = h;
           end

       end
    end

	methods (Access='public', Hidden=true)
       %-------------------------------------------------------------------
       function this = append(this,varargin)
           %append Appends additional KAZE points
           indexS = this.Count + 1;
           inputs = KAZEPoints(varargin{:});
           checkCompatibility(this, inputs);
           indexE = indexS + size(inputs.Location,1) - 1;
           
           this.pLocation(indexS:indexE, 1:2)     = inputs.Location;
           this.pScale(indexS:indexE, 1)          = inputs.Scale;
           this.pMetric(indexS:indexE, 1)         = inputs.Metric;
           this.pOrientation(indexS:indexE, 1)    = inputs.Orientation;
           this.pLayerID(indexS:indexE, 1)        = inputs.getLayerID;
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

           this.pScale           = subsref(this.pScale,option);
           this.pOrientation     = subsref(this.pOrientation,option);
           this.pLayerID         = subsref(this.pLayerID,option);
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
               this.pLayerID = ...
                   subsasgn(this.pLayerID, option, in);
           else
               % Scale, Orientation and LayerID are Mx1 vectors
               if length(option.subs) == 1
                   option.subs{2} = 1;
               end
           
               this.pScale = ...
                   subsasgn(this.pScale, option, in.pScale);
               this.pOrientation = ...
                   subsasgn(this.pOrientation, option, in.pOrientation);
               this.pLayerID = ...
                   subsasgn(this.pLayerID, option, in.pLayerID);
           end
       end
       %------------------------------------------------------------------
       % Concatenate data for vertcat. This method is used in vertcat.
       %------------------------------------------------------------------
       function obj = vertcatObj(varargin)
           obj = varargin{1};

           for i=2:nargin
               obj.checkCompatibility(varargin{i});
               obj.pLocation    = [obj.pLocation; varargin{i}.pLocation];
               obj.pMetric      = [obj.pMetric  ; varargin{i}.pMetric];
               obj.pScale       = [obj.pScale   ; varargin{i}.pScale];
               obj.pOrientation = [obj.pOrientation ; varargin{i}.pOrientation];
               obj.pLayerID     = [obj.pLayerID; varargin{i}.pLayerID];
           end
       end
   end

   methods (Access='private')
       function checkCompatibility(this, scndObj)
           pass = true;
           errMsg = 'KAZEPoints: Cannot concatenate because points are detected with differences in the following setting(s):';
           if ~strcmp(this.pDiffusion,scndObj.pDiffusion)
               errMsg = [errMsg, '\n- diffusion method'];
               pass = false;
           end
           if(this.pNumOctaves ~= scndObj.pNumOctaves)
               errMsg = [errMsg, '\n- number of octave'];
               pass = false;
           end
           if(this.pNumScaleLevels ~= scndObj.pNumScaleLevels)
               errMsg = [errMsg, '\n- number of scale levels\n'];
               pass = false;
           end
           if ~pass
               error([errMsg,'%s'],'');
           end
       end
	end
end

% LocalWords: OpenCV
