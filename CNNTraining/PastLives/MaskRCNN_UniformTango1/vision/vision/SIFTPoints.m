classdef SIFTPoints < vision.internal.SIFTPointsImpl & vision.internal.FeaturePoints

    % Copyright 2020-2023 The MathWorks, Inc.
   
   methods(Access=public, Static, Hidden)
       function name = matlabCodegenRedirect(~)
         name = 'vision.internal.SIFTPoints_cg';
       end
   end
   
   %-----------------------------------------------------------------------
   methods (Access='public')
       
       function this = SIFTPoints(varargin)                      
           this = this@vision.internal.SIFTPointsImpl(varargin{:});                             
       end  
              
       %-------------------------------------------------------------------
       function varargout = plot(this, varargin)
           
           nargoutchk(0,1);           
           
           supportsScaleAndOrientation = true;
           
           this.PlotScaleFactor = 8;
           
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
           %append Appends additional SIFT points
           
           indexS = this.Count + 1;
           inputs = parseInputs(this, varargin{:});
           indexE = indexS + size(inputs.Location,1) - 1;
           
           this.pLocation(indexS:indexE, 1:2)     = inputs.Location;
           this.pScale(indexS:indexE, 1)          = inputs.Scale;
           this.pMetric(indexS:indexE, 1)         = inputs.Metric;
           this.pOrientation(indexS:indexE, 1)    = inputs.Orientation;
           this.pOctave(indexS:indexE, 1)         = inputs.Octave;
           this.pLayer(indexS:indexE, 1)          = inputs.Layer;
       end
   end
   
   methods (Access='protected')
       %-------------------------------------------------------------------
       % Copy data for subsref. This method is used in subsref
       function this = subsref_data(this, option)
           this = subsref_data@vision.internal.FeaturePoints(this, option);
           
           % Scale, Orientation, Octave and Layer are Mx1 matrices. 
           % When the indices for sub-referencing is a 1-D array, we 
           % explicitly specify the size for the second dimension.
           if length(option.subs) == 1
               option.subs{2} = 1;
           end
           
           this.pScale           = subsref(this.pScale,option);
           this.pOrientation     = subsref(this.pOrientation,option);
           this.pOctave          = subsref(this.pOctave,option);
           this.pLayer           = subsref(this.pLayer,option);
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
               this.pOctave = ...
                   subsasgn(this.pOctave, option, in);
               this.pLayer = ...
                   subsasgn(this.pLayer, option, in);
           else
               % Scale, Orientation, Octave and Layer are Mx1 matrices
               if length(option.subs) == 1
                   option.subs{2} = 1;
               end
           
               this.pScale = ...
                   subsasgn(this.pScale, option, in.pScale);
               this.pOrientation = ...
                   subsasgn(this.pOrientation, option, in.pOrientation);
               this.pOctave = ...
                   subsasgn(this.pOctave, option, in.pOctave);
               this.pLayer = ...
                   subsasgn(this.pLayer, option, in.pLayer);
           end
       end
       %------------------------------------------------------------------
       % Concatenate data for vertcat. This method is used in vertcat.
       %------------------------------------------------------------------
       function obj = vertcatObj(varargin)
           obj = varargin{1};
           
           for i=2:nargin
               obj.pLocation    = [obj.pLocation    ; varargin{i}.pLocation   ];
               obj.pMetric      = [obj.pMetric      ; varargin{i}.pMetric     ];
               obj.pScale       = [obj.pScale       ; varargin{i}.pScale      ];
               obj.pOrientation = [obj.pOrientation ; varargin{i}.pOrientation];
               obj.pOctave      = [obj.pOctave      ; varargin{i}.pOctave     ];
               obj.pLayer       = [obj.pLayer       ; varargin{i}.pLayer      ];
           end
       end
   end
   
end

% LocalWords:  OpenCV
