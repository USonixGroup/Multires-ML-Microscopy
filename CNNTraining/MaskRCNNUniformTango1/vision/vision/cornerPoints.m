classdef cornerPoints < vision.internal.FeaturePoints

    % Copyright 2012-2023 The MathWorks, Inc.

    methods(Access=public, Static, Hidden)
        function name = matlabCodegenRedirect(~)
            name = 'vision.internal.cornerPoints_cg';
        end
    end

    methods(Access='public')
       
       function this = cornerPoints(varargin)         
           this = this@vision.internal.FeaturePoints(varargin{:});
       end
       
       %-------------------------------------------------------------------
       function varargout = plot(this, varargin)

           nargoutchk(0,1);
           
           supportsScaleAndOrientation = false;
           
           h = plot@vision.internal.FeaturePoints(this, ...
               supportsScaleAndOrientation, varargin{:});
          
           if nargout == 1
               varargout{1} = h;
           end
       end
       
       %-------------------------------------------------------------------
       function this = selectStrongest(this, N)

           this = selectStrongest@vision.internal.FeaturePoints(this, N);
       end
       
       %-------------------------------------------------------------------
       function that = selectUniform(this, N, imageSize)
            
            that = selectUniform@vision.internal.FeaturePoints(this, N, imageSize);
       end
       
       %-------------------------------------------------------------------
       function out = gather(this)
           
           if isa(this.Location, 'gpuArray')
               % only call gather if input has a gpuArray
               location = gather(this.Location);
               metric   = gather(this.Metric);
               
               out = cornerPoints(location, 'Metric', metric);
           else
               out = this;
           end
       end
   end
   
end

