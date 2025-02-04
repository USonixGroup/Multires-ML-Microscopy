%KAZEPoints_cg Object used during codegen instead of KAZEPoints
%
%   KAZEPoints_cg replaces KAZEPoints during codegen.

% Copyright 2017-2024 The MathWorks, Inc.

%#codegen
classdef KAZEPoints_cg < vision.internal.KAZEPointsImpl

   methods (Access='public')
       function this = KAZEPoints_cg(varargin)
           this = this@vision.internal.KAZEPointsImpl(varargin{:});
       end  
                         
   end
              
   methods (Access='public')

       %-------------------------------------------------------------------
       % Returns feature points at specified indices. Indexing operations
       % that exceed the dimensions of the object will error out.
       %-------------------------------------------------------------------             
       function obj = select(this, idx)
           
           if islogical(idx)
               validateattributes(idx, {'logical'}, {'vector'}, 'KAZEPoints'); %#ok<*EMCA>
           else
               validateattributes(idx, {'numeric'}, {'vector', 'integer', 'positive', 'finite'}, 'KAZEPoints'); %#ok<*EMCA>
           end
                      
           location        = this.pLocation(idx,:);
           metric          = this.pMetric(idx,:);
           scale           = this.pScale(idx,:);
           orientation     = this.pOrientation(idx,:);
           layerID         = this.pLayerID(idx,:);
           % The following properties are meta scalars and not subject to
           % the indexing.
           diffusion       = this.pDiffusion;
           numOctaves      = this.pNumOctaves;
           numScaleLevels  = this.pNumScaleLevels;
           
           obj = vision.internal.KAZEPoints_cg(location,'Metric',metric,...
               'Scale',scale, 'Orientation', orientation, ...
               'Diffusion', diffusion, ...
               'NumOctaves', numOctaves,...
               'NumScaleLevels', numScaleLevels);
           obj.pLayerID = layerID;
       end
   end    
end

% LocalWords:  OpenCV


