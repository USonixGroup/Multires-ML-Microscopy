%SIFTPoints_cg Object used during codegen instead of SIFTPoints
%
%   SIFTPoints_cg replaces SIFTPoints during codegen.

% Copyright 2020-2024 The MathWorks, Inc.

%#codegen
classdef SIFTPoints_cg < vision.internal.SIFTPointsImpl

   methods (Access='public')
       function this = SIFTPoints_cg(varargin)
           this = this@vision.internal.SIFTPointsImpl(varargin{:});
       end            
   end
              
   methods (Access='public')

       %-------------------------------------------------------------------
       % Returns feature points at specified indices. Indexing operations
       % that exceed the dimensions of the object will error out.
       %-------------------------------------------------------------------             
       function obj = select(this, idx)
           
           if islogical(idx)
               validateattributes(idx, {'logical'}, {'vector'}, 'SIFTPoints');
           else
               validateattributes(idx, {'numeric'}, {'vector', 'integer', 'positive', 'finite'}, 'SIFTPoints');
           end
                      
           location        = this.pLocation(idx,:);
           metric          = this.pMetric(idx,:);
           scale           = this.pScale(idx,:);
           orientation     = this.pOrientation(idx,:);
           octave          = this.pOctave(idx,:);
           layer           = this.pLayer(idx,:);
           
           obj = vision.internal.SIFTPoints_cg(location,'Metric',metric,...
               'Scale',scale, 'Orientation', orientation, 'Octave', octave, ...
               'Layer', layer);
           
       end
   end    
end

% LocalWords:  OpenCV


