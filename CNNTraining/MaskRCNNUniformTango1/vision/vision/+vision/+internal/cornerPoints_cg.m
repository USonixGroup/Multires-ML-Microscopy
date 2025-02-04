%cornerPoints_cg Object used during codegen instead of cornerPoints
%
%   cornerPoints_cg replaces cornerPoints during codegen.

% Copyright 2012-2024 The MathWorks, Inc.

%#codegen
%#ok<*EMCA>

classdef cornerPoints_cg < vision.internal.FeaturePointsImpl
    
   methods (Access='public')
       
       function this = cornerPoints_cg(varargin)
           %cornerPoints constructor
           this = this@vision.internal.FeaturePointsImpl(varargin{:});
       end            
   end
   
   methods
       %-------------------------------------------------------------------
       % Returns feature points at specified indices. Indexing operations
       % that exceed the dimensions of the object will error out.
       %-------------------------------------------------------------------             
       function obj = select(this, idx)
           
           if islogical(idx)
               validateattributes(idx, {'logical'}, {'vector'}, 'SURFPoints'); %#ok<*EMCA>
           else               
               validateattributes(idx, {'numeric'}, {'vector', 'integer', 'positive', 'finite'}, 'SURFPoints'); %#ok<*EMCA>
           end
                      
           obj = vision.internal.cornerPoints_cg(this.pLocation(idx,:),...
               'Metric', this.pMetric(idx,:));                      
       end
   end  
end

