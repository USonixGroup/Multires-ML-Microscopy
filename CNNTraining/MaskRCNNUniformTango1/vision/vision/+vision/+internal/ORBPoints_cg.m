%ORBPoints_cg Object used during codegen instead of ORBPoints
%
%   ORBPoints_cg replaces ORBPoints during codegen.

% Copyright 2018-2024 The MathWorks, Inc.

%#codegen
classdef ORBPoints_cg < vision.internal.ORBPointsImpl
    
    methods (Access='public')
        function this = ORBPoints_cg(varargin)
            this = this@vision.internal.ORBPointsImpl(varargin{:});
        end
        
    end
    
    methods (Access='public')
        
        %-------------------------------------------------------------------
        % Returns feature points at specified indices. Indexing operations
        % that exceed the dimensions of the object will error out.
        %-------------------------------------------------------------------
        function obj = select(this, idx)
            
            if islogical(idx)
                validateattributes(idx, {'logical'}, {'vector'}, 'ORBPoints'); %#ok<*EMCA>
            else
                validateattributes(idx, {'numeric'}, {'vector', 'integer', 'positive', 'finite'}, 'ORBPoints'); %#ok<*EMCA>
            end
            
            location        = this.pLocation(idx,:);
            metric          = this.pMetric(idx,:);
            scale           = this.pScale(idx,:);
            orientation     = this.pOrientation(idx,:);
            % The following properties are meta scalars and not subject to
            % the indexing.
            numLevels       = this.pNumLevels;
            scaleFactor     = this.pScaleFactor;
            obj = vision.internal.ORBPoints_cg(location, 'Metric', metric,...
                'Scale',scale, 'Orientation', orientation, ...
                'NumLevels',  numLevels,...
                'ScaleFactor',scaleFactor);
            
        end
    end
end

% LocalWords:  OpenCV


