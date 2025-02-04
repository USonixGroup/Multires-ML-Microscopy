classdef RectangleSet < vision.internal.shape.ShapeSet
    % This class is used to show a set of rectangles on a figure.
    %
    % This function is for internal use only and is likely to change in
    % future releases.
    
    % Copyright 2020-2021 The MathWorks, Inc.
    
    methods
        function this = RectangleSet()
        end
        
        function show(this,bbox,params)
            % Show a set of rectangles. bbox is an M-by-4 array of [x y w
            % h] axis-aligned boxes. [x y w h] are in spatial coordinates.
            for ii = 1:size(bbox,1)
                this.Shape{ii}.Position = bbox(ii,1:4);
                vision.internal.shape.ShapeSet.setCommonParameters(this.Shape{ii},params,ii);
            end
        end
        
    end
    
    methods(Static)
        
        function set = makeSet()
            % Return an instance of RectangleSet.
            set = vision.internal.shape.RectangleSet();
        end
        
        function set = allocate(N)
            % Allocate a set of N shapes.
            
            obj = cell(1,N);
            defaultParameters = vision.internal.shape.ShapeSet.defaultParameters();
            for ii = 1:N
                obj{ii} = images.roi.Rectangle(...
                    'Rotatable',false,...
                    'FaceSelectable',false,...
                    'Tag',vision.internal.shape.ShapeSet.uniqueTag(),...
                    defaultParameters{:});
            end
            set = vision.internal.shape.RectangleSet();
            set.Shape = obj;
        end
    end
end
