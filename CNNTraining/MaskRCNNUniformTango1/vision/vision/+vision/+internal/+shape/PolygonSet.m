classdef PolygonSet < vision.internal.shape.ShapeSet
    % This class is used to show a set of polygons on a figure.
    %
    % This function is for internal use only and is likely to change in
    % future releases.
    
    % Copyright 2020 The MathWorks, Inc.
    
    methods
        function this = PolygonSet() 
        end
        
        function show(this,vertices,params)
            % Show a set of polygons. Polygon vertices are specified as a
            % cell array of M-by-2 matrices of [x y] vertex locations.
            for ii = 1:size(vertices,1)
                this.Shape{ii}.Position = vertices{ii};
                vision.internal.shape.ShapeSet.setCommonParameters(this.Shape{ii},params,ii);
            end
        end
        
    end
    
    methods(Static)
        
        function set = makeSet()
            set = vision.internal.shape.PolygonSet();
        end
        
        function set = allocate(n)
            obj = cell(1,n);
            defaultParameters = vision.internal.shape.ShapeSet.defaultParameters();
            for ii = 1:n
                obj{ii} = images.roi.Polygon(...
                    'FaceSelectable',false,...
                    'Tag',vision.internal.shape.ShapeSet.uniqueTag(),...
                    defaultParameters{:});
            end
            set = vision.internal.shape.PolygonSet();
            set.Shape = obj;
        end
    end
    
end
