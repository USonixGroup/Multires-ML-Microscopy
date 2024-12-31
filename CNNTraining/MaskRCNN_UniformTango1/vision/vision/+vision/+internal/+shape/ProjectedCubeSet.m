classdef ProjectedCubeSet < vision.internal.shape.ShapeSet
    % This class is used to show a set of projected cuboids on a figure.
    %
    % This function is for internal use only and is likely to change in
    % future releases.
    
    % Copyright 2022 The MathWorks, Inc.
    
    methods
        function this = ProjectedCubeSet()
        end
        
        function show(this,bbox,params)
            % Show a set of projected cuboids. bbox is either M-by-8 array 
            % or 8-by-2-by-M array for M projected cuboids in rectangles or
            % vertices representation.
            
            if size(bbox, 2)==8
                % [x1 y1 w1 h1 x2 y2 w2 h2] rectangles representation.
                for ii = 1:size(bbox, 1)
                    this.Shape{ii}.Position = bbox(ii,1:8);
                    vision.internal.shape.ShapeSet.setCommonParameters(this.Shape{ii},params,ii);
                end
            else
                % Vertices representation.
                for ii = 1:size(bbox, 3)
                    this.Shape{ii}.Position = bbox(:,:,ii);
                    vision.internal.shape.ShapeSet.setCommonParameters(this.Shape{ii},params,ii);
                end
            end

        end
        
    end
    
    methods(Static)
        
        function set = makeSet()
            % Return an instance of ProjectedCubeSet.
            set = vision.internal.shape.ProjectedCubeSet();
        end
        
        function set = allocate(N)
            % Allocate a set of N shapes.
            
            obj = cell(1,N);
            defaultParameters = vision.internal.shape.ShapeSet.defaultParameters();
            for ii = 1:N
                obj{ii} = vision.roi.ProjectedCuboid(...
                    'Tag',vision.internal.shape.ShapeSet.uniqueTag(),...
                    defaultParameters{:});
            end
            set = vision.internal.shape.ProjectedCubeSet();
            set.Shape = obj;
        end
    end
end
