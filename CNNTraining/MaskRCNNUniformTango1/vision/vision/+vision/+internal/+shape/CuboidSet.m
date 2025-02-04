classdef CuboidSet < vision.internal.shape.ShapeSet
    % This class is used to show a set of cuboids on a figure.
    %
    % This function is for internal use only and is likely to change in
    % future releases.
    
    % Copyright 2020 The MathWorks, Inc.
    
    methods
        function this = CuboidSet() 
        end
        
        function show(this,bbox,params)
            % Show a set of cuboids. bbox is an M-by-9 matrix of cuboids
            % specified as [xc yc zc w h l rx ry rz]. Cuboid coordinates
            % are specified in spatial coordinates.
            for ii = 1:size(bbox,1)
                this.Shape{ii}.CenteredPosition = bbox(ii,1:6);
                this.Shape{ii}.RotationAngle = bbox(ii,7:9);
                vision.internal.shape.ShapeSet.setCommonParameters(this.Shape{ii},params,ii);
            end
        end
        
    end
    
    methods(Static)
        
        function set = makeSet()
            % Return an instance of CuboidSet.
            set = vision.internal.shape.CuboidSet();
        end
        
        function set = allocate(n)
            % Allocate a set of N shapes.
            
            obj = cell(1,n);
            defaultParameters = vision.internal.shape.ShapeSet.defaultParameters();
            for ii = 1:n
                obj{ii} = images.roi.Cuboid(...
                    'Rotatable','none',...
                    'Tag',vision.internal.shape.ShapeSet.uniqueTag(),...
                    defaultParameters{:});
            end
            set = vision.internal.shape.CuboidSet();
            set.Shape = obj;
        end
    end
    
end