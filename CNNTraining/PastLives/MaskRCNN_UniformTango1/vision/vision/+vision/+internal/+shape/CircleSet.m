classdef CircleSet < vision.internal.shape.ShapeSet
    % This class is used to show a set of circles on a figure.
    %
    % This function is for internal use only and is likely to change in
    % future releases.
    
    % Copyright 2020 The MathWorks, Inc.
    
    methods
        function this = CircleSet()
        end
        
        function show(this,pos,params)
            % Show a set of circles. pos is an M-by-3 matrix of [xc yc
            % radius] circles. [xc yc r] are in spatial coordinates.
            for ii = 1:size(pos,1)
                this.Shape{ii}.Center = pos(ii,1:2);
                this.Shape{ii}.Radius = pos(ii,3);
                vision.internal.shape.ShapeSet.setCommonParameters(this.Shape{ii},params,ii);
            end
        end
        
    end
    
    methods(Static)
        
        function set = makeSet()
            % Return an instance of CircleSet.
            set = vision.internal.shape.CircleSet();
        end
        
        function set = allocate(n)
            % Allocate a set of N shapes.
            
            obj = cell(1,n);
            defaultParameters = vision.internal.shape.ShapeSet.defaultParameters();
            for ii = 1:n
                obj{ii} = images.roi.Circle(...
                    'FaceSelectable',false,...
                    'Tag',vision.internal.shape.ShapeSet.uniqueTag(),...
                    defaultParameters{:});
            end
            set = vision.internal.shape.CircleSet();
            set.Shape = obj;
        end
    end
    
end
