classdef EllipseSet < vision.internal.shape.ShapeSet
    % This class is used to show a set of ellipses on a figure.
    %
    % This function is for internal use only and is likely to change in
    % future releases.
    
    % Copyright 2023 The MathWorks, Inc.
    
    methods
        function this = EllipseSet()
        end
        
        function show(this,pos,params)
            % Show a set of ellipses. pos is an M-by-5 matrix of [xc yc
            % major minor yaw]. yaw is a rotation angle in degrees 
            % around the center of the ROI, specified as a nonnegative 
            % numeric scalar. The angle is measured in degrees in a 
            % clockwise direction. 
            % [xc yc] are spatial coordinates. 
            % [major minor] are floating point values.

            % Negate the rotation angle as images.roi.EllipseSet uses the
            % opposite rotation convention.
            theta = -pos(:,end);

            for ii = 1:size(pos,1)
                this.Shape{ii}.Center = pos(ii,1:2);
                this.Shape{ii}.SemiAxes = 0.5*pos(ii,3:4);
                this.Shape{ii}.RotationAngle = theta(ii,end);
                vision.internal.shape.ShapeSet.setCommonParameters(this.Shape{ii},params,ii);
            end
        end
        
    end
    
    methods(Static)
        
        function set = makeSet()
            % Return an instance of EllipseSet.
            set = vision.internal.shape.EllipseSet();
        end
        
        function set = allocate(n)
            % Allocate a set of N shapes.
            
            obj = cell(1,n);
            defaultParameters = vision.internal.shape.ShapeSet.defaultParameters();
            for ii = 1:n
                obj{ii} = images.roi.Ellipse(...
                    'FaceSelectable',false,...
                    'Tag',vision.internal.shape.ShapeSet.uniqueTag(),...
                    defaultParameters{:});
            end
            set = vision.internal.shape.EllipseSet();
            set.Shape = obj;
        end
    end
    
end
