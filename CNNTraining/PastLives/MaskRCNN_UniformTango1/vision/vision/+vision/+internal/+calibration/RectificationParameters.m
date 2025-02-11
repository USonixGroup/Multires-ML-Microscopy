% RectificationParameters Object that stores the stereo rectification parameters
%
% RectificationParameters properties:
%   H1                 - projective transformation for image 1
%   H2                 - projective transformation for image 2
%   R1                 - rectification rotation matrix for camera 1
%   R2                 - rectification rotation matrix for camera 2
%   CameraMatrix1      - camera projection matrix for rectified camera 1
%   CameraMatrix2      - camera projection matrix for rectified camera 2
%   Q                  - 4x4 matrix that maps [x, y, disparity] to [X, Y, Z]
%   XBounds            - the x bounds of output image
%   YBounds            - the y bounds of the output image
%   RectifiedImageSize - image size after rectification
%   Initialized        - true if the rectification parameters have been computed
%
% RectificationParameters methods:
%   needToUpdate - returns true if the rectification parameters need to be updated
%   update       - updates the rectification parameters
%   toStruct     - convert a RectificationParameters object into a struct

% Copyright 2014-2021 MathWorks, Inc.

% References:
%
% G. Bradski and A. Kaehler, "Learning OpenCV : Computer Vision with
% the OpenCV Library," O'Reilly, Sebastopol, CA, 2008.

%#codegen
classdef RectificationParameters < handle
    
    properties (SetAccess=private, GetAccess=public)                
        %H1 A 3x3 matrix containing projective transformation for image 1
        H1
        
        %H2 A 3x3 matrix containing projective transformation for image 2
        H2
        
        %Q A 4x4 matrix that maps [x, y, disparity] to [X, Y, Z]
        Q

        %R1 A 3x3 matrix representing the rectification rotation matrix for the first camera
        R1

        %R2 A 3x3 matrix representing the rectification rotation matrix for the second camera
        R2

        %CameraMatrix1 A 4x3 matrix representing the projection matrix for the rectified first camera
        CameraMatrix1

        %CameraMatrix2 A 4x3 matrix representing the projection matrix for the rectified second camera
        CameraMatrix2
                        
        %XBounds A 2-element vector containing the x bounds of output image
        XBounds = zeros(1, 2);
        
        %YBounds A 2-element vector containing the y bounds of the output image
        YBounds = zeros(1, 2);
        
        %Initialized A flag set to true by the first call to update method
        Initialized = false;
    end
    
    properties(Access=private)
        % OriginalImageSize Image size before rectification
        OriginalImageSize = zeros(1, 2);        
        
        % OutputView 'full' or 'valid'                
        OutputView = 'full';
        
    end
    
    properties(Dependent, SetAccess=private, GetAccess=public)
        % RectifiedImageSize Image size after rectification
        RectifiedImageSize;
    end
    
    methods
        function this = RectificationParameters()
            this.OutputView = 'full';
            this.OutputView = 'valid';
            this.H1 = projective2d();
            this.H2 = projective2d();
            this.Q  = eye(4);
            this.R1 = eye(3);
            this.R2 = eye(3);
            this.CameraMatrix1 = [eye(3); zeros(1, 3)];
            this.CameraMatrix2 = [eye(3); zeros(1, 3)];
        end
        
        %------------------------------------------------------------------
        function paramStruct = toStruct(this)
        % toStruct - convert a RectificationParameters object into a struct
        %   paramStruct = toStruct(obj) returns a struct containing the
        %   rectification parameters.         
            paramStruct.Initialized = this.Initialized;
            paramStruct.H1 = this.H1.T;
            paramStruct.H2 = this.H2.T;
            paramStruct.R1 = this.R1;
            paramStruct.R2 = this.R2;
            paramStruct.CameraMatrix1 = this.CameraMatrix1;
            paramStruct.CameraMatrix2 = this.CameraMatrix2;
            paramStruct.Q  = this.Q;
            paramStruct.XBounds = this.XBounds;
            paramStruct.YBounds = this.YBounds;
            paramStruct.OriginalImageSize = this.OriginalImageSize;
            paramStruct.OutputView = this.OutputView;
        end
        
        %------------------------------------------------------------------
        % Returns true if the object needs to be updated.
        %------------------------------------------------------------------
        function tf = needToUpdate(this, imageSize, outputView)      
        % needToUpdate Check if the rectification parameters need to be updated
        %   tf = needToUpdate(obj, imageSize, outputView) returns true if
        %   the rectification parameters need to be updated, and false
        %   otherwise. obj is a RectificationParameters object. imageSize
        %   is a 2-element vector [height, width] representing the size of
        %   the input image. outputView is a string that determines the
        %   size of the output image. Valid values are 'full' and 'valid'.
            tf = ~ (isequal(imageSize, this.OriginalImageSize) && ...
                strcmp(outputView, this.OutputView));
        end
        
        %------------------------------------------------------------------
        % Update the rectification parameters.
        %------------------------------------------------------------------
        function update(this, imageSize, h1, h2, r1, r2, camMatrix1, camMatrix2, ...
                q, outputView, xBounds, yBounds)
        % UPDATE Update the rectification parameters.
        %   UPDATE(obj, imageSize, h1, h2, r1, r2, camMatrix1, camMatrix2, q, outputView, xBounds, yBounds)
        %   updates the rectification parameters.
            this.Initialized = true;
            this.OriginalImageSize = imageSize;
            this.H1 = h1;
            this.H2 = h2;
            this.R1 = r1;
            this.R2 = r2;
            this.CameraMatrix1 = camMatrix1;
            this.CameraMatrix1 = camMatrix2;
            this.Q  = q;
            this.OutputView = outputView;
            this.XBounds = xBounds;
            this.YBounds = yBounds;
        end
        
        %------------------------------------------------------------------
        % Computes the size of the rectified image
        %------------------------------------------------------------------
        function imageSize = get.RectifiedImageSize(this)
            imageSize = [this.YBounds(2) - this.YBounds(1) + 1, ...
                         this.XBounds(2) - this.XBounds(1) + 1];
        end
            
    end
    
    %----------------------------------------------------------------------
    methods (Static, Hidden)
        function this = loadobj(that) 
            % handle pre-R2015a version, which did not have the
            % Initialize property.
            this = vision.internal.calibration.RectificationParameters;
            if (isprop(that, 'Initialized') && that.Initialized) ||...
                    (~isprop(that, 'Initialized') && ~isempty(that.Q))
                R = eye(3);
                camMatrix = [eye(3); zeros(1, 3)];
                this.update(that.OriginalImageSize, that.H1, that.H2,...
                    R, R, camMatrix, camMatrix, that.Q, that.OutputView, that.XBounds, that.YBounds);
            end
        end
    end
end
