% Copyright 2023 MathWorks, Inc.

classdef cameraIntrinsicsKB < vision.internal.cameraIntrinsicsKBImpl

    properties (Access=protected, Hidden)
       Version = ver('vision');
    end

    methods (Access = private)
        %------------------------------------------------------------------
        % Constructor
        %------------------------------------------------------------------
        function this = cameraIntrinsicsKB(varargin)
           this = this@vision.internal.cameraIntrinsicsKBImpl(varargin{:});
        end        
    end
    
    %----------------------------------------------------------------------
    % Static methods
    %----------------------------------------------------------------------
    methods(Static, Hidden)
        
        %------------------------------------------------------------------
        function this = construct(varargin)
            this = cameraIntrinsicsKB(varargin{:});
        end

        %------------------------------------------------------------------
        function this = loadobj(that)
            this = cameraIntrinsicsKB(...
                that.FocalLength,...
                that.PrincipalPoint,...
                that.ImageSize,...
                that.DistortionCoefficients);
        end
    end
    
    methods (Hidden)
       
        %------------------------------------------------------------------
        function undistortedPoints = undistortPointsImpl(this, points) 
            options = optimset('Display', 'off');
            undistortedPoints = ...
                lscftsh(@this.distortPoints, points, [], points, [], [], options);
        end

        %------------------------------------------------------------------
        function that = saveobj(this)
            that.FocalLength            = this.FocalLength;
            that.PrincipalPoint         = this.PrincipalPoint;
            that.ImageSize              = this.ImageSize;
            that.DistortionCoefficients = this.DistortionCoefficients;
            that.Version                = this.Version;
        end
    end
end