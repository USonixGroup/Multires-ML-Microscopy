
% Copyright 2016-2023 MathWorks, Inc.

%#codegen

classdef cameraIntrinsics < vision.internal.cameraIntrinsicsImpl

    properties (Access=protected, Hidden)
       Version = ver('vision');
    end

    methods
        %------------------------------------------------------------------
        % Constructor
        %------------------------------------------------------------------
        function this = cameraIntrinsics(varargin)
           this = this@vision.internal.cameraIntrinsicsImpl(varargin{:});
        end        
    end
    
    %----------------------------------------------------------------------
    % Static methods
    %----------------------------------------------------------------------
    methods(Static, Hidden)

        function this = loadobj(that)
            this = cameraIntrinsics(...
                that.FocalLength,...
                that.PrincipalPoint,...
                that.ImageSize,...
                'RadialDistortion', that.RadialDistortion,...
                'TangentialDistortion', that.TangentialDistortion,...
                'Skew', that.Skew);
        end
        
    end
    
    %----------------------------------------------------------------------
    % saveobj is implemented to ensure compatibility across releases by
    % converting the class to a struct prior to saving it. It also contains
    % a version number, which can be used to customize the loading process.
    methods (Hidden)
       
        function that = saveobj(this)
            that.FocalLength          = this.FocalLength;
            that.PrincipalPoint       = this.PrincipalPoint;
            that.ImageSize            = this.ImageSize;
            that.RadialDistortion     = this.RadialDistortion;
            that.TangentialDistortion = this.TangentialDistortion;
            that.Skew                 = this.Skew;
            that.Version              = this.Version;
        end 
    end
    
    methods(Access=public, Static, Hidden)
        %----------------------------------------------------------------------
        function name = matlabCodegenRedirect(~)
            name = 'vision.internal.codegen.cameraIntrinsics';
        end
    end
end