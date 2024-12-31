%   cameraIntrinsics Array Class Hold the data of array of cameraIntrinsics
%   for code generation

%#codegen

% Copyright 2021 The MathWorks, Inc.

classdef(Hidden) cameraIntrinsicsArray

    properties
        FocalLength;
        PrincipalPoint;
        ImageSize;
        RadialDistortion;
        TangentialDistortion;
        Skew;
        IntrinsicMatrix;
    end

    methods
        %------------------------------------------------------------------
        % Constructor
        %------------------------------------------------------------------
        function this = cameraIntrinsicsArray(varargin)
            narginchk(1,1);

            obj = varargin{1};
            this.FocalLength = obj.FocalLength;
            this.PrincipalPoint = obj.PrincipalPoint;
            this.ImageSize = obj.ImageSize;
            this.RadialDistortion = obj.RadialDistortion;
            this.TangentialDistortion = obj.TangentialDistortion;
            this.Skew = obj.Skew;
            this.IntrinsicMatrix = obj.IntrinsicMatrix;
        end
    end
end
