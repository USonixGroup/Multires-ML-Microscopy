% Copyright 2023 The MathWorks, Inc.

classdef cameraIntrinsicsKBImpl < vision.internal.cameraIntrinsicsBase
    properties (SetAccess='protected', GetAccess='public')
        
        %FocalLength Focal length in pixels.
        %   FocalLength is a vector [fx, fy].  fx = F * sx and fy = F * sy,
        %   where F is the focal length in world units, typically
        %   millimeters, and [sx, sy] are the number of pixels per world
        %   unit in the x and y direction respectively. Thus, fx and fy are
        %   in pixels.
        FocalLength
        
        %PrincipalPoint Optical center of the camera.
        %   PrincipalPoint is a vector [cx, cy], containing the
        %   coordinates of the optical center of the camera in pixels.
        PrincipalPoint
        
        %ImageSize Image size produced by the camera.
        %   ImageSize is a vector [mrows, ncols] corresponding to the image
        %   size produced by the camera.
        ImageSize
        
        %DistortionCoefficients A 1-by-4 distortion coefficients vector.
        DistortionCoefficients

        %K A 3-by-3 projection matrix.
        %   K is of the form [fx 0 cx; 0 fy cy; 0 0 1], where [cx, cy] are
        %   the coordinates of the optical center (the principal point) in
        %   pixels. fx = F * sx and fy = F * sy, where F is the focal
        %   length in world units, typically millimeters, and [sx, sy] are
        %   the number of pixels per world unit in the x and y direction
        %   respectively. Thus, fx and fy are in pixels.
        K;
    end

    properties (SetAccess='private', GetAccess='public', Hidden)
        
        % Skew Skew is always 0 in cameraIntrinsicsKB class. This property
        % exists for maintaining design parity with the cameraIntrinsics class.
        Skew = 0;
    end
    
    properties(Access = protected, Hidden)

        UndistortMap
    end

    methods
        
        function this = cameraIntrinsicsKBImpl(focalLength, principalPoint,...
                                               imageSize, distortionCoefficients)
            arguments
                focalLength {validateFocalLength}
                principalPoint {validatePrincipalPoint}
                imageSize {validateImageSize}
                distortionCoefficients {validateDistortionCoefficients}
            end

            this.FocalLength = focalLength;
            this.PrincipalPoint = principalPoint; 
            this.K = this.buildIntrinsicMatrix();

            this.ImageSize = imageSize;
            this.DistortionCoefficients = distortionCoefficients;
            
            % Initialize the undistort map property. It must be done in the
            % constructor in order to successfully cache the undistortion
            % map.
            this.UndistortMap = vision.internal.calibration.ImageTransformerKB();
        end
    end

    %----------------------------------------------------------------------
    %
    %----------------------------------------------------------------------
    methods (Access='private', Hidden)
        
        %------------------------------------------------------------------
        function intrinsicMat = buildIntrinsicMatrix(this)
            intrinsicMat = ...
                [this.FocalLength(1), 0                  , this.PrincipalPoint(1); ...
                0                   , this.FocalLength(2), this.PrincipalPoint(2); ...
                0                   , 0                  , 1];
        end
    end

    %----------------------------------------------------------------------
    % Implementation of abstract methods.
    %----------------------------------------------------------------------
    methods(Hidden)
        
        %------------------------------------------------------------------
        function updateUndistortMap(this, I, outputView, xBounds, yBounds)
            this.UndistortMap.update(I, this.K, this.DistortionCoefficients, ...
                    outputView, xBounds, yBounds);
        end
        
        %------------------------------------------------------------------
        % Apply distortion to a set of points
        %------------------------------------------------------------------
        function distortedPoints = distortPoints(this, points, ~)

            % in matlab use the builtin c++ function
            distortedPoints = visionDistortPointsKB(points, this.K, ...
                this.DistortionCoefficients);
        end

        %------------------------------------------------------------------
        function undistortedMask = computeUndistortedMask(this, xBounds, ...
                yBounds, imageSize, outputView)

            mask = ones(imageSize, 'uint8');
            fillValuesMask = cast(0, 'uint8');
            
            myMap = vision.internal.calibration.ImageTransformerKB;
            myMap.update(mask, this.K, this.DistortionCoefficients, ...
                outputView, xBounds, yBounds);
            
            undistortedMask = myMap.transformImage(mask, 'nearest', ...
                fillValuesMask);
        end
    end
end

%--------------------------------------------------------------------------
function validateFocalLength(focalLength)
    
    filename = 'cameraIntrinsicsKB';
    vision.internal.inputValidation.validateFocalLength(focalLength, filename);
end

%--------------------------------------------------------------------------
function validatePrincipalPoint(principalPoint)
    
    filename = 'cameraIntrinsicsKB';
    vision.internal.inputValidation.validatePrincipalPoint(principalPoint, filename);
end

%--------------------------------------------------------------------------
function validateImageSize(imageSize)
    
    filename = 'cameraIntrinsicsKB';
    vision.internal.inputValidation.validateImageSize(imageSize, filename);
end

%--------------------------------------------------------------------------
function validateDistortionCoefficients(distortionCoefficients)

    filename = 'cameraIntrinsicsKB';
    argName = 'DistortionCoefficients';
    validTypes = {'double', 'single'};
    validateattributes(distortionCoefficients, validTypes,...
        {'vector', 'real', 'nonsparse', 'finite', 'numel', 4},...
        filename, argName);
end