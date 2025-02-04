%   cameraIntrinsicsImpl Common implementation for cameraIntrinsics
%
%   This class is for internal use only.
%
%   This class implements the core functionality of cameraIntrinsics.
%   It is common for both simulation and codegen.

% Copyright 2021-2024 The MathWorks, Inc.

%#codegen
classdef cameraIntrinsicsImpl < vision.internal.cameraIntrinsicsBase
    properties (SetAccess='protected', GetAccess='public')
        
        %FocalLength Focal length in pixels.
        %   FocalLength is a vector [fx, fy].  fx = F * sx and fy = F * sy,
        %   where F is the focal length in world units, typically
        %   millimeters, and [sx, sy] are the number of pixels per world
        %   unit in the x and y direction respectively. Thus, fx and fy are
        %   in pixels.
        FocalLength;
        
        %PrincipalPoint Optical center of the camera.
        %   PrincipalPoint is a vector [cx, cy], containing the
        %   coordinates of the optical center of the camera in pixels.
        PrincipalPoint;
        
        %ImageSize Image size produced by the camera.
        %   ImageSize is a vector [mrows, ncols] corresponding to the image
        %   size produced by the camera.
        ImageSize;        
        
        %RadialDistortion Radial distortion coefficients.
        %   Radial distortion is a 2-element vector [k1 k2], 3-element vector [k1 k2 k3], 
        %   or a 6-element vector [k1 k2 k3 k4 k5 k6]. Radial lens distortion is caused by light
        %   rays bending more, the farther away they are from the optical center.
        RadialDistortion;

        %TangentialDistortion Tangential distortion coefficients.
        %   TangentialDistortion is a 2-element vector [p1 p2]. Tangential
        %   distortion is caused by the lens not being exactly parallel to
        %   to the image plane.
        TangentialDistortion;
        
        %Skew Represents skew of the camera axes. 
        %   Skew coefficient is non-zero if X and Y axis are not perpendicular. 
        Skew;
        
        %K A 3-by-3 projection matrix.
        %   K is of the form [fx s cx; 0 fy cy; 0 0 1], where
        %   [cx, cy] are the coordinates of the optical center (the
        %   principal point) in pixels and s is the skew parameter which is
        %   0 if the x and y axes are exactly perpendicular. fx = F * sx
        %   and fy = F * sy, where F is the focal length in world units,
        %   typically millimeters, and [sx, sy] are the number of pixels
        %   per world unit in the x and y direction respectively. Thus, fx
        %   and fy are in pixels.
        K;
    end
    
    properties (Dependent, SetAccess=protected, Hidden)
       %IntrinsicMatrix A 3-by-3 post-multiply projection matrix.
        %   IntrinsicMatrix is of the form [fx 0 0; s fy 0; cx cy 1], where
        %   [cx, cy] are the coordinates of the optical center (the
        %   principal point) in pixels and s is the skew parameter which is
        %   0 if the x and y axes are exactly perpendicular. fx = F * sx
        %   and fy = F * sy, where F is the focal length in world units,
        %   typically millimeters, and [sx, sy] are the number of pixels
        %   per world unit in the x and y direction respectively. Thus, fx
        %   and fy are in pixels.
        IntrinsicMatrix;
    end

    properties (Access=protected, Hidden)
       UndistortMap;
    end

    methods
        
        function this = cameraIntrinsicsImpl(varargin)
            
            narginchk(3, inf);
            r = parseInputs(varargin{:});
            
            % Required parameters
            if isscalar(r.focalLength) % scalar expand
                this.FocalLength   = [r.focalLength, r.focalLength];
            else
                this.FocalLength   = r.focalLength;
            end
            this.PrincipalPoint    = r.principalPoint;
            this.ImageSize         = r.imageSize;
            
            % Parameters from N-V pairs
            this.RadialDistortion     = r.RadialDistortion;
            this.TangentialDistortion = r.TangentialDistortion;
            this.Skew                 = r.Skew;
                        
            % Derived parameters
            this.K  = this.buildIntrinsicMatrix();
            
            % NOTE: classes for the above quantities can be mixed. Use
            % MATLAB rules for choosing the class of K, i.e. single wins.

            % Initialize the undistort map property. It must be done in the
            % constructor in order to successfully cache the undistortion
            % map inside a cameraIntrinsics object.
            this.UndistortMap = vision.internal.calibration.ImageTransformer;
        end

    end

    %----------------------------------------------------------------------
    % Implementation of abstract methods.
    %----------------------------------------------------------------------
    methods (Hidden=true)

        %------------------------------------------------------------------
        function updateUndistortMap(this, I, outputView, xBounds, yBounds)
            this.UndistortMap.update(I, this.K, this.RadialDistortion, ...
                this.TangentialDistortion, outputView, xBounds, yBounds);
        end

        %------------------------------------------------------------------
        % Apply radial and tangential distortion to a set of points
        %------------------------------------------------------------------
        function distortedPoints = distortPoints(this, points, ~)
            if isSimMode
                % in matlab use the builtin c++ function
                intrinsicMatrix = cast(this.K, 'like', points);
                radialDistortion = cast(this.RadialDistortion, 'like', points);
                tangentialDistortion = cast(this.TangentialDistortion, 'like', points);
                distortedPoints = visionDistortPoints(points, ...
                    intrinsicMatrix, radialDistortion, tangentialDistortion);
            else            
                if isvector(points)
                    pointsVec = points(:);
                    pointsReshaped = reshape(points, [size(pointsVec,1)/2,2]);
                else
                    pointsReshaped = points;
                end
		        % in codegen use matlab function
                distortedPoints = vision.internal.calibration.distortPoints(...
                    pointsReshaped, this.K, this.RadialDistortion, ...
                    this.TangentialDistortion);
            end
        end

        %------------------------------------------------------------------
        function undistortedMask = computeUndistortedMask(this, xBounds, ...
                yBounds, imageSize, outputView)

            mask = ones(imageSize, 'uint8');
            fillValuesMask = uint8(0);
            
            myMap = vision.internal.calibration.ImageTransformer;
            myMap.update(mask, this.K, ...
                this.RadialDistortion, this.TangentialDistortion,...
                outputView, xBounds, yBounds);
            
            undistortedMask = myMap.transformImage(mask, 'nearest', ...
                fillValuesMask);
        end  
    end

    %----------------------------------------------------------------------
    % Property set/get methods
    %----------------------------------------------------------------------
    methods
        function this = set.IntrinsicMatrix(this, intrinsicMatrix)
            this.K = intrinsicMatrix';
        end

        function intrinsicMatrix = get.IntrinsicMatrix(this)
            intrinsicMatrix = this.K';
        end
    end
    %----------------------------------------------------------------------
    %
    %----------------------------------------------------------------------
    methods (Access='private', Hidden=true)
        
        %------------------------------------------------------------------
        function intrinsicMat = buildIntrinsicMatrix(this)
            intrinsicMat = ...
                [this.FocalLength(1), this.Skew          , this.PrincipalPoint(1); ...
                0                   , this.FocalLength(2), this.PrincipalPoint(2); ...
                0                   , 0                  , 1];
        end
    end

    %----------------------------------------------------------------------
    % Static methods
    %----------------------------------------------------------------------
    methods(Static, Hidden)
        
        %------------------------------------------------------------------
        function checkSkew(skew)
            validateattributes(skew, {'double', 'single'}, ...
                {'scalar','real', 'nonsparse', 'finite'}, 'cameraIntrinsics', 'Skew');
        end
        
    end
end

function r = parseInputs(varargin)
    % Define default values
    defaultParams = struct(...
        'RadialDistortion', [0 0], ...
        'TangentialDistortion', [0 0], ...
        'Skew', 0);
    
    if coder.target('MATLAB') % MATLAB
        r = parseInputsSimulation(defaultParams,varargin{:});
    else % Code generation
        r = parseInputsCodegen(defaultParams,varargin{:});
    end
end

function r = parseInputsSimulation(defaultParams,varargin)

    parser = inputParser;
    focalLength = varargin{1};
    validateattributes(focalLength, {'double', 'single'}, ...
                {'vector','real', 'nonsparse', 'finite', 'positive'}, ...
                'cameraIntrinsics', 'abcd');
    
    filename = 'cameraIntrinsics';
    parser.addRequired('focalLength', @(f)vision.internal.inputValidation.validateFocalLength(f,filename));
    parser.addRequired('principalPoint', @(p)vision.internal.inputValidation.validatePrincipalPoint(p,filename));
    parser.addRequired('imageSize', @(sz)vision.internal.inputValidation.validateImageSize(sz,filename));
    
    parser.addParameter('RadialDistortion', defaultParams.RadialDistortion, ...
        @vision.internal.calibration.CameraParametersImpl.checkRadialDistortion);
    parser.addParameter('TangentialDistortion', defaultParams.TangentialDistortion,...
        @vision.internal.calibration.CameraParametersImpl.checkTangentialDistortion);
    
    parser.addParameter('Skew', defaultParams.Skew, @cameraIntrinsics.checkSkew);
    
    % Parse and check optional parameters
    parser.parse(varargin{:});
    r = parser.Results;
end

function r = parseInputsCodegen(defaultParams,varargin)
    
    filename = 'cameraIntrinsics';
    
    focalLength = varargin{1};
    vision.internal.inputValidation.validateFocalLength(focalLength, filename);
    
    principalPoint = varargin{2};
    vision.internal.inputValidation.validatePrincipalPoint(principalPoint, filename);
    
    imageSize = varargin{3};
    vision.internal.inputValidation.validateImageSize(imageSize, filename);
    
    parms = struct( ...
        'RadialDistortion', uint32(0), ...
        'TangentialDistortion', uint32(0), ...
        'Skew', uint32(0));
    
    popt = struct( ...
        'CaseSensitivity', false, ...
        'StructExpand', true, ...
        'PartialMatching', false);
    
    optarg = eml_parse_parameter_inputs(parms, popt, varargin{4:end});
    
    radialDistortion = eml_get_parameter_value(optarg.RadialDistortion,...
        defaultParams.RadialDistortion, varargin{4:end});
    vision.internal.calibration.CameraParametersImpl.checkRadialDistortion(radialDistortion);
    
    tangentialDistortion = eml_get_parameter_value(optarg.TangentialDistortion,...
        defaultParams.TangentialDistortion, varargin{4:end});
    vision.internal.calibration.CameraParametersImpl.checkTangentialDistortion(tangentialDistortion);
    
    skew = eml_get_parameter_value(optarg.Skew,...
        defaultParams.Skew, varargin{4:end});
    cameraIntrinsics.checkSkew(skew);
    
    r = struct( ...
        'focalLength', focalLength, ...
        'principalPoint', principalPoint, ...
        'imageSize', imageSize, ...
        'RadialDistortion', radialDistortion, ...
        'TangentialDistortion', tangentialDistortion, ...
        'Skew', skew);
end

function mode = isSimMode()
    mode = isempty(coder.target);
end