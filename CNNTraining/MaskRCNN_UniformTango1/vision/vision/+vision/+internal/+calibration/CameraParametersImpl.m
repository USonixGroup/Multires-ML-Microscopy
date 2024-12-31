% CameraParametersImpl Internal implementation of an object for storing camera parameters.

%   Copyright 2013-2024 The MathWorks, Inc.

%#codegen
%#ok<*EMCLS>
%#ok<*EMCA>        
classdef CameraParametersImpl < vision.internal.cameraIntrinsicsBase & ...
                                vision.internal.EnforceScalarHandle

    properties (GetAccess=public, SetAccess=public)
        % ImageSize [mrows, ncols] vector specifying image size produced by
        %   the camera.
        %
        %   Default: []
        ImageSize;        
    end
    
    properties (GetAccess=public, SetAccess=protected)
        % RadialDistortion A 2-element vector [k1 k2], 3-element vector [k1 k2 k3],
        %   or a six-element vector [k1 k2 k3 k4 k5 k6]. If a 2-element vector
        %   is supplied, the third coefficient is assumed to be 0. 
        %   The radial distortion is caused by the fact
        %   that light rays are bent more the farther away they are from
        %   the optical center. Distorted location of a point is computed
        %   as follows:
        %     x_distorted = x(1 + k1 * r^2 + k2 * r^4 + k3 * r^6) / (1 + k4 * r^2 + k5 * r^4 + k6 * r^6)
        %     y_distorted = y(1 + k1 * r^2 + k2 * r^4 + k3 * r^6) / (1 + k4 * r^2 + k5 * r^4 + k6 * r^6)
        %   where [x,y] is a non-distorted image point in normalized image
        %   coordinates in world units with the origin at the optical center,
        %   and r^2 = x^2 + y^2. Typically two coefficients are sufficient,
        %   and additional coefficients are only needed for severe distortion.
        %
        %   Default: [0 0 0]
        RadialDistortion;
        
        % TangentialDistortion A 2-element vector [p1 p2]. Tangential
        %   distortion is caused by the lens not being exactly parallel to
        %   to the image plane. Distorted location of a point is computed
        %   as follows:
        %     x_distorted = x + [2 * p1 * x * y + p2 * (r^2 + 2 * x^2)]
        %     y_distorted = y + [p1 * (r^2 + 2*y^2) + 2 * p2 * x * y]
        %   where [x,y] is a non-distorted image point in normalized image
        %   coordinates in world units with the origin at the optical center,
        %   and r^2 = x^2 + y^2.
        %
        %   Default: [0 0]'
        TangentialDistortion;
                
        % WorldPoints An M-by-2 array of [x,y] world coordinates of
        %   keypoints on the calibration pattern, where M is the number of
        %   keypoints in the pattern.  WorldPoints must be non-empty for
        %   showExtrinsics to work.
        %
        %   Default: []
        WorldPoints;
        
        % WorldUnits A string describing the units, in which the
        %   WorldPoints are specified.
        %
        %   Default: 'mm'
        WorldUnits;
        
        % EstimateSkew A logical scalar that specifies whether image axes
        %   skew was estimated. When set to false, the image axes are
        %   assumed to be exactly perpendicular.
        %
        %   Default: false
        EstimateSkew;
        
        % NumRadialDistortionCoefficients 2 or 3. Specifies the number
        %   of radial distortion coefficients that were estimated.
        %
        %   Default: 2
        NumRadialDistortionCoefficients;
        
        % EstimateTangentialDistortion A logical scalar that specifies
        %   whether tangential distortion was estimated. When set to false,
        %   tangential distortion is assumed to be negligible.
        %
        %   Default: false
        EstimateTangentialDistortion;
        
        % ReprojectionErrors An M-by-2-by-P array of [x,y] pairs representing
        %   the translation in x and y between the reprojected pattern
        %   keypoints and the detected pattern keypoints. These values
        %   indicate the accuracy of the estimated camera parameters. P is
        %   the number of pattern images used to estimate camera
        %   parameters, and M is the number of keypoints in each image.
        %
        %   Default: []
        ReprojectionErrors;

        % DetectedKeypoints An M-by-P logical array representing the
        %   detected keypoints in the calibration pattern. M is the number
        %   of keypoints in the complete calibration pattern and P is the
        %   number of calibration images. 
        %
        %   Default: []
        DetectedKeypoints;

        % RotationVectors An M-by-3 matrix containing M rotation vectors
        %   Each vector describes the 3-D rotation of the camera's image
        %   plane relative to the corresponding calibration pattern. The
        %   vector specifies the 3-D axis about which the camera is rotated,
        %   and its magnitude is the rotation angle in radians. The
        %   corresponding 3-D rotation matrices are given by the
        %   PatternExtrinsics property in the form of tfrom.R, where tform
        %   is a rigidtform3d object.
        %
        %   Default: []
        RotationVectors;

        % K A 3-by-3 projection matrix of the form
        %   [fx s cx; 0 fy cy; 0 0 1], where [cx, cy] are the coordinates
        %   of the optical center (the principal point) in pixels and s is
        %   the skew parameter which is 0 if the x and y axes are exactly
        %   perpendicular. fx = F * sx and fy = F * sy, where F is the
        %   focal length in world units, typically millimeters, and [sx, sy]
        %   are the number of pixels per world unit in the x and y direction
        %   respectively. Thus, fx and fy are in pixels.
        %
        %   Default: eye(3)
        K;
    end
    
    properties(Dependent)
        % NumPatterns The number of calibration patterns which were used to
        %   estimate the camera extrinsics. This is also the number of
        %   translation and rotation vectors.
        NumPatterns;
        
        % Intrinsics A cameraIntrinsics object storing information about the 
        %   intrinsic calibration parameters of the camera, including the  
        %   lens distortion parameters.
        Intrinsics;

        % PatternExtrinsics An M-by-1 array containing M rigidtform3d objects.
        %   Each rigidtform3d object represents the pose of the camera's
        %   image plane relative to the corresponding calibration pattern.
        %
        %   The transformation relating a world point [X Y Z] and the
        %   corresponding image point [x y] is given by the following equation:
        %
        %      s * [x y 1]' = K * [tform.R, tform.Translation'] * [X Y Z 1]
        %
        %   where
        %     - tform is a rigidtform3d object
        %     - K is the pre-multiply IntrinsicMatrix
        %     - s is a scalar
        %   Note that this equation does not account for lens distortion.
        %   Thus it assumes that the distortion has been removed using
        %   undistortImage function.
        PatternExtrinsics;
        
        % FocalLength A 2-element vector [fx, fy].  fx = F * sx and
        %   fy = F * sy, where F is the focal length in world units,
        %   typically millimeters, and [sx, sy] are the number of pixels
        %   per world unit in the x and y direction respectively. Thus,
        %   fx and fy are in pixels.
        FocalLength;
        
        % PrincipalPoint A 2-element vector [cx, cy], containing the
        %   coordinates of the optical center of the camera in pixels.
        PrincipalPoint;
        
        % Skew A scalar, containing the camera axes skew, which is 0 if the
        % x and the y axes are exactly perpendicular.
        Skew;
        
        % MeanReprojectionError Average Euclidean distance between
        %   reprojected points and detected points.
        MeanReprojectionError;
        
        % ReprojectedPoints An M-by-2-by-NumPatterns array of [x,y] coordinates of
        %   world points re-projected onto calibration images. M is the
        %   number of points per image. NumPatterns is the number of
        %   patterns. [NaN, NaN] coordinates correspond to undetected keypoints in
        %   the calibration pattern.
        ReprojectedPoints;
    end
    
    properties (Access=protected, Hidden)
        UndistortMap;
        Version = [];
    end

    properties (GetAccess=public, SetAccess=protected, Hidden)
        % RotationMatrices A 3-by-3-by-P array containing P rotation matrices.
        %   Each 3-by-3 matrix represents the 3-D rotation of the camera's
        %   image plane relative to the corresponding calibration pattern.
        %
        %   The transformation relating a world point [X Y Z] and the
        %   corresponding image point [x y] is given by the following equation:
        %              s * [x y 1] = [X Y Z 1] * [R; t] * K
        %   where
        %     - R is the 3-D post-multiply rotation matrix
        %     - t is the translation row vector
        %     - K is the post-multiply IntrinsicMatrix
        %     - s is a scalar
        %   Note that this equation does not account for lens distortion.
        %   Thus it assumes that the distortion has been removed using
        %   undistortImage function.
        RotationMatrices;

        % TranslationVectors An M-by-3 matrix containing M translation vectors.
        %   Each vector describes the translation of the camera's image plane
        %   relative to the corresponding calibration pattern in world units.
        %
        %   The transformation relating a world point [X Y Z] and the
        %   corresponding image point [x y] is given by the following equation:
        %              s * [x y 1] = [X Y Z 1] * [R; t] * K
        %   where
        %     - R is the 3-D post-multiply rotation matrix
        %     - t is the translation row vector
        %     - K is the post-multiply IntrinsicMatrix
        %     - s is a scalar
        %   Note that this equation does not account for lens distortion.
        %   Thus it assumes that the distortion has been removed using
        %   undistortImage function.
        %
        %   Default: []
        TranslationVectors;
    end

    properties (Dependent, SetAccess=protected, Hidden)
        % IntrinsicMatrix A 3-by-3 projection matrix of the form
        %   [fx 0 0; s fy 0; cx cy 1], where [cx, cy] are the coordinates
        %   of the optical center (the principal point) in pixels and s is
        %   the skew parameter which is 0 if the x and y axes are exactly
        %   perpendicular. fx = F * sx and fy = F * sy, where F is the
        %   focal length in world units, typically millimeters, and [sx, sy]
        %   are the number of pixels per world unit in the x and y direction
        %   respectively. Thus, fx and fy are in pixels.
        %
        %   Default: eye(3)
        IntrinsicMatrix;
    end
    
    
    methods
        %----------------------------------------------------------------------
        function this = CameraParametersImpl(varargin)
            if isSimMode
                paramStruct = vision.internal.calibration.CameraParametersImpl.parseInputsSim(varargin{:});
            else
                paramStruct = vision.internal.calibration.CameraParametersImpl.parseInputsCodegen(varargin{:});
            end
            validateInputs(this, paramStruct);
            
            coder.internal.errorIf((isempty(this.RotationVectors) && ...
                    ~isempty(this.TranslationVectors)) || ...
                    (isempty(this.TranslationVectors) && ...
                    ~isempty(this.RotationVectors)),... 
                'vision:calibrate:rotationAndTranslationVectorsMustBeSetTogether');
            
            coder.internal.errorIf(...
                any(size(this.RotationVectors) ~= size(this.TranslationVectors)),...
                'vision:calibrate:rotationAndTranslationVectorsNotSameSize');
            
            coder.internal.errorIf(~isempty(this.ReprojectionErrors) &&...
                size(this.ReprojectionErrors, 3) ~= size(this.TranslationVectors, 1),...
                'vision:calibrate:reprojectionErrorsSizeMismatch');
            
            % Initialize the undistort map property. It must be done in the
            % constructor in order to successfully cache the undistortion
            % map inside a CameraParameters object.
            this.UndistortMap = vision.internal.calibration.ImageTransformer;
        end
        
        %------------------------------------------------------------------
        function paramStruct = toStruct(this)
            % toStruct Convert a cameraParameters object into a struct.
            %   paramStruct = toStruct(cameraParams) returns a struct
            %   containing the camera parameters, which can be used to
            %   create an identical cameraParameters object.
            %
            %   This method is useful for C code generation. You can call
            %   toStruct, and then pass the resulting structure into the generated
            %   code, which re-creates the cameraParameters object.
            paramStruct = saveobj(this);
        end                
    end
    
    methods(Access=private)
        function validateInputs(this, paramStruct)

            this.K = paramStruct.K;
            this.IntrinsicMatrix = paramStruct.IntrinsicMatrix;

            vision.internal.calibration.CameraParametersImpl.checkRadialDistortion(paramStruct.RadialDistortion);
            this.RadialDistortion = paramStruct.RadialDistortion(:)';
            
            vision.internal.calibration.CameraParametersImpl.checkTangentialDistortion(paramStruct.TangentialDistortion);
            this.TangentialDistortion = paramStruct.TangentialDistortion(:)';
            
            vision.internal.calibration.CameraParametersImpl.checkImageSize(paramStruct.ImageSize);
            this.ImageSize = paramStruct.ImageSize;
            
            vision.internal.calibration.CameraParametersImpl.checkRotationVectors(paramStruct.RotationVectors)
            this.RotationVectors = paramStruct.RotationVectors;
            
            vision.internal.calibration.CameraParametersImpl.checkTranslationVectors(paramStruct.TranslationVectors);
            this.TranslationVectors = paramStruct.TranslationVectors;
            
            vision.internal.calibration.CameraParametersImpl.checkWorldPoints(paramStruct.WorldPoints);
            this.WorldPoints = paramStruct.WorldPoints;
            
            vision.internal.calibration.CameraParametersImpl.checkWorldUnits(paramStruct.WorldUnits);
            this.WorldUnits = paramStruct.WorldUnits;
            
            vision.internal.calibration.CameraParametersImpl.checkEstimateSkew(paramStruct.EstimateSkew );
            this.EstimateSkew = paramStruct.EstimateSkew;
            
            vision.internal.calibration.CameraParametersImpl.checkNumRadialCoeffs(paramStruct.NumRadialDistortionCoefficients);
            this.NumRadialDistortionCoefficients = paramStruct.NumRadialDistortionCoefficients;
            
            vision.internal.calibration.CameraParametersImpl.checkEstimateTangentialDistortion(paramStruct.EstimateTangentialDistortion);
            this.EstimateTangentialDistortion = paramStruct.EstimateTangentialDistortion;
            
            vision.internal.calibration.CameraParametersImpl.checkReprojectionErrors(paramStruct.ReprojectionErrors);
            this.ReprojectionErrors = paramStruct.ReprojectionErrors;
            
            vision.internal.calibration.CameraParametersImpl.checkDetectedKeypoints(paramStruct.DetectedKeypoints );
            this.DetectedKeypoints = paramStruct.DetectedKeypoints;
            
        end
    end
    
    methods(Static, Hidden)
        %------------------------------------------------------------------
        function paramStruct = parseInputsCodegen(varargin)
            params = struct( ...
                'K',                    uint32(0),...
                'IntrinsicMatrix',      uint32(0),...
                'RadialDistortion',     uint32(0),...
                'TangentialDistortion', uint32(0),...
                'ImageSize',            uint32(0),...
                'RotationVectors',      uint32(0),...
                'TranslationVectors',   uint32(0),...
                'WorldPoints',          uint32(0),...
                'WorldUnits',           uint32(0),...
                'EstimateSkew',         uint32(0),...
                'NumRadialDistortionCoefficients', uint32(0),...
                'EstimateTangentialDistortion',    uint32(0),...
                'ReprojectionErrors',              uint32(0),...
                'DetectedKeypoints',               uint32(0),...
                'Version', uint32(0));
            
            popt = struct( ...
                'CaseSensitivity', false, ...
                'StructExpand',    true, ...
                'PartialMatching', true);
            
            optarg = coder.internal.parseParameterInputs(params, popt, varargin{:});

            % If both K and IntrinsicMatrix are given assignments, give error asking for only K input.
            coder.internal.errorIf((optarg.K && optarg.IntrinsicMatrix), 'vision:calibrate:assignedBothIntrinsicMatrices')

            % Codegen needs the original property assignment of the inversed
            % intrinsics matrix to match the input assignment type. If K is
            % assigned as a single, IntrinsicMatrix must intialize as a
            % single too.
            if optarg.K
                intrinsicMatrix = coder.internal.getParameterValue(optarg.K, eye(3), varargin{:});
                validateattributes(intrinsicMatrix, {'double', 'single'}, ...
                    {'2d', 'ncols', 3, 'nrows', 3, 'nonsparse', 'real', 'finite'}, ...
                    'cameraParameters', 'K');
                dataType = class(intrinsicMatrix);
            elseif optarg.IntrinsicMatrix
                intrinsicMatrix = coder.internal.getParameterValue(optarg.IntrinsicMatrix, eye(3), varargin{:});
                validateattributes(intrinsicMatrix, {'double', 'single'}, ...
                    {'2d', 'ncols', 3, 'nrows', 3, 'nonsparse', 'real', 'finite'}, ...
                    'cameraParameters', 'IntrinsicMatrix');
                dataType = class(intrinsicMatrix);
            else
                dataType = class(eye);
            end

            paramStruct.K = coder.internal.getParameterValue(...
                optarg.K, eye(3, dataType), varargin{:});

            paramStruct.IntrinsicMatrix = coder.internal.getParameterValue(...
                optarg.IntrinsicMatrix, eye(3, dataType), varargin{:});  

            paramStruct = checkAssignedIntrinsicMatrix(paramStruct);
            
            paramStruct.RadialDistortion = coder.internal.getParameterValue(...
                optarg.RadialDistortion, [0 0 0], varargin{:});
            
            paramStruct.TangentialDistortion = coder.internal.getParameterValue(...
                optarg.TangentialDistortion, [0 0], varargin{:});
            
            paramStruct.ImageSize = coder.internal.getParameterValue(...
                optarg.ImageSize, zeros(0, 2), varargin{:});
            
            paramStruct.RotationVectors = coder.internal.getParameterValue(...
                optarg.RotationVectors, zeros(0, 3), varargin{:});
            
            paramStruct.TranslationVectors = coder.internal.getParameterValue(...
                optarg.TranslationVectors, zeros(0, 3), varargin{:});
            
            paramStruct.WorldPoints = coder.internal.getParameterValue(...
                optarg.WorldPoints, zeros(0, 2), varargin{:});
             
            paramStruct.WorldUnits = coder.internal.getParameterValue(...
                optarg.WorldUnits, 'mm', varargin{:});
            
            paramStruct.EstimateSkew = coder.internal.getParameterValue(...
                optarg.EstimateSkew, false, varargin{:});
             
            paramStruct.NumRadialDistortionCoefficients = coder.internal.getParameterValue(...
                optarg.NumRadialDistortionCoefficients, 2, varargin{:});
             
            paramStruct.EstimateTangentialDistortion = coder.internal.getParameterValue(...
                optarg.EstimateTangentialDistortion, false, varargin{:});
            
            paramStruct.DetectedKeypoints = coder.internal.getParameterValue(...
                optarg.DetectedKeypoints, zeros(0, 2), varargin{:});
             
            reprojErrors = coder.internal.getParameterValue(...
                optarg.ReprojectionErrors, zeros(0, 2), varargin{:});
            
            if isempty(reprojErrors)
                paramStruct.ReprojectionErrors = zeros(0, 2);
            else
                paramStruct.ReprojectionErrors = reprojErrors;
            end
            
            paramStruct.Version = coder.internal.getParameterValue(...
                optarg.Version, '', varargin{:});
            
        end
        
        function paramStruct = parseInputsSim(varargin)
            parser = inputParser;
            parser.addParameter('K', eye(3));
            parser.addParameter('IntrinsicMatrix', eye(3));
            parser.addParameter('RadialDistortion', [0 0 0]);
            parser.addParameter('TangentialDistortion', [0 0]);
            parser.addParameter('ImageSize', zeros(0,2));
            parser.addParameter('RotationVectors', zeros(0, 3));
            parser.addParameter('TranslationVectors', zeros(0, 3));
            parser.addParameter('WorldPoints', zeros(0, 2));
            parser.addParameter('WorldUnits', 'mm');
            parser.addParameter('EstimateSkew', false);
            parser.addParameter('NumRadialDistortionCoefficients', 2);
            parser.addParameter('EstimateTangentialDistortion', false);
            parser.addParameter('ReprojectionErrors', zeros(0, 2));
            parser.addParameter('DetectedKeypoints', zeros(0,2));

            % Version is actually not used directly.  The only reason
            % for this parsing is to enable toStruct and fromStruct
            % methods to work seamlessly because toStruct calls saveobj
            % to get all object properties into a struct. It can then
            % be used to recreate the object via the constructor,
            % which is forced to parse 'Version' P-V pair.
            % Both of these methods are used only for codegen.
            parser.addParameter('Version', []);

            parser.parse(varargin{:});

            % If both K and IntrinsicMatrix are given assignments, give error asking for only K input.
            unsetValues = parser.UsingDefaults;
            coder.internal.errorIf((~ismember('K', unsetValues) && ...
                ~ismember('IntrinsicMatrix', unsetValues)), 'vision:calibrate:assignedBothIntrinsicMatrices')
            paramStruct = checkAssignedIntrinsicMatrix(parser.Results);
        end
    end
        
    methods
        %------------------------------------------------------------------
        function set.ImageSize(this, in)
            this.checkImageSize(in);
            this.ImageSize = in;
        end

        %------------------------------------------------------------------
        function set.IntrinsicMatrix(this, intrinsicMatrix)
            this.K = intrinsicMatrix';
        end
        
        %------------------------------------------------------------------
        function numPatterns = get.NumPatterns(this)
            numPatterns = size(this.RotationVectors, 1);
        end

        %------------------------------------------------------------------
        function intrinsicMatrix = get.IntrinsicMatrix(this)
            intrinsicMatrix = this.K';
        end
        
        %------------------------------------------------------------------
        function intrinsics = get.Intrinsics(this)

            if (isempty(this.ImageSize) || any(this.FocalLength==0) || ...
                    any(this.PrincipalPoint==0)) && isSimMode
                % The method 'empty' is not supported for code generation.
                % Only show empty intrinsics in simulation mode. In code
                % generation mode, rely on the constructor of cameraIntrinsics
                % to validate inputs during running time.
                intrinsics = cameraIntrinsics.empty();
            else
                intrinsics = cameraIntrinsics(this.FocalLength, ...
                    this.PrincipalPoint, this.ImageSize, ...
                    'RadialDistortion', this.RadialDistortion, ...
                    'TangentialDistortion', this.TangentialDistortion, ...
                    'Skew', this.Skew);
            end
        end

        %------------------------------------------------------------------
        function patternExtrinsics = get.PatternExtrinsics(this)
            patternExtrinsics = repelem(rigidtform3d, size(this.RotationMatrices, 3), 1);
            for i = 1:size(this.RotationMatrices, 3)
                patternExtrinsics(i, 1) = rigidtform3d(...
                    this.RotationMatrices(:,:,i)', this.TranslationVectors(i,:));
            end
        end
        
        %------------------------------------------------------------------
        function meanError = get.MeanReprojectionError(this)
            meanError = computeMeanError(this);
        end
        
        %------------------------------------------------------------------
        function reprojectedPoints = get.ReprojectedPoints(this)
            points = this.WorldPoints;
            reprojectedPoints = zeros([size(points), this.NumPatterns]);
            
            for i = 1:this.NumPatterns
                reprojectedPoints(:, :, i) = ...
                    reprojectWorldPointsOntoPattern(this, i);
            end
            
            % apply distortion
            reprojectedPoints = distortPoints(this, reprojectedPoints);
            
            % Set undetected corners in board to NaN
            for i = 1:this.NumPatterns 
                reprojectedPoints(~this.DetectedKeypoints(:,i),:,i) = NaN;
            end
        end
        
        %------------------------------------------------------------------
        function rotationMatrices = get.RotationMatrices(this)
            rotationMatrices = zeros(3, 3, this.NumPatterns);
            for i = 1:this.NumPatterns
                v = this.RotationVectors(i, :);
                R = vision.internal.calibration.rodriguesVectorToMatrix(v);
                rotationMatrices(:, :, i) = R';
            end
        end
        
        %------------------------------------------------------------------
        function focalLength = get.FocalLength(this)
            focalLength = zeros(1, 2, 'like', this.K);
            focalLength(1) = this.K(1, 1);
            focalLength(2) = this.K(2, 2);
        end
        
        %------------------------------------------------------------------
        function principalPoint = get.PrincipalPoint(this)
            principalPoint = zeros(1, 2, 'like', this.K);
            principalPoint(1) = this.K(1, 3);
            principalPoint(2) = this.K(2, 3);
        end
        
        %------------------------------------------------------------------
        function skew = get.Skew(this)
            skew = this.K(1, 2);
        end
    end
    
    methods(Hidden, Access=public)
        %------------------------------------------------------------------
        % This method is different from the MeanReprojectionError property,
        % because it also computes the mean reprojection error per image.
        %------------------------------------------------------------------
        function [meanError, meanErrorsPerImage] = computeMeanError(this)
            errors = hypot(this.ReprojectionErrors(:, 1, :), ...
                this.ReprojectionErrors(:, 2, :));
            
            % Mean error over detected corners in each image
            meanErrorsPerImage = squeeze(sum(errors, 1, 'omitnan'))./squeeze(sum(~isnan(errors), 1));
            meanError = mean(meanErrorsPerImage);
        end
    end

    %----------------------------------------------------------------------
    % constructor parameter validation
    %----------------------------------------------------------------------
    methods(Static)
        %------------------------------------------------------------------        
        function checkIntrinsicMatrix(preIntrinsicMatrix)
            validateattributes(preIntrinsicMatrix, {'double', 'single'}, ...
                {'2d', 'ncols', 3, 'nrows', 3, 'nonsparse', 'real', 'finite'}, ...
                'cameraParameters', 'K');

            % Elements (2,1), (3,1), and (3,2) should be zero elements in K
            coder.internal.errorIf(any(preIntrinsicMatrix([2, 3, 6])), 'vision:calibrate:kIntrinsicMatrixNonZeroElements')
        end

        %------------------------------------------------------------------        
        function checkIntrinsicMatrixPostMultiply(intrinsicMatrix)
            validateattributes(intrinsicMatrix, {'double', 'single'}, ...
                {'2d', 'ncols', 3, 'nrows', 3, 'nonsparse', 'real', 'finite'}, ...
                'cameraParameters', 'IntrinsicMatrix');

            % Elements (1,2), (1,3), and (2,3) should be zero elements in IntrinsicMatrix
            coder.internal.errorIf(any(intrinsicMatrix([4, 7, 8])), 'vision:calibrate:intrinsicMatrixNonZeroElements')
        end
        
        %------------------------------------------------------------------        
        function checkRadialDistortion(radialDistortion)
            validTypes = {'double', 'single'};
            validateattributes(radialDistortion, validTypes,...
                {'vector', 'real', 'nonsparse', 'finite'},...
                'cameraParameters', 'RadialDistortion');
            
            coder.internal.errorIf(...
                numel(radialDistortion) ~= 2 && numel(radialDistortion) ~= 3 && numel(radialDistortion) ~= 6, ...
                'vision:dims:twoThreeOrSixElementVector','RadialDistortion');
        end
        %------------------------------------------------------------------        
        function checkTangentialDistortion(tangentialDistortion)
            validateattributes(tangentialDistortion, {'double', 'single'},...
                {'vector', 'real', 'nonsparse', 'finite', 'numel', 2},...
                'cameraParameters', 'TangentialDistortion');
        end
       
        %------------------------------------------------------------------
        function checkImageSize(imageSize)
            if isempty(imageSize)
                return;
            end            
            validateattributes(imageSize, {'double', 'single'}, ...
                {'vector','real', 'nonsparse','numel', 2, 'integer', 'positive'}, ...
                mfilename, 'imageSize');
        end        
        
        %------------------------------------------------------------------                
        function checkRotationVectors(rotationVectors)
            if isempty(rotationVectors)
                return;
            end
            validateattributes(rotationVectors, {'double', 'single'},...
                {'2d', 'real', 'nonsparse', 'finite', 'ncols', 3},...
                'cameraParameters', 'RotationVectors');
        end
        
        %------------------------------------------------------------------                
        function checkTranslationVectors(translationVectors)
            if isempty(translationVectors)
                return;
            end
            validateattributes(translationVectors, {'double', 'single'},...
                {'2d', 'real', 'nonsparse', 'finite', 'ncols', 3},...
                'cameraParameters', 'TranslationVectors');
        end        
        
        %------------------------------------------------------------------                        
        function checkWorldPoints(worldPoints)
            if isempty(worldPoints)
                return;        
            end    
            validateattributes(worldPoints, {'double', 'single'},...
                {'2d', 'real', 'nonsparse', 'ncols', 2},...
                'cameraParameters', 'WorldPoints');
        end
        
        %------------------------------------------------------------------                        
        function checkWorldUnits(worldUnits)
            if isstring(worldUnits)
                validateattributes(worldUnits, {'string'}, ...
                    {'scalar'}, 'cameraParameters', 'WorldUnits');
            else
                validateattributes(worldUnits, {'char'}, ...
                {'nonempty','vector'}, 'cameraParameters', 'WorldUnits');
            end
            
        end
        
        %------------------------------------------------------------------                        
        function checkEstimateSkew(esitmateSkew)
            validateattributes(esitmateSkew, {'logical'}, {'scalar'}, ...
                'cameraParameters', 'EstimateSkew');
        end
        
        %------------------------------------------------------------------                        
        function checkNumRadialCoeffs(numRadialCoeffs)
            validateattributes(numRadialCoeffs, {'numeric'}, ...
                {'scalar', 'integer'}, ...
                 'cameraParameters', 'NumRadialDistortionCoefficients');
                        
            coder.internal.errorIf(...
                numRadialCoeffs ~= 2 && numRadialCoeffs ~= 3 && numRadialCoeffs ~= 6, ...
                'vision:dims:twoThreeOrSixElementVector','numRadialCoeffs');
        end
        
        %------------------------------------------------------------------                        
        function checkEstimateTangentialDistortion(estimateTangential)
            validateattributes(estimateTangential, {'logical'}, {'scalar'},...
                'cameraParameters', 'EstimateTangentialDistortion');            
        end
        
        %------------------------------------------------------------------                        
        function checkReprojectionErrors(reprojErrors)
            if ~isempty(reprojErrors)
                validateattributes(reprojErrors, {'double', 'single'},...
                    {'3d', 'real', 'nonsparse', 'ncols', 2},...
                    'cameraParameters', 'ReprojectionErrors');
            end
        end
        
        %------------------------------------------------------------------
        function checkDetectedKeypoints(detectedKeypoints)
            if isempty(detectedKeypoints)
                return;        
            end    
            validateattributes(detectedKeypoints, {'logical'},...
                {'2d', 'binary', 'nonsparse'},...
                'fisheyeParameters', 'DetectedKeypoints');
        end
    end
    
    methods(Access=private)
        %------------------------------------------------------------------
        % Reproject world points using one set of extrinsics without
        % applying distortion
        %------------------------------------------------------------------
        function reprojectedPoints = ...
                reprojectWorldPointsOntoPattern(this, patternIdx)
            
            points = this.WorldPoints;
            R = vision.internal.calibration.rodriguesVectorToMatrix(...
                this.RotationVectors(patternIdx, :));
            t = this.TranslationVectors(patternIdx, :)';
            tform = projtform2d((this.K * [R(:, 1), R(:, 2), t]));
            reprojectedPoints = transformPointsForward(tform, points);
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
            fillValuesMask = cast(0, 'uint8');
            
            myMap = vision.internal.calibration.ImageTransformer;
            myMap.update(mask, this.K, ...
                this.RadialDistortion, this.TangentialDistortion,...
                outputView, xBounds, yBounds);
            
            undistortedMask = myMap.transformImage(mask, 'nearest', ...
                fillValuesMask);
        end  
    end
        
    methods (Hidden=true, Access=public)
        %------------------------------------------------------------------
        % Returns a CameraParameters object with updated reprojection
        % errors.  Used in stereoParameters.
        %------------------------------------------------------------------
        function computeReprojectionErrors(this, imagePoints)
            this.DetectedKeypoints = squeeze(~isnan(imagePoints(:,1,:)));
            this.ReprojectionErrors = this.ReprojectedPoints - imagePoints;
        end
        
        %------------------------------------------------------------------
        % Returns a CameraParameters object with new extrinsics.
        %------------------------------------------------------------------
        function setExtrinsics(this, rvecs, tvecs)
            this.RotationVectors = rvecs;
            this.TranslationVectors = tvecs;
        end
    end
        
    %----------------------------------------------------------------------
    % saveobj and loadobj are implemented to ensure compatibility across
    % releases.
    methods (Hidden)
       
        function that = saveobj(this)
            
            if isSimMode
                % Execute only in simulation mode
                this.Version = ver('vision');
            end
            
            that.RadialDistortion     = this.RadialDistortion;     
            that.TangentialDistortion = this.TangentialDistortion;
            that.ImageSize            = this.ImageSize;
            that.WorldPoints          = this.WorldPoints;
            that.WorldUnits           = this.WorldUnits;  
            that.EstimateSkew         = this.EstimateSkew;
            that.NumRadialDistortionCoefficients = this.NumRadialDistortionCoefficients;
            that.EstimateTangentialDistortion    = this.EstimateTangentialDistortion;
            that.RotationVectors      = this.RotationVectors;
            that.TranslationVectors   = this.TranslationVectors;
            that.ReprojectionErrors   = this.ReprojectionErrors; 
            that.K                    = this.K;
            that.DetectedKeypoints    = this.DetectedKeypoints; 
            that.Version              = this.Version;
        end
        
    end
    
    
    %--------------------------------------------------------------------------
    
    methods (Static, Hidden)
        
        function this = loadobj(that)
            
            if isempty(that.ReprojectionErrors)
                reprojErrors = zeros(0, 2, 0);
            else
                reprojErrors = that.ReprojectionErrors;
            end
            
            if ~isfield(that, 'ImageSize')
                imageSize = zeros(0,2);
            else
                imageSize = that.ImageSize;
            end

            % Ensure backward compatibility (DetectedKeypoints introduced in 21a)
            if ~isfield(that, 'DetectedKeypoints')
                if isempty(that.RotationVectors)
                    detectedKeypoints = zeros(0,2);
                else
                    numPoints = size(that.WorldPoints, 1);
                    numImages = size(that.RotationVectors, 1);
                    detectedKeypoints = true(numPoints, numImages);
                end
            else
                detectedKeypoints = that.DetectedKeypoints;
            end

            % Ensure backwards compatibility (K introduced in 22b)
            if ~isfield(that, 'K')
                K = that.IntrinsicMatrix';
            else
                K = that.K;
            end
            
            this = cameraParameters(...
                'K',                     K,...
                'RadialDistortion',      that.RadialDistortion,...
                'TangentialDistortion',  that.TangentialDistortion,...
                'ImageSize',             imageSize,...
                'WorldPoints',           that.WorldPoints,...
                'WorldUnits',            that.WorldUnits,...
                'EstimateSkew',          that.EstimateSkew,...
                'NumRadialDistortionCoefficients', that.NumRadialDistortionCoefficients,...
                'EstimateTangentialDistortion', that.EstimateTangentialDistortion,...
                'RotationVectors',       that.RotationVectors,...
                'TranslationVectors',    that.TranslationVectors,...
                'ReprojectionErrors',    reprojErrors, ...
                'DetectedKeypoints',     detectedKeypoints);
        end
        
    end
end

function mode = isSimMode()
mode = isempty(coder.target);
end

%--------------------------------------------------------------------------
% Check if K or IntrinsicMatrix is specified. Assign the transpose to the
% unassigned intrisic matrix.
%--------------------------------------------------------------------------
function paramStructAssigned = checkAssignedIntrinsicMatrix(paramStruct)
    kAssigned = ~isequal(paramStruct.K, eye(3));
    intrinsicMatrixAssigned = ~isequal(paramStruct.IntrinsicMatrix, eye(3));

    % When K is specified, assign IntrinsicMatrix as K'. When
    % IntrinsicMatrix is specified, assign K as IntrinsicMatrix'.
    if kAssigned
        % Verify specified matrix is a valid pre-multiply matrix
        vision.internal.calibration.CameraParametersImpl.checkIntrinsicMatrix(paramStruct.K);
        % Assign IntrinsicMatrix
        paramStruct.IntrinsicMatrix = paramStruct.K';
    elseif intrinsicMatrixAssigned
        % Verify specified matrix is a valid post-multiply matrix
        vision.internal.calibration.CameraParametersImpl.checkIntrinsicMatrixPostMultiply(paramStruct.IntrinsicMatrix);
        % Assign K
        paramStruct.K = paramStruct.IntrinsicMatrix';
    end
    paramStructAssigned = paramStruct;
end