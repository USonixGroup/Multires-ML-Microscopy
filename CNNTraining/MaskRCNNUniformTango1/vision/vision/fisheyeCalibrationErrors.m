classdef fisheyeCalibrationErrors

% Copyright 2017-2023 MathWorks, Inc.

    properties(GetAccess=public, SetAccess=private)
        % IntrinsicsErrors A fisheyeIntrinsicsEstimationErrors object 
        %   containing the standard error of the estimated camera
        %   intrinsics.
        IntrinsicsErrors;
        
        % ExtrinsicsErrors An extrinsicsEstimationErrors object 
        %   containing the standard error for the estimated camera rotations
        %   and translations relative to the calibration pattern.
        ExtrinsicsErrors;
    end
    
    properties (Access=private, Hidden)
        Version = ver('vision');
    end
    
    methods
        function this = fisheyeCalibrationErrors(errors)
            this.IntrinsicsErrors = fisheyeIntrinsicsEstimationErrors(errors);
            this.ExtrinsicsErrors = extrinsicsEstimationErrors(errors);
        end
        
        %------------------------------------------------------------------
        function displayErrors(this, fisheyeParams)
            
            checkDisplayErrorsInputs(this, fisheyeParams);
            
            displayMainHeading();
            
            % Intrinsics
            displayIntrinsicsHeading();           
            displayErrors(this.IntrinsicsErrors, fisheyeParams);
            
            % Extrinsics
            displayExtrinsicsHeading();            
            displayErrors(this.ExtrinsicsErrors, fisheyeParams);
        end
    end
    
    methods(Access=private)
        %------------------------------------------------------------------
        function checkDisplayErrorsInputs(~, fisheyeParams)
            % Check data type
            validateattributes(fisheyeParams, {'fisheyeParameters'}, {}, ...
                'displayErrors', 'fisheyeParams');
        end
    end
    
    %----------------------------------------------------------------------
    % saveobj and loadobj are implemented to ensure compatibility across
    % releases even if architecture of this class changes
    methods (Hidden)
        
        function that = saveobj(this)
            % version
            that.version = this.Version;
            
            % intrinsics            
            that.stretchMatrix = [this.IntrinsicsErrors.StretchMatrixError, 1];
            that.distortionCenter = this.IntrinsicsErrors.DistortionCenterError;
            that.mappingCoefficients = this.IntrinsicsErrors.MappingCoefficientsError;
           
            % extrinsics
            that.rotationVectors = this.ExtrinsicsErrors.RotationVectorsError;
            that.translationVectors = this.ExtrinsicsErrors.TranslationVectorsError;
        end
        
    end
    
    %----------------------------------------------------------------------
    methods (Static, Hidden)
        
        function this = loadobj(that)            
            this = fisheyeCalibrationErrors(that);
            this.Version = that.version;            
        end
        
    end
end

%--------------------------------------------------------------------------
function displayMainHeading()
headingFormat = '\n\t\t\t%s\n\t\t\t%s\n';
mainHeading = vision.getMessage(...
    'vision:cameraCalibrationErrors:singleCameraHeading');
headingUnderline = getUnderlineString(mainHeading);
fprintf(headingFormat, mainHeading, headingUnderline);
end

%--------------------------------------------------------------------------
function displayIntrinsicsHeading()
heading = vision.getMessage('vision:cameraCalibrationErrors:intrinsicsHeading');
fprintf('\n%s\n%s\n', heading, getUnderlineString(heading));
end

%--------------------------------------------------------------------------
function displayExtrinsicsHeading()
heading = vision.getMessage('vision:cameraCalibrationErrors:extrinsicsHeading');
fprintf('\n%s\n%s\n', heading, getUnderlineString(heading));
end

%--------------------------------------------------------------------------
function underline = getUnderlineString(header)
underline = repmat('-', [1, numel(header)]);
end
