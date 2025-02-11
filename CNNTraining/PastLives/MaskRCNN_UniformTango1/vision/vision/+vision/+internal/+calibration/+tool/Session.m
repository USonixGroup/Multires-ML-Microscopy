% Session holds the state of the Camera Calibration App
%
%   This class holds the entire state of the camera calibration UI.
%   It is used to save and load the camera calibration session. It is also
%   used to pass data amongst other classes.

% Copyright 2012-2024 The MathWorks, Inc.

classdef Session < handle

    properties
        CameraModel;             % holds current/selected camera model
        StandardCameraModelUI;   % holds standard options specified in the UI
        FisheyeCameraModelUI;    % holds fisheye options specified in the UI
        OptimizationOptions      = []; % initial values for intrinsics and radial distortion
        IsFisheyeModel           = false; % True if calibrated model is fisheye
        IsFisheyeSelected        = false; % True if model selected in UI isfisheye
        IsFixedIntrinsics        = false; % True if fixed intrinsics are used
        StereoIntrinsics1        = []; % Fixed Intrinsics for Camera 1
        StereoIntrinsics2        = []; % Fixed Intrinsics for Camera 2
        StereoIntrinsicsVarName1 = []; % Variable name for Camera 1 intrinsics
        StereoIntrinsicsVarName2 = [];% Variable name for Camera 2 intrinsics
        CameraParameters         = []; % actual calibration results
        EstimationErrors         = [];
        ShouldExportErrors       = false;
        
        PatternSet = [];            % holds all checkerboard information
        
        HasEnoughBoards = false;  % true if enough images were loaded and processed
        CanExport       = false;  % true when cameraParameters can be exported
        IsChanged       = false;  % true when session may need saving
        
        ExtrinsicsView = 'CameraCentric';
        
        % ErrorsView is no longer used, since we only use the BarGraph view
        % in the app. However, we have to keep this property around for
        % compatibility with older versions.
        ErrorsView = 'BarGraph';
        
        Filename = []; % filename for the session
        
        ExportVariableName = 'cameraParams'; % default export variable name     
        ExportErrorsVariableName = 'estimationErrors';
    end
    
    properties(Access=private, Hidden)
        Version = ver('vision');
    end
        
    properties(Dependent)
        FileName; % Had to add this for backward compatibility
        
        % The following properties are NOT mutually exclusive.
        % An empty session (with empty PatternSet and CameraParameters) can 
        % be either stereo or single camera. 
        IsValidStereoCameraSession;
        IsValidSingleCameraSession;
    end
    
    methods
        
        %------------------------------------------------------------------
        function fileName = get.FileName(this)
            fileName = this.Filename;
        end
        
        %------------------------------------------------------------------
        function set.FileName(this, fileName)
            this.Filename = fileName;
        end
        
        %------------------------------------------------------------------
        function tf = get.IsValidStereoCameraSession(this)
            tf = (isempty(this.CameraParameters) || ...
                  isa(this.CameraParameters, 'stereoParameters')) && ...
                 (isempty(this.PatternSet) || ...
                  size(this.PatternSet.FullPathNames, 1) == 2);
        end
        
        %------------------------------------------------------------------
        function tf = get.IsValidSingleCameraSession(this)
            tf = (isempty(this.CameraParameters) || ...
                 (isa(this.CameraParameters, 'cameraParameters')  || ...
                 isa(this.CameraParameters, 'fisheyeParameters'))) && ...
                 (isempty(this.PatternSet) || ...
                  size(this.PatternSet.FullPathNames, 1) == 1);
        end
            
        %------------------------------------------------------------------
        % return true if the tool went though the calibration
        %------------------------------------------------------------------
        function ret = isCalibrated(this)
            ret = ~isempty(this.CameraParameters);
        end

        function ret = hasAnyBoards(this)
            ret = ~isempty(this.PatternSet) && ...
                    this.PatternSet.NumPatterns ~= 0;
        end        
        
        %------------------------------------------------------------------
        function reset(this)
            
            this.ExtrinsicsView = 'CameraCentric';
            this.FileName = [];
            this.CanExport = false;
            this.HasEnoughBoards = false;
            this.CameraParameters = [];
            this.IsChanged = false;
            this.IsFisheyeModel = false;
            this.IsFixedIntrinsics = false;
            this.StereoIntrinsics1 = [];
            this.StereoIntrinsics2 = [];
            this.StereoIntrinsicsVarName1 = [];
            this.StereoIntrinsicsVarName2 = [];
            this.CameraModel = [];
            this.OptimizationOptions = [];
            
            if ~isempty(this.PatternSet)
                this.PatternSet.reset();
            end
        end
        
        %------------------------------------------------------------------
        % Wipes only the calibration portion of the session
        %------------------------------------------------------------------
        function resetCalibration(this)
            this.CanExport = false;
            this.CameraParameters = [];
        end
        
        %------------------------------------------------------------------
        function checkImagePaths(this, pathname, filename)
            if ~isempty(this.PatternSet)
                this.PatternSet.checkImagePaths(pathname, filename);
            end
        end
        
        %------------------------------------------------------------------
        function imagesUsed = calibrate(this, isFisheyeModel, isFixedIntrinsics, parent)
            % Ensure that image size is populated.
            this.PatternSet.updateImageSize();
            if nargin == 3
                parent = [];
            end
            
            if ~isFisheyeModel
                
                if isFixedIntrinsics
                    % run large baseline estimation function
                    [cameraParams, imagesUsed, estimationErrors] = ...
                        estimateStereoBaseline(this.PatternSet.PatternPoints, ...
                        this.PatternSet.WorldPoints, ...
                        this.StereoIntrinsics1, this.StereoIntrinsics2, ...
                        parent, 'WorldUnits', this.PatternSet.Units);
                else                   
                    % run standard calibration function
                    if isempty(this.OptimizationOptions) || ...
                            isempty(this.OptimizationOptions.InitialDistortion)
                        numRadial = this.CameraModel.NumDistortionCoefficients;                
                    else
                        numRadial = numel(this.OptimizationOptions.InitialDistortion);
                    end

                    if ~isempty(this.OptimizationOptions)
                        initIntrinsics = this.OptimizationOptions.InitialIntrinsics;
                        initDistortion = this.OptimizationOptions.InitialDistortion;
                    else
                        initIntrinsics = [];
                        initDistortion = [];
                    end
                    
                    try
                        [cameraParams, imagesUsed, estimationErrors] = ...
                            estimateCameraParameters(this.PatternSet.PatternPoints, ...
                            this.PatternSet.WorldPoints, parent,...
                            'EstimateSkew', this.CameraModel.ComputeSkew, ...
                            'EstimateTangentialDistortion', ...
                            this.CameraModel.ComputeTangentialDistortion, ...
                            'NumRadialDistortionCoefficients', numRadial, ...
                            'WorldUnits', this.PatternSet.Units, ...
                            'ShowProgressBar', true, ...
                            'InitialIntrinsicMatrix', initIntrinsics, ...
                            'InitialRadialDistortion', initDistortion, ...
                            'ImageSize', this.PatternSet.ImageSize);
                    catch except
                        % If user closes waitbar, catch here, and rethrow a
                        % cleaner error message.
                        if strcmp(except.identifier, 'MATLAB:waitbar:InvalidSecondInput')
                            errmsgID = 'vision:caltool:CanceledCalibration';
                            err.identifier = errmsgID; 
                            err.message = vision.getMessage(errmsgID);
                        else
                            err.identifier = except.identifier;
                            err.message = except.message;
                        end
                        rethrow(err)
                    end
                end
                this.IsFisheyeModel = false;
            else
                if ~isempty(this.CameraModel.EstimateAlignment)
                    estimateAlignment = this.CameraModel.EstimateAlignment;
                else                
                    estimateAlignment = false;
                end
                
                % run fisheye calibration function
                try
                    [cameraParams, imagesUsed, estimationErrors] = ...
                        estimateFisheyeParameters(this.PatternSet.PatternPoints, ...
                        this.PatternSet.WorldPoints, this.PatternSet.ImageSize, ...
                        parent, 'EstimateAlignment', estimateAlignment, ...
                        'ShowProgressBar', true, ...
                        'WorldUnits', this.PatternSet.Units);
                catch except
                    % If user closes waitbar, catch here, and rethrow a
                    % cleaner error message.
                    if strcmp(except.identifier, 'MATLAB:waitbar:InvalidSecondInput')
                        errmsgID = 'vision:caltool:CanceledCalibration';
                        err.identifier = errmsgID; 
                        err.message = vision.getMessage(errmsgID);
                    else
                        err.identifier = except.identifier;
                        err.message = except.message;
                    end
                    rethrow(err)
                end
                this.IsFisheyeModel = true;
            end
            
            this.CanExport = true;
            this.IsChanged = true;
            this.CameraParameters = cameraParams;
            this.EstimationErrors = estimationErrors;
        end
        
        %------------------------------------------------------------------
        function codeString = generateCode(this)
            if isa(this.CameraParameters, 'stereoParameters')
                codeString = generateCodeStereo(this);
            else
                codeString = generateCodeSingle(this);
            end
        end
    end
    
    methods(Access=private)
        %------------------------------------------------------------------
        function codeString = generateCodeSingle(this)
            cameraModel = this.CameraModel;
            codeGenerator = vision.internal.calibration.tool.MCodeGenerator;            

            % Write a header
            codeGenerator.addHeader('cameraCalibrator');

            % Detect calibration patterns
            codeGenerator.addReturn();
            codeGenerator.addComment('Define images to process'); 
            codeGenerator.addLine(sprintf('imageFileNames = %s;',...
                cell2str(this.PatternSet.FullPathNames)));

            codeGenerator.addComment('Detect calibration pattern in images');
            modifiedPatternDetectorFile = replace(this.PatternSet.PatternDetectorFile,".internal",""); % App container Version
            codeGenerator.addLine(['detector = ', modifiedPatternDetectorFile, '();']);

            if isa(this.PatternSet, 'vision.internal.calibration.tool.BoardSet') % checkerboard
                minCornerMetric = this.PatternSet.PatternDetector.MinCornerMetric;
                codeGenerator.addLine(sprintf('minCornerMetric = %f;', minCornerMetric));

                otherInputs = ', ''MinCornerMetric'', minCornerMetric';
                if this.PatternSet.IsDistortionHigh
                    % Set HighDistortion to true. This setting is typically
                    % used to handle images taken with fisheye lens.
                    otherInputs = [otherInputs ', ''HighDistortion'', true'];
                end
                otherInputs = [otherInputs ');']; 
            elseif isa(this.PatternSet.PatternDetector, 'vision.internal.calibration.webTool.CircleGridDetectorImpl') % circle grid patterns
                patternDims = this.PatternSet.PatternDetector.PatternDims;
                codeGenerator.addLine(sprintf('patternDims = [%d, %d];', patternDims(1), patternDims(2)));
                circleColor = this.PatternSet.PatternDetector.CircleColor;
                codeGenerator.addLine(sprintf('circleColor = "%s";', circleColor));
                otherInputs = ', ''PatternDims'', patternDims, ''CircleColor'', circleColor);';
            elseif isa(this.PatternSet.PatternDetector, 'vision.internal.calibration.webTool.CharucoBoardDetectorImpl') % charuco board
                patternDims = this.PatternSet.PatternDetector.PatternDims;
                markerFamily = this.PatternSet.PatternDetector.MarkerFamily;
                checkerSize = this.PatternSet.PatternDetector.CheckerSize;
                markerSize = this.PatternSet.PatternDetector.MarkerSize;
                minMarkerID = this.PatternSet.PatternDetector.MinMarkerID;
                originCheckerColor = this.PatternSet.PatternDetector.OriginCheckerColor;
                units = this.PatternSet.PatternDetector.WorldUnits;
                
                codeGenerator.addLine(sprintf('patternDims = [%d, %d];', patternDims(1), patternDims(2)));
                codeGenerator.addLine(sprintf('markerFamily = ''%s'';', markerFamily));
                codeGenerator.addLine(sprintf('checkerSize = %f;  %% in %s', checkerSize, units));
                codeGenerator.addLine(sprintf('markerSize = %f;  %% in %s', markerSize, units));
                codeGenerator.addLine(sprintf('minMarkerID = %d;', minMarkerID));
                codeGenerator.addLine(sprintf('originCheckerColor = ''%s'';', originCheckerColor));
                otherInputs = [', patternDims, markerFamily, checkerSize, markerSize', ...
                    ', ''MinMarkerID'', minMarkerID, ''OriginCheckerColor'', originCheckerColor);'];
            elseif isa(this.PatternSet.PatternDetector, 'vision.internal.calibration.webTool.AprilGridDetectorImpl') % aprilgrid
                patternDims = this.PatternSet.PatternDetector.PatternDims;
                tagFamily = this.PatternSet.PatternDetector.TagFamily;
                minTagID = this.PatternSet.PatternDetector.MinTagID;
                numBorderBits = this.PatternSet.PatternDetector.NumBorderBits;
                
                codeGenerator.addLine(sprintf('patternDims = [%d, %d];', patternDims(1), patternDims(2)));
                codeGenerator.addLine(sprintf('tagFamily = ''%s'';', tagFamily));
                codeGenerator.addLine(sprintf('minTagID = %d;', minTagID));
                codeGenerator.addLine(sprintf('numBorderBits = %d;', numBorderBits));
                otherInputs = [', patternDims, tagFamily', ...
                    ', ''MinTagID'', minTagID, ''NumBorderBits'', numBorderBits);'];
            else % custom patterns
                otherInputs = ');';
            end
            codeGenerator.addLine(['[imagePoints, imagesUsed] = detectPatternPoints(detector, imageFileNames', otherInputs]);
            codeGenerator.addLine('imageFileNames = imageFileNames(imagesUsed);');
            codeGenerator.addReturn();
            codeGenerator.addComment('Read the first image to obtain image size');
            codeGenerator.addLine('originalImage = imread(imageFileNames{1});');
            codeGenerator.addLine('[mrows, ncols, ~] = size(originalImage);');

            % Set up data for the calibration
            codeGenerator.addReturn();
            codeGenerator.addComment('Generate world coordinates for the planar pattern keypoints');
            
            if isa(this.PatternSet, 'vision.internal.calibration.tool.BoardSet') % checkerboard
                codeGenerator.addLine(sprintf('squareSize = %f;  %% in %s',...
                    this.PatternSet.PatternDetector.SquareSize,...
                    this.PatternSet.PatternDetector.WorldUnits));
                otherInputs = ', ''SquareSize'', squareSize);';
            elseif isa(this.PatternSet.PatternDetector, 'vision.internal.calibration.webTool.CircleGridDetectorImpl') % circle grid patterns
                codeGenerator.addLine(sprintf('centerDistance = %f;  %% in %s',...
                    this.PatternSet.PatternDetector.CenterDistance,...
                    this.PatternSet.PatternDetector.WorldUnits));
                otherInputs = ', ''PatternDims'', patternDims, ''CenterDistance'', centerDistance);';
            elseif isa(this.PatternSet.PatternDetector, 'vision.internal.calibration.webTool.CharucoBoardDetectorImpl') % charuco board pattern
                otherInputs = ', ''PatternDims'', patternDims, ''CheckerSize'', checkerSize);';
            elseif isa(this.PatternSet.PatternDetector, 'vision.internal.calibration.webTool.AprilGridDetectorImpl') % aprilgrid pattern
                tagSize = this.PatternSet.PatternDetector.TagSize;
                tagSpacing = this.PatternSet.PatternDetector.TagSpacing;
                units = this.PatternSet.PatternDetector.WorldUnits;
                codeGenerator.addLine(sprintf('tagSize = %f;  %% in %s', tagSize, units));
                codeGenerator.addLine(sprintf('tagSpacing = %f;  %% in %s', tagSpacing, units));
                otherInputs = ', ''PatternDims'', patternDims, ''TagSize'', tagSize, ''TagSpacing'', tagSpacing);';
            else % custom patterns
                otherInputs = ');';
            end
            codeGenerator.addLine(['worldPoints = generateWorldPoints(detector', otherInputs]);
            
            % Calibrate
            if isempty(this.OptimizationOptions)
                initIntrinsics = [];
                initDistortion = [];
            else
                initIntrinsics = this.OptimizationOptions.InitialIntrinsics;
                initDistortion = this.OptimizationOptions.InitialDistortion;
            end
            
            % Branch based on camera model
            if this.IsFisheyeModel
                % If fisheye, use the estimateFisheyeParameters function
                codeGenerator.addReturn();
                codeGenerator.addComment('Calibrate the camera using fisheye parameters');
                codeGenerator.addLine(sprintf(['[cameraParams, imagesUsed, estimationErrors] = estimateFisheyeParameters(imagePoints, worldPoints, ...\n',...
                    '[mrows, ncols], ...\n', ...
                    '''EstimateAlignment'', %s, ...\n', ...
                    '''WorldUnits'', ''%s'');'], ...
                    mat2str(cameraModel.EstimateAlignment), ...
                    this.PatternSet.Units));
                
            else
                % If not, use the estimateCameraParameters function
                codeGenerator.addReturn();
                codeGenerator.addComment('Calibrate the camera');
                codeGenerator.addLine(sprintf(['[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...\n',...
                    '''EstimateSkew'', %s, ''EstimateTangentialDistortion'', %s, ...\n', ...
                    '''NumRadialDistortionCoefficients'', %d, ''WorldUnits'', ''%s'', ...\n', ...
                    '''InitialIntrinsicMatrix'', %s, ''InitialRadialDistortion'', %s, ...\n', ...
                    '''ImageSize'', [mrows, ncols]);'], ...
                    mat2str(cameraModel.ComputeSkew), ...
                    mat2str(cameraModel.ComputeTangentialDistortion),...
                    cameraModel.NumDistortionCoefficients, this.PatternSet.Units, ...
                    mat2str(initIntrinsics), mat2str(initDistortion)));
            end
            
            % Add visualizations
            %-------------------
            
            % Reprojection errors
            codeGenerator.addReturn();
            codeGenerator.addComment('View reprojection errors');
            codeGenerator.addLine('h1=figure; showReprojectionErrors(cameraParams);');
            
            % Extrinsics
            codeGenerator.addReturn();
            codeGenerator.addComment('Visualize pattern locations');
            codeGenerator.addLine(sprintf('h2=figure; showExtrinsics(cameraParams, ''%s'');', ...
                this.ExtrinsicsView));

            % Estimation errors
            codeGenerator.addReturn();
            codeGenerator.addComment('Display parameter estimation errors');
            codeGenerator.addLine('displayErrors(estimationErrors, cameraParams);');

            % Suggest possible next steps
            if this.IsFisheyeModel
                codeGenerator.addReturn();
                codeGenerator.addComment('For example, you can use the calibration data to remove effects of lens distortion.');
                codeGenerator.addLine('undistortedImage = undistortFisheyeImage(originalImage, cameraParams.Intrinsics);');                 
            else
                codeGenerator.addReturn();
                codeGenerator.addComment('For example, you can use the calibration data to remove effects of lens distortion.');
                codeGenerator.addLine('undistortedImage = undistortImage(originalImage, cameraParams);'); 
            end
            codeGenerator.addReturn();
            codeGenerator.addComment('See additional examples of how to use the calibration data.  At the prompt type:');
            codeGenerator.addLine('% showdemo(''MeasuringPlanarObjectsExample'')');
            codeGenerator.addLine('% showdemo(''StructureFromMotionExample'')');
            
            % Terminate the file with carriage return
            codeGenerator.addReturn();
            
            codeString = codeGenerator.CodeString;            
        end
        
        %------------------------------------------------------------------
        function codeString = generateCodeStereo(this)
            cameraModel = this.CameraModel;
            codeGenerator = vision.internal.calibration.tool.MCodeGenerator;    
            
            % Write a header
            codeGenerator.addHeader('stereoCalibrator');

            % Detect calibration patterns
            codeGenerator.addReturn();
            codeGenerator.addComment('Define images to process'); 
            codeGenerator.addLine(sprintf('imageFileNames1 = %s;',...
                cell2str(this.PatternSet.FullPathNames(1, :))));
            codeGenerator.addLine(sprintf('imageFileNames2 = %s;',...
                cell2str(this.PatternSet.FullPathNames(2, :))));
            codeGenerator.addReturn();

            codeGenerator.addComment('Detect calibration pattern in images');
            modifiedPatternDetectorFile = replace(this.PatternSet.PatternDetectorFile,".internal",""); % App container Version
            codeGenerator.addLine(['detector = ', modifiedPatternDetectorFile, '();']);
            
            if isa(this.PatternSet, 'vision.internal.calibration.tool.BoardSet') % checkerboard
                minCornerMetric = this.PatternSet.PatternDetector.MinCornerMetric;
                codeGenerator.addLine(sprintf('minCornerMetric = %f;', minCornerMetric));

                otherInputs = ', ''MinCornerMetric'', minCornerMetric';
                if this.PatternSet.IsDistortionHigh
                    % Set HighDistortion to true.
                    otherInputs = [otherInputs ', ''HighDistortion'', true'];
                end
                otherInputs = [otherInputs ');'];
            elseif isa(this.PatternSet.PatternDetector, 'vision.internal.calibration.webTool.CircleGridDetectorImpl') % circle grid patterns
                patternDims = this.PatternSet.PatternDetector.PatternDims;
                codeGenerator.addLine(sprintf('patternDims = [%d, %d];', patternDims(1), patternDims(2)));
                circleColor = this.PatternSet.PatternDetector.CircleColor;
                codeGenerator.addLine(sprintf('circleColor = "%s";', circleColor));
                otherInputs = ', ''PatternDims'', patternDims, ''CircleColor'', circleColor);';
            elseif isa(this.PatternSet.PatternDetector, 'vision.internal.calibration.webTool.CharucoBoardDetectorImpl') % charuco board
                patternDims = this.PatternSet.PatternDetector.PatternDims;
                markerFamily = this.PatternSet.PatternDetector.MarkerFamily;
                checkerSize = this.PatternSet.PatternDetector.CheckerSize;
                markerSize = this.PatternSet.PatternDetector.MarkerSize;
                minMarkerID = this.PatternSet.PatternDetector.MinMarkerID;
                originCheckerColor = this.PatternSet.PatternDetector.OriginCheckerColor;
                units = this.PatternSet.PatternDetector.WorldUnits;
                
                codeGenerator.addLine(sprintf('patternDims = [%d, %d];', patternDims(1), patternDims(2)));
                codeGenerator.addLine(sprintf('markerFamily = ''%s'';', markerFamily));
                codeGenerator.addLine(sprintf('checkerSize = %f;  %% in %s', checkerSize, units));
                codeGenerator.addLine(sprintf('markerSize = %f;  %% in %s', markerSize, units));
                codeGenerator.addLine(sprintf('minMarkerID = %d;', minMarkerID));
                codeGenerator.addLine(sprintf('originCheckerColor = ''%s'';', originCheckerColor));
                otherInputs = [', patternDims, markerFamily, checkerSize, markerSize', ...
                    ', ''MinMarkerID'', minMarkerID, ''OriginCheckerColor'', originCheckerColor);'];
            elseif isa(this.PatternSet.PatternDetector, 'vision.internal.calibration.webTool.AprilGridDetectorImpl') % aprilgrid
                patternDims = this.PatternSet.PatternDetector.PatternDims;
                tagFamily = this.PatternSet.PatternDetector.TagFamily;
                minTagID = this.PatternSet.PatternDetector.MinTagID;
                numBorderBits = this.PatternSet.PatternDetector.NumBorderBits;
                
                codeGenerator.addLine(sprintf('patternDims = [%d, %d];', patternDims(1), patternDims(2)));
                codeGenerator.addLine(sprintf('tagFamily = ''%s'';', tagFamily));
                codeGenerator.addLine(sprintf('minTagID = %d;', minTagID));
                codeGenerator.addLine(sprintf('numBorderBits = %d;', numBorderBits));
                otherInputs = [', patternDims, tagFamily', ...
                    ', ''MinTagID'', minTagID, ''NumBorderBits'', numBorderBits);'];
            else % custom patterns
                otherInputs = ');';
            end
            codeGenerator.addLine(['[imagePoints, imagesUsed] = detectPatternPoints(detector, imageFileNames1, imageFileNames2', otherInputs]);
            
            % Set up data for the calibration
            codeGenerator.addReturn();
            codeGenerator.addComment('Generate world coordinates for the planar pattern keypoints');
            
            if isa(this.PatternSet, 'vision.internal.calibration.tool.BoardSet') % checkerboard
                codeGenerator.addLine(sprintf('squareSize = %f;  %% in %s', ...
                    this.PatternSet.PatternDetector.SquareSize,...
                    this.PatternSet.PatternDetector.WorldUnits));
                otherInputs = ', ''SquareSize'', squareSize);';
            elseif isa(this.PatternSet.PatternDetector, 'vision.internal.calibration.webTool.CircleGridDetectorImpl')% circle grid patterns
                codeGenerator.addLine(sprintf('centerDistance = %f;  %% in %s', ...
                    this.PatternSet.PatternDetector.CenterDistance,...
                    this.PatternSet.PatternDetector.WorldUnits));
                otherInputs = ', ''PatternDims'', patternDims, ''CenterDistance'', centerDistance);';
            elseif isa(this.PatternSet.PatternDetector, 'vision.internal.calibration.webTool.CharucoBoardDetectorImpl') % charuco board pattern
                otherInputs = ', ''PatternDims'', patternDims, ''CheckerSize'', checkerSize);';
            elseif isa(this.PatternSet.PatternDetector, 'vision.internal.calibration.webTool.AprilGridDetectorImpl') % aprilgrid pattern
                tagSize = this.PatternSet.PatternDetector.TagSize;
                tagSpacing = this.PatternSet.PatternDetector.TagSpacing;
                units = this.PatternSet.PatternDetector.WorldUnits;
                codeGenerator.addLine(sprintf('tagSize = %f;  %% in %s', tagSize, units));
                codeGenerator.addLine(sprintf('tagSpacing = %f;  %% in %s', tagSpacing, units));
                otherInputs = ', ''PatternDims'', patternDims, ''TagSize'', tagSize, ''TagSpacing'', tagSpacing);';
            else % custom patterns
                otherInputs = ');';
            end
            codeGenerator.addLine(['worldPoints = generateWorldPoints(detector', otherInputs]);
            
            % Read first image
            codeGenerator.addReturn();
            codeGenerator.addComment('Read one of the images from the first stereo pair');
            codeGenerator.addLine('I1 = imread(imageFileNames1{1});');
            
            % Branch based on intrinsics being pre-computed or not
            if this.IsFixedIntrinsics
                % Using pre-computed intrinsics
                codeGenerator.addReturn();
                codeGenerator.addComment('Create camera intrinsics to be used for calibration');
                codeGenerator.addReturn();
                codeGenerator.addComment('Check if camera intrinsics object for first camera is still in the workspace');
                codeGenerator.addLine(sprintf('if exist(''%s'', ''var'')', this.StereoIntrinsicsVarName1));
                    codeGenerator.addLine(sprintf('camera1Intrinsics = %s;', this.StereoIntrinsicsVarName1));
                codeGenerator.addLine(sprintf('else'));
                    codeGenerator.addComment('Recreate intrinsics from stored parameters');
                    codeGenerator.addLine(sprintf('\tcamera1Intrinsics = cameraIntrinsics([%f, %f], [%f, %f], [%d, %d], ...', ...
                                                 this.StereoIntrinsics1.FocalLength, ...
                                                 this.StereoIntrinsics1.PrincipalPoint, ...
                                                 this.StereoIntrinsics1.ImageSize));
                    if numel(this.StereoIntrinsics1.RadialDistortion) == 3
                        codeGenerator.addLine(sprintf('\t\t''RadialDistortion'', [%f, %f, %f], ...', ...
                                                      this.StereoIntrinsics1.RadialDistortion));
                    else
                        codeGenerator.addLine(sprintf('\t\t''RadialDistortion'', [%f, %f], ...', ...
                                                       this.StereoIntrinsics1.RadialDistortion));
                    end
                    codeGenerator.addLine(sprintf('\t\t''TangentialDistortion'', [%f, %f], ...', ...
                                                  this.StereoIntrinsics1.TangentialDistortion));
                    codeGenerator.addLine(sprintf('\t\t''Skew'', %f);', ...
                                                  this.StereoIntrinsics1.Skew));
               codeGenerator.addLine('end');
               codeGenerator.addReturn();
               codeGenerator.addComment('Check if camera intrinsics object for second camera is still in the workspace');
               codeGenerator.addLine(sprintf('if exist(''%s'', ''var'')', this.StereoIntrinsicsVarName2));
                   codeGenerator.addLine(sprintf('camera2Intrinsics = %s;', this.StereoIntrinsicsVarName2));
               codeGenerator.addLine(sprintf('else'));               
                    codeGenerator.addLine(sprintf('\tcamera2Intrinsics = cameraIntrinsics([%f, %f], [%f, %f], [%d, %d], ...', ...
                                                   this.StereoIntrinsics2.FocalLength, ...
                                                   this.StereoIntrinsics2.PrincipalPoint, ...
                                                   this.StereoIntrinsics2.ImageSize));
                    if numel(this.StereoIntrinsics2.RadialDistortion) == 3
                        codeGenerator.addLine(sprintf('\t\t''RadialDistortion'', [%f, %f, %f], ...', ...
                                                       this.StereoIntrinsics2.RadialDistortion));
                    else
                        codeGenerator.addLine(sprintf('\t\t''RadialDistortion'', [%f, %f], ...', ...
                                                       this.StereoIntrinsics2.RadialDistortion));
                    end
                    codeGenerator.addLine(sprintf('\t\t''TangentialDistortion'', [%f, %f], ...', ...
                                                   this.StereoIntrinsics2.TangentialDistortion));
                    codeGenerator.addLine(sprintf('\t\t''Skew'', %f);', ...
                                                   this.StereoIntrinsics2.Skew));
                codeGenerator.addLine('end');
                codeGenerator.addReturn();
                
                % Now calibrate using these parameters
                codeGenerator.addComment('Calibrate the camera');
                codeGenerator.addLine(sprintf(['[stereoParams, pairsUsed, estimationErrors] = estimateStereoBaseline(imagePoints, worldPoints, ...\n',...
                    'camera1Intrinsics, camera2Intrinsics, ...\n', ...
                    '''WorldUnits'', ''%s'');'], ...
                    this.PatternSet.Units));
                
            else
                % Computing intrinsics along with stereoParams
                % Compute Image Size from first image
                codeGenerator.addLine('[mrows, ncols, ~] = size(I1);');

                % Calibrate
                if isempty(this.OptimizationOptions)
                    initIntrinsics = [];
                    initDistortion = [];
                else
                    initIntrinsics = this.OptimizationOptions.InitialIntrinsics;
                    initDistortion = this.OptimizationOptions.InitialDistortion;
                end
                codeGenerator.addReturn();
                codeGenerator.addComment('Calibrate the camera');
                codeGenerator.addLine(sprintf(['[stereoParams, pairsUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...\n',...
                    '''EstimateSkew'', %s, ''EstimateTangentialDistortion'', %s, ...\n', ...
                    '''NumRadialDistortionCoefficients'', %d, ''WorldUnits'', ''%s'', ...\n', ...
                    '''InitialIntrinsicMatrix'', %s, ''InitialRadialDistortion'', %s, ...\n', ...
                    '''ImageSize'', [mrows, ncols]);'], ...
                    mat2str(cameraModel.ComputeSkew), ...
                    mat2str(cameraModel.ComputeTangentialDistortion),...
                    cameraModel.NumDistortionCoefficients, this.PatternSet.Units, ...
                    mat2str(initIntrinsics), mat2str(initDistortion)));
                
            end

            % Add visualizations
            
            % Reprojection errors
            codeGenerator.addReturn();
            codeGenerator.addComment('View reprojection errors');            
            codeGenerator.addLine('h1=figure; showReprojectionErrors(stereoParams);');
            
            % Extrinsics
            codeGenerator.addReturn();
            codeGenerator.addComment('Visualize pattern locations');
            codeGenerator.addLine(sprintf('h2=figure; showExtrinsics(stereoParams, ''%s'');', ...
                this.ExtrinsicsView));
            
            % Estimation errors
            codeGenerator.addReturn();
            codeGenerator.addComment('Display parameter estimation errors');
            codeGenerator.addLine('displayErrors(estimationErrors, stereoParams);');
            
            % Suggest possible next steps
            codeGenerator.addReturn();
            codeGenerator.addComment('You can use the calibration data to rectify stereo images.');
            codeGenerator.addLine('I2 = imread(imageFileNames2{1});');
            codeGenerator.addLine('[J1, J2, reprojectionMatrix] = rectifyStereoImages(I1, I2, stereoParams);');
            codeGenerator.addReturn();
            codeGenerator.addComment('See additional examples of how to use the calibration data.  At the prompt type:');
            codeGenerator.addLine('% showdemo(''StereoCalibrationAndSceneReconstructionExample'')');
            codeGenerator.addLine('% showdemo(''DepthEstimationFromStereoVideoExample'')');
            
            % Terminate the file with carriage return
            codeGenerator.addReturn();
            codeString = codeGenerator.CodeString;
        end
    end
    
    %----------------------------------------------------------------------
    % saveobj and loadobj are implemented to ensure compatibility across
    % releases even if architecture of Session class changes
    methods (Hidden)
       
        function that = saveobj(this)            
            that.version                   = this.Version;
            that.cameraModel               = this.CameraModel;
            that.standardCameraModelUI     = this.StandardCameraModelUI;
            that.fisheyeCameraModelUI      = this.FisheyeCameraModelUI;
            that.isFisheyeModel            = this.IsFisheyeModel;
            that.isFisheyeSelected         = this.IsFisheyeSelected;
            that.IsFixedIntrinsics         = this.IsFixedIntrinsics;
            that.StereoIntrinsics1         = this.StereoIntrinsics1;
            that.StereoIntrinsics2         = this.StereoIntrinsics2;
            that.StereoIntrinsicsVarName1  = this.StereoIntrinsicsVarName1;
            that.StereoIntrinsicsVarName2  = this.StereoIntrinsicsVarName2;
            that.optimizationOptions       = this.OptimizationOptions;
            that.cameraParams              = this.CameraParameters;
            that.estimationErrors          = this.EstimationErrors;
            that.shouldExportErrors        = this.ShouldExportErrors;
            that.patternSet                = this.PatternSet;
            that.hasEnoughBoards           = this.HasEnoughBoards;
            that.canExport                 = this.CanExport;
            that.isChanged                 = this.IsChanged;
            that.extrinsicsView            = this.ExtrinsicsView;
            that.errorsView                = this.ErrorsView;
            that.filename                  = this.FileName;
            that.exportVarName             = this.ExportVariableName;
        end
        
    end
    
    %----------------------------------------------------------------------
    methods (Static, Hidden)
       
        function this = loadobj(that)
            % The mustReturnObject warning is disabled in 'loadobj' method.
            % The disabled warning avoids the warning popup due to mismatch
            % of field from older version. The BoardIcons field is removed
            % in R2020b and further releases.
            % Retain the disabled warning state
            warning('on','MATLAB:class:mustReturnObject');
            
            if isa(that, 'vision.internal.calibration.tool.Session')
                this = that;
                this.OptimizationOptions.InitialIntrinsics = [];
                this.OptimizationOptions.InitialDistortion = [];
                
            else
                this = vision.internal.calibration.tool.Session;
                
                if isfield(that, 'cameraModel')
                    this.CameraModel        = that.cameraModel;
                elseif isfield(that, 'CameraModel')
                    this.CameraModel        = that.CameraModel;
                end
                
                if isfield(that, 'cameraParams')
                    this.CameraParameters   = that.cameraParams;
                elseif isfield(that, 'CameraParameters')
                    this.CameraParameters   = that.CameraParameters;
                end
                
                if isfield(that, 'estimationErrors')
                    this.EstimationErrors   = that.estimationErrors;
                elseif isfield(that, 'EstimationErrors')
                    this.EstimationErrors   = that.EstimationErrors;
                end
                
                if isfield(that, 'shouldExportErrors')
                    this.ShouldExportErrors = that.shouldExportErrors;
                elseif isfield(that, 'ShouldExportErrors')
                    this.ShouldExportErrors = that.ShouldExportErrors;
                end
                
                if isfield(that,'boardSet')
                    this.PatternSet      = that.boardSet;
                elseif isfield(that, 'BoardSet')
                    this.PatternSet      = that.BoardSet;
                else
                    this.PatternSet      = that.patternSet;
                end
                
                if isfield(that, 'hasEnoughBoards')
                    this.HasEnoughBoards    = that.hasEnoughBoards;
                elseif isfield(that, 'HasEnoughBoards')
                    this.HasEnoughBoards    = that.HasEnoughBoards;
                end
                
                if isfield(that, 'canExport')
                    this.CanExport          = that.canExport;
                elseif isfield(that, 'CanExport')
                    this.CanExport          = that.CanExport;
                end
                
                if isfield(that, 'isChanged')
                    this.IsChanged          = that.isChanged;
                elseif isfield(that, 'IsChanged')
                    this.IsChanged          = that.IsChanged;
                end
                
                if isfield(that, 'extrinsicsView')
                    this.ExtrinsicsView     = that.extrinsicsView;
                elseif isfield(that, 'ExtrinsicsView')
                    this.ExtrinsicsView     = that.ExtrinsicsView;
                end
                
                if isfield(that, 'filename')
                    this.FileName           = that.filename;
                elseif isfield(that, 'FileName')
                    this.FileName           = that.FileName;
                end
                
                if isfield(that, 'exportVarName')
                    this.ExportVariableName = that.exportVarName;
                elseif isfield(that, 'ExportVariableName')
                    this.ExportVariableName = that.ExportVariableName;
                end
                
                if isfield(that, 'standardCameraModelUI')
                    this.StandardCameraModelUI = that.standardCameraModelUI;
                elseif isfield(that, 'StandardCameraModelUI')
                    this.StandardCameraModelUI = that.StandardCameraModelUI;
                else
                    % Use default values.
                    this.StandardCameraModelUI.ComputeSkew = false;
                    this.StandardCameraModelUI.ComputeTangentialDistortion = false;
                    this.StandardCameraModelUI.NumDistortionCoefficients = 2;
                end
                
                if isfield(that, 'fisheyeCameraModelUI')
                    this.FisheyeCameraModelUI = that.fisheyeCameraModelUI;
                elseif isfield(that, 'FisheyeCameraModelUI')
                    this.FisheyeCameraModelUI = that.FisheyeCameraModelUI;
                else
                    % Use default values.
                    this.FisheyeCameraModelUI.EstimateAlignment = false;
                end
                
                if isfield(that, 'isFisheyeModel')
                    this.IsFisheyeModel = that.isFisheyeModel;
                elseif isfield(that, 'IsFisheyeModel')
                    this.IsFisheyeModel = that.IsFisheyeModel;
                else
                    this.IsFisheyeModel = false;
                end
                
                if isfield(that, 'isFisheyeSelected')
                    this.IsFisheyeSelected = that.isFisheyeSelected;
                elseif isfield(that, 'IsFisheyeSelected')
                    this.IsFisheyeSelected = that.IsFisheyeSelected;
                else
                    this.IsFisheyeSelected = false;
                end
                
                if isfield(that, 'IsFixedIntrinsics')
                    this.IsFixedIntrinsics = that.IsFixedIntrinsics;
                else
                    this.IsFixedIntrinsics = false;
                end
                
                if isfield(that, 'StereoIntrinsics1')
                    this.StereoIntrinsics1 = that.StereoIntrinsics1;
                else
                    this.StereoIntrinsics1 = [];
                end
                
                if isfield(that, 'StereoIntrinsics2')
                    this.StereoIntrinsics2 = that.StereoIntrinsics2;
                else
                    this.StereoIntrinsics2 = [];
                end
                
                if isfield(that, 'StereoIntrinsicsVarName1')
                    this.StereoIntrinsicsVarName1 = that.StereoIntrinsicsVarName1;
                else
                    this.StereoIntrinsics1 = '';
                end
                
                if isfield(that, 'StereoIntrinsicsVarName2')
                    this.StereoIntrinsicsVarName2 = that.StereoIntrinsicsVarName2;
                else
                    this.StereoIntrinsicsVarName2 = '';
                end
                
                if isfield(that, 'optimizationOptions')
                    this.OptimizationOptions = that.optimizationOptions;
                elseif isfield(that, 'OptimizationOptions')
                    this.OptimizationOptions = that.OptimizationOptions;
                else
                    this.OptimizationOptions.InitialIntrinsics = [];
                    this.OptimizationOptions.InitialDistortion = [];
                end
            end
        end
        
    end
end

%--------------------------------------------------------------
% This function handles conversion of cell array of strings
% to a string representing the entire cell array
%--------------------------------------------------------------
function str = cell2str(cellArray)
str = '{'; % opening bracket

% constants that are easier to read once assigned into
% variables
quote = '''';
nextLine = sprintf(',...\n');

for i=1:numel(cellArray)
    str = [str, quote, cellArray{i}, quote, nextLine]; %#ok<AGROW>
end

str = [str, '}']; % closing bracket
end
