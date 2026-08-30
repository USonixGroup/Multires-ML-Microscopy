% Multires_ML_Microscopy — Multi-resolution ML Microscopy Segmentation App
%
% Required MATLAB toolboxes:
%   - Image Processing Toolbox
%   - Deep Learning Toolbox
%   - Computer Vision Toolbox
%   - Statistics and Machine Learning Toolbox
%   - Wavelet Toolbox
%   - Parallel Computing Toolbox
%
% External dependencies:
%   - Bio-Formats for MATLAB       : Required for loading LSM image stacks
%
% External function dependencies (must be on the MATLAB path):
%   - segmentCells.m               : ML-based cell segmentation for single/multi-image input
%   - segmentFrame.m               : Single-frame segmentation with temporal tracking state
%   - segmentSobelWatershed.m      : Classical Sobel edge + Watershed segmentation
%   - createTracks.m               : IoU-based cell tracking across image sequences
%   - ExtractFeatures.m            : Morphological and intensity feature extraction per cell
%   - loadMicroglia3D.m            : Load 3D TIFF and LSM microscopy stacks
%   - preprocessMicroglia3D.m      : Preprocess 3D microglia image stacks
%   - segmentMicroglia3D.m         : Segment microglia in 3D image stacks
%   - removeXYBorderObjects3D.m    : Remove objects touching lateral image borders
%   - detectMicrogliaSomas3D.m     : Detect soma regions within segmented cells
%   - separateMicroglia3D.m        : Separate possible merged microglia
%   - extractMicrogliaFeatures3D.m : Extract 3D morphological features
%   - classifyMicroglia3D.m        : Classify microglia morphology
%
% Required model files (must be in the working directory or on the MATLAB path):
%   - EfficientNet.mat             : Default Mask R-CNN network, loaded on startup
%   - ResNet50.mat                 : Optional alternative network
%   - ResNet101.mat                : Optional alternative network
%   - CascadeEfficientNet.mat      : Optional alternative network
%   - Microglia_Classifier.mat     : Trained microglia morphology classifier
%
% Tested on MATLAB R2026a Update 4 (version 26.1)

classdef Multires_ML_Microscopy < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                  matlab.ui.Figure
        Panel                     matlab.ui.container.Panel
        Label_19                  matlab.ui.control.Label
        Label_18                  matlab.ui.control.Label
        Label_17                  matlab.ui.control.Label
        Label_16                  matlab.ui.control.Label
        Label_15                  matlab.ui.control.Label
        Label_14                  matlab.ui.control.Label
        SizeMax                   matlab.ui.control.NumericEditField
        ObjectSize                matlab.ui.control.RangeSlider
        Label_13                  matlab.ui.control.Label
        Label_11                  matlab.ui.control.Label
        Label_10                  matlab.ui.control.Label
        Label_9                   matlab.ui.control.Label
        Label_8                   matlab.ui.control.Label
        Label_7                   matlab.ui.control.Label
        Label_6                   matlab.ui.control.Label
        TrackingOption            matlab.ui.control.Switch
        Label_2                   matlab.ui.control.Label
        SizeMin                   matlab.ui.control.NumericEditField
        LevelEditFieldLabel_8     matlab.ui.control.Label
        CloseButton               matlab.ui.control.Button
        DWTThresh_t               matlab.ui.control.NumericEditField
        MinIOUTrack               matlab.ui.control.Slider
        IOUTrackText              matlab.ui.control.NumericEditField
        RPNOptAlphaText           matlab.ui.control.NumericEditField
        Alpha                     matlab.ui.control.Slider
        LevelEditFieldLabel_7     matlab.ui.control.Label
        LevelEditFieldLabel_6     matlab.ui.control.Label
        TrackingLabel_2           matlab.ui.control.Label
        DenoiseSwitch             matlab.ui.control.Switch
        MaxProposals              matlab.ui.control.NumericEditField
        LevelEditFieldLabel_5     matlab.ui.control.Label
        OverlapProp_t             matlab.ui.control.NumericEditField
        OverlaPP_t                matlab.ui.control.NumericEditField
        LevelEditFieldLabel_4     matlab.ui.control.Label
        LevelEditFieldLabel_3     matlab.ui.control.Label
        OverlapProp               matlab.ui.control.Slider
        OverlaPP                  matlab.ui.control.Slider
        DropDown                  matlab.ui.control.DropDown
        SegmentationNetworkLabel  matlab.ui.control.Label
        LevelEditFieldLabel_2     matlab.ui.control.Label
        DWTThresh                 matlab.ui.control.Slider
        DWTLEv                    matlab.ui.control.NumericEditField
        LevelEditFieldLabel       matlab.ui.control.Label
        PreProcessingLabel        matlab.ui.control.Label
        AdvancedLabel             matlab.ui.control.Label
        ConfidenceField           matlab.ui.control.NumericEditField
        Confidence                matlab.ui.control.Slider
        ConfidenceThresholdLabel  matlab.ui.control.Label
        RelaxedButton             matlab.ui.control.Button
        ConservativeButton        matlab.ui.control.Button
        DefaultButton             matlab.ui.control.Button
        PresetLabel               matlab.ui.control.Label
        SettingsLabel             matlab.ui.control.Label
        ExportPanel               matlab.ui.container.Panel
        JSONFileCheckBox          matlab.ui.control.CheckBox
        ExportButton              matlab.ui.control.Button
        matFileCheckBox           matlab.ui.control.CheckBox
        CSVFileCheckBox           matlab.ui.control.CheckBox
        CloseButton_2             matlab.ui.control.Button
        ExportDataLabel           matlab.ui.control.Label
        InitialLabel              matlab.ui.control.Label
        YFeatureLabel_2           matlab.ui.control.Label
        XFeatureLabel_2           matlab.ui.control.Label
        PlotIndex                 matlab.ui.control.NumericEditField
        PauseButton               matlab.ui.control.Button
        PlotFeaturesText          matlab.ui.control.Label
        NextButton                matlab.ui.control.Button
        PreviousButton            matlab.ui.control.Button
        PlotMode                  matlab.ui.control.Switch
        Reset                     matlab.ui.control.Button
        ImageDisp                 matlab.ui.control.Image
        ExportData_Button         matlab.ui.control.Button
        Credit                    matlab.ui.control.Hyperlink
        XDropDown                 matlab.ui.control.DropDown
        YDropDown                 matlab.ui.control.DropDown
        Number_of_Cells_Field     matlab.ui.control.NumericEditField
        Number_of_Cells_Text      matlab.ui.control.Label
        Segment_Button            matlab.ui.control.Button
        Help_Button               matlab.ui.control.Button
        Settings_Button           matlab.ui.control.Button
        UploadImagesButton        matlab.ui.control.Button
        ProgressBarAxes           matlab.ui.control.UIAxes
        HistogramAxes             matlab.ui.control.UIAxes

        % Sobel+Watershed parameter controls (visible only when that algorithm is selected)
        SW_Panel                  matlab.ui.container.Panel
        SW_TitleLabel             matlab.ui.control.Label
        SW_SigmaLabel             matlab.ui.control.Label
        SW_SigmaField             matlab.ui.control.NumericEditField
        SW_DiskSizeLabel          matlab.ui.control.Label
        SW_DiskSizeField          matlab.ui.control.DropDown
        SW_PolarityLabel          matlab.ui.control.Label
        SW_PolarityDropDown       matlab.ui.control.DropDown

        % Microglia3D main-window controls
        StackSliceLabel            matlab.ui.control.Label
        StackSliceSlider           matlab.ui.control.Slider
        MIPButton                  matlab.ui.control.Button
        SegmentedSliceLabel        matlab.ui.control.Label
        SegmentedSliceSlider       matlab.ui.control.Slider
        SegmentedMIPButton         matlab.ui.control.Button
        Show2DButton               matlab.ui.control.Button
        Show3DButton               matlab.ui.control.Button
        Classify3DButton           matlab.ui.control.Button

        % Microglia3D settings controls
        MG3D_Panel                 matlab.ui.container.Panel
        MG3D_TitleLabel            matlab.ui.control.Label
        MG3D_XVoxelLabel           matlab.ui.control.Label
        MG3D_XVoxelField           matlab.ui.control.NumericEditField
        MG3D_YVoxelLabel           matlab.ui.control.Label
        MG3D_YVoxelField           matlab.ui.control.NumericEditField
        MG3D_ZVoxelLabel           matlab.ui.control.Label
        MG3D_ZVoxelField           matlab.ui.control.NumericEditField
        MG3D_LowThresholdLabel     matlab.ui.control.Label
        MG3D_LowThresholdField     matlab.ui.control.NumericEditField
        MG3D_HighThresholdLabel    matlab.ui.control.Label
        MG3D_HighThresholdField    matlab.ui.control.NumericEditField
        MG3D_MinVolumeLabel        matlab.ui.control.Label
        MG3D_MinVolumeField        matlab.ui.control.NumericEditField
        MG3D_MinSomaLabel          matlab.ui.control.Label
        MG3D_MinSomaField          matlab.ui.control.NumericEditField
    end

    properties (Access = private)
        Images cell = {}; % Stores the uploaded image
        Filenames cell = {};
        CurrentImageIndex int8 = 1;
        SegmentedImages = []; % Stores the segmented image
        net = []; % Stores the pre-trained CNN network
        Masks cell = {};
        Boxes cell = {};
        Labels cell = {};
        Scores cell = {};
        SingleMasks = [];
        SingleBoxes = [];
        SingleScores = [];
        SingleLabels = [];
        AnalysisData = [];
        Progress = 0;
        TrackingInfo table = [];
        SingleImage logical = 0;
        AggregateAnalysis table = [];
        Tracks struct = [];
        CancelRequested logical = false; % Set by PauseButton to stop the segmentation loop cleanly

        % Microglia3D data
        Microglia3DFilename string = ""
        Microglia3DImage = []
        Microglia3DLabelVolume = []
        Microglia3DSomaMask = []
        Microglia3DMetadataX double = NaN
        Microglia3DMetadataY double = NaN
        Microglia3DMetadataZ double = NaN
        Microglia3DFeatures table = table()
        Microglia3DClassified table = table()
    end

    methods (Access = private)

        function [] = IMGDisplay(~, Image, ImageDisp) %function to display image file without labels properly

            targetH = 520; targetW = 740;   % match the ImageDisp panel size
            h = size(Image, 1); w = size(Image, 2);
            scale = min(targetH/h, targetW/w);
            if scale ~= 1
                Image = imresize(Image, scale, 'bilinear');
            end
            if size(Image, 3) == 1
                ImageDisp.ImageSource = repmat(im2uint8(Image), [1 1 3]);
            elseif size(Image, 3) == 3
                ImageDisp.ImageSource = im2uint8(Image);
            end
        end

        function UpdateProgress(app)
            barh(app.Progress, 'Parent', app.ProgressBarAxes, 'EdgeColor', 'none', ...
                 'FaceColor', [0.4660 0.6740 0.1880], 'BarWidth', 23000)
            if app.Progress > 0
                text(0.43, 0, num2str(app.Progress*100,'%.0f') + "%", ...
                     'Parent', app.ProgressBarAxes, 'FontSize', 20, 'FontWeight', 'bold')
            end
            drawnow
        end

        function highlightButton(app, selectedButton)
            allButtons = [app.DefaultButton, app.ConservativeButton, app.RelaxedButton];
            for btn = allButtons
                btn.BackgroundColor = [0.76 0.80 0.80];
                btn.FontColor = [0 0 0];
            end
            selectedButton.BackgroundColor = [0.05 0.30 0.30]; % Blue highlight
            selectedButton.FontColor = [1 1 1]; % White text
        end

        function UpdateIm(app)
            i = app.CurrentImageIndex;
            K = length(app.Scores{i});
        
            targetH = 520; targetW = 740;
            [natH, natW] = size(app.Images{i});
            scale = min(targetH/natH, targetW/natW);
            dispImg = imresize(app.Images{i}, scale, 'bilinear');
            dispImg = repmat(im2uint8(dispImg), [1 1 3]);
        
            if isempty(app.Masks{i}) || K == 0
                app.SegmentedImages = dispImg;
        
            elseif strcmp(app.DropDown.Value, 'Sobel+Watershed')
                % Masks{i} is an M×N uint32 label matrix — draw boundaries only,
                % exactly as the standalone script does, no M×N×K allocation
                W_fg   = app.Masks{i};
                W_disp = imresize(W_fg, scale, 'nearest');
        
                boundaries = boundarymask(W_disp); 
        
                Irgb = dispImg;
                redL = double(Irgb(:,:,1));
                grnL = double(Irgb(:,:,2));
                bluL = double(Irgb(:,:,3));
        
                redL(boundaries) = 255;
                grnL(boundaries) = 0;
                bluL(boundaries) = 0;
        
                Irgb(:,:,1) = uint8(redL);
                Irgb(:,:,2) = uint8(grnL);
                Irgb(:,:,3) = uint8(bluL);
                app.SegmentedImages = Irgb;
        
            else
                % Original ML path — unchanged
                DispColors = 1.4 - 0.5*rescale(app.Scores{i}) + 0.05*rand([K 1]);
                DispColors = DispColors .* [0.466 0.674 0.188];
        
                [dH, dW, ~] = size(dispImg);
                dispMasks = false(dH, dW, K);
                for k = 1:K
                    dispMasks(:,:,k) = imresize(app.Masks{i}(:,:,k), [dH dW], 'nearest');
                end
                app.SegmentedImages = insertObjectMask(dispImg, dispMasks, Color=DispColors);
            end
        
            IMGDisplay(app, app.SegmentedImages, app.ImageDisp); drawnow;
            app.Number_of_Cells_Field.Value = K;
        end

        % Show/hide the Sobel+Watershed parameter panel and the
        % ML-specific controls depending on the selected algorithm.
        function updateAlgorithmControls(app)
            % Show the controls required by the selected segmentation method.
            isSW = strcmp(app.DropDown.Value, 'Sobel+Watershed');
            isMG3D = strcmp(app.DropDown.Value, 'Microglia3D');

            if isSW
                app.SW_Panel.Visible = 'on';
            else
                app.SW_Panel.Visible = 'off';
            end

            mlVisibility = 'on';
            if isSW || isMG3D
                mlVisibility = 'off';
            end

            mlControls = { ...
                app.PreProcessingLabel, ...
                app.DenoiseSwitch, ...
                app.LevelEditFieldLabel, app.DWTLEv, ...
                app.LevelEditFieldLabel_2, app.DWTThresh_t, app.DWTThresh, ...
                app.Label_19, ...
                app.LevelEditFieldLabel_3, app.OverlaPP, app.OverlaPP_t, app.Label_9, ...
                app.LevelEditFieldLabel_4, app.OverlapProp, app.OverlapProp_t, app.Label_10, ...
                app.LevelEditFieldLabel_5, app.MaxProposals, ...
                app.LevelEditFieldLabel_8, app.ObjectSize, app.SizeMin, app.SizeMax, ...
                app.ConfidenceThresholdLabel, app.Confidence, app.ConfidenceField, app.Label_8, ...
                app.PresetLabel, app.DefaultButton, app.ConservativeButton, app.RelaxedButton, ...
                app.TrackingLabel_2, app.TrackingOption ...
            };

            for c = mlControls
                ctrl = c{1};
                ctrl.Visible = mlVisibility;
            end

            if isMG3D
                app.MG3D_Panel.Visible = 'on';
            else
                app.MG3D_Panel.Visible = 'off';

                % Restore the standard 2D layout.
                app.HistogramAxes.Position = [457 72 361 267];
                
                app.HistogramAxes.Box = 'on';
                app.HistogramAxes.Color = [1 1 1];
                app.HistogramAxes.XColor = [0.15 0.15 0.15];
                app.HistogramAxes.YColor = [0.15 0.15 0.15];
                app.ProgressBarAxes.Position = [38 395 397 37];
                app.StackSliceLabel.Visible = 'off';
                app.StackSliceSlider.Visible = 'off';
                app.MIPButton.Visible = 'off';
                app.SegmentedSliceLabel.Visible = 'off';
                app.SegmentedSliceSlider.Visible = 'off';
                app.SegmentedMIPButton.Visible = 'off';
                app.Show2DButton.Visible = 'off';
                app.Show3DButton.Visible = 'off';
                app.Classify3DButton.Visible = 'off';
            end
        end

        function updateMicrogliaVoxelFields(app)
            % Lock voxel sizes read from metadata and leave missing values editable.
            metadataValues = [ ...
                app.Microglia3DMetadataX, ...
                app.Microglia3DMetadataY, ...
                app.Microglia3DMetadataZ];

            fields = { ...
                app.MG3D_XVoxelField, ...
                app.MG3D_YVoxelField, ...
                app.MG3D_ZVoxelField};

            axisNames = {'X','Y','Z'};

            for k = 1:3
                if isfinite(metadataValues(k)) && metadataValues(k) > 0
                    fields{k}.Value = metadataValues(k);
                    fields{k}.Editable = 'off';
                    fields{k}.Tooltip = {sprintf( ...
                        '%s voxel size was read from image metadata and cannot be changed.', ...
                        axisNames{k})};
                else
                    fields{k}.Value = 0;
                    fields{k}.Editable = 'on';
                    fields{k}.Tooltip = {sprintf( ...
                        '%s voxel size was not found in image metadata. Enter it in micrometres.', ...
                        axisNames{k})};
                end
            end
        end

        function displayMicrogliaSlice(app, sliceNumber)
            if isempty(app.Microglia3DImage)
                return;
            end

            numberOfSlices = size(app.Microglia3DImage, 3);
            sliceNumber = max(1, min(numberOfSlices, round(sliceNumber)));

            app.StackSliceSlider.Value = sliceNumber;

            currentSlice = mat2gray( ...
                app.Microglia3DImage(:,:,sliceNumber));

            IMGDisplay(app, currentSlice, app.ImageDisp);

            app.StackSliceLabel.Text = sprintf( ...
                'Z %d/%d', sliceNumber, numberOfSlices);
        end

        function displaySegmentedMicrogliaSlice(app, sliceNumber)
            if isempty(app.Microglia3DLabelVolume) || isempty(app.Microglia3DImage)
                return;
            end

            numberOfSlices = size(app.Microglia3DLabelVolume, 3);
            sliceNumber = max(1, min(numberOfSlices, round(sliceNumber)));

            app.SegmentedSliceSlider.Value = sliceNumber;

            originalSlice = mat2gray( ...
                app.Microglia3DImage(:,:,sliceNumber));

            rgbImage = repmat( ...
                im2uint8(originalSlice), ...
                [1 1 3]);

            labelSlice = app.Microglia3DLabelVolume(:,:,sliceNumber);
            objectLabels = unique(labelSlice);
            objectLabels(objectLabels == 0) = [];

            % Before classification each object has a different colour
            % After classification the borders use the three class colours
            if ~isempty(objectLabels)

                if isempty(app.Microglia3DClassified)
                    objectColours = hsv(numel(objectLabels));

                    for objectIndex = 1:numel(objectLabels)
                        currentMask = ...
                            labelSlice == objectLabels(objectIndex);

                        boundary = bwperim(currentMask);

                        rgbImage = imoverlay( ...
                            rgbImage, ...
                            boundary, ...
                            objectColours(objectIndex,:));
                    end

                else
                    for objectIndex = 1:numel(objectLabels)
                        currentLabel = objectLabels(objectIndex);

                        rowIndex = find( ...
                            app.Microglia3DClassified.ObjectID == currentLabel, ...
                            1);

                        if isempty(rowIndex)
                            continue;
                        end

                        className = ...
                            app.Microglia3DClassified.PredictedMorphology(rowIndex);

                        classColour = ...
                            getMicrogliaClassColour(app, className);

                        currentMask = ...
                            labelSlice == currentLabel;

                        boundary = bwperim(currentMask);

                        rgbImage = imoverlay( ...
                            rgbImage, ...
                            boundary, ...
                            classColour);
                    end
                end
            end

            cla(app.HistogramAxes);
            imshow(rgbImage, 'Parent', app.HistogramAxes);

            app.HistogramAxes.Visible = 'on';
            app.HistogramAxes.XTick = [];
            app.HistogramAxes.YTick = [];
            app.HistogramAxes.XLabel.String = '';
            app.HistogramAxes.YLabel.String = '';
            title(app.HistogramAxes, '');

            app.SegmentedSliceLabel.Text = sprintf( ...
                'Z %d/%d', sliceNumber, numberOfSlices);
        end

        function showSegmentedMIP(app)
            if isempty(app.Microglia3DLabelVolume) || isempty(app.Microglia3DImage)
                return;
            end

            % If classification already exists, keep the classified colours
            if ~isempty(app.Microglia3DClassified)
                displayClassifiedMicrogliaMIP(app);
                return;
            end

            originalProjection = mat2gray( ...
                max(app.Microglia3DImage, [], 3));

            rgbImage = repmat( ...
                im2uint8(originalProjection), ...
                [1 1 3]);

            objectLabels = unique(app.Microglia3DLabelVolume);
            objectLabels(objectLabels == 0) = [];

            if ~isempty(objectLabels)
                objectColours = hsv(numel(objectLabels));

                for objectIndex = 1:numel(objectLabels)
                    objectMask = ...
                        app.Microglia3DLabelVolume == objectLabels(objectIndex);

                    objectProjection = max(objectMask, [], 3);
                    boundary = bwperim(objectProjection);

                    rgbImage = imoverlay( ...
                        rgbImage, ...
                        boundary, ...
                        objectColours(objectIndex,:));
                end
            end

            cla(app.HistogramAxes);
            imshow(rgbImage, 'Parent', app.HistogramAxes);

            app.HistogramAxes.Visible = 'on';
            app.HistogramAxes.XTick = [];
            app.HistogramAxes.YTick = [];
            app.HistogramAxes.XLabel.String = '';
            app.HistogramAxes.YLabel.String = '';
            title(app.HistogramAxes, '');

            app.SegmentedSliceLabel.Text = 'MIP';
        end

        function colour = getMicrogliaClassColour(~, className)
            % Fixed class colours used everywhere in the app:
            % Amoeboid  = red
            % Activated = yellow
            % Ramified  = green

            className = string(className);

            if className == "Amoeboid"
                colour = [1 0 0];

            elseif className == "Activated"
                colour = [1 1 0];

            elseif className == "Ramified"
                colour = [0 1 0];

            else
                colour = [1 1 1];
            end
        end

        function displayClassifiedMicrogliaMIP(app)
            if isempty(app.Microglia3DClassified) || ...
                    isempty(app.Microglia3DLabelVolume) || ...
                    isempty(app.Microglia3DImage)
                return;
            end

            originalProjection = mat2gray( ...
                max(app.Microglia3DImage, [], 3));

            rgbImage = repmat( ...
                im2uint8(originalProjection), ...
                [1 1 3]);

            objectLabels = unique(app.Microglia3DLabelVolume);
            objectLabels(objectLabels == 0) = [];

            for objectIndex = 1:numel(objectLabels)
                currentLabel = objectLabels(objectIndex);

                rowIndex = find( ...
                    app.Microglia3DClassified.ObjectID == currentLabel, ...
                    1);

                if isempty(rowIndex)
                    continue;
                end

                className = ...
                    app.Microglia3DClassified.PredictedMorphology(rowIndex);

                classColour = ...
                    getMicrogliaClassColour(app, className);

                objectMask = ...
                    app.Microglia3DLabelVolume == currentLabel;

                objectProjection = max(objectMask, [], 3);
                boundary = bwperim(objectProjection);

                rgbImage = imoverlay( ...
                    rgbImage, ...
                    boundary, ...
                    classColour);
            end

            cla(app.HistogramAxes);
            imshow(rgbImage, 'Parent', app.HistogramAxes);

            app.HistogramAxes.Visible = 'on';
            app.HistogramAxes.XTick = [];
            app.HistogramAxes.YTick = [];
            app.HistogramAxes.XLabel.String = '';
            app.HistogramAxes.YLabel.String = '';
            title(app.HistogramAxes, '');

            app.SegmentedSliceLabel.Text = 'MIP';
        end

        function addMicrogliaClassLegend(app, figureHandle)
            % Add colour code and class counts at the right side of a figure

            if isempty(app.Microglia3DClassified)
                return;
            end

            classNames = ["Amoeboid", "Activated", "Ramified"];
            classColours = [ ...
                1 0 0; ...
                1 1 0; ...
                0 1 0];

            classCounts = zeros(3,1);

            for classIndex = 1:3
                classCounts(classIndex) = sum( ...
                    app.Microglia3DClassified.PredictedMorphology == ...
                    classNames(classIndex));
            end

            annotation( ...
                figureHandle, ...
                'textbox', ...
                [0.76 0.68 0.21 0.08], ...
                'String', 'Classification', ...
                'FontWeight', 'bold', ...
                'FontSize', 12, ...
                'EdgeColor', 'none');

            yPositions = [0.59 0.50 0.41];

            for classIndex = 1:3
                annotation( ...
                    figureHandle, ...
                    'rectangle', ...
                    [0.77 yPositions(classIndex) 0.035 0.035], ...
                    'FaceColor', classColours(classIndex,:), ...
                    'EdgeColor', [0 0 0]);

                annotation( ...
                    figureHandle, ...
                    'textbox', ...
                    [0.815 yPositions(classIndex)-0.008 0.17 0.055], ...
                    'String', sprintf( ...
                        '%s: %d', ...
                        classNames(classIndex), ...
                        classCounts(classIndex)), ...
                    'FontSize', 11, ...
                    'EdgeColor', 'none');
            end

            annotation( ...
                figureHandle, ...
                'textbox', ...
                [0.77 0.30 0.20 0.07], ...
                'String', sprintf( ...
                    'Total cells: %d', ...
                    height(app.Microglia3DClassified)), ...
                'FontWeight', 'bold', ...
                'FontSize', 11, ...
                'EdgeColor', 'none');
        end

        function T = fixTableNames(~, T)
            T.Properties.VariableNames = matlab.lang.makeValidName( ...
                T.Properties.VariableNames);
        end

    end % private methods


    % Callbacks that handle component events
    methods (Access = private)

   
        function startupFcn(app)
         
            appFolder = fileparts(mfilename('fullpath'));

            possibleMicrogliaFolders = { ...
                fullfile(appFolder, 'Microglia_3D'), ...
                fullfile(appFolder, 'Multires_ML_Microscopy', 'Microglia_3D')};

            microgliaFolderFound = false;
            for folderIndex = 1:numel(possibleMicrogliaFolders)
                if isfolder(possibleMicrogliaFolders{folderIndex})
                    addpath(possibleMicrogliaFolders{folderIndex});
                    microgliaFolderFound = true;
                    break;
                end
            end

            if ~microgliaFolderFound
                warning('Microglia_3D folder was not found beside the app.');
            end

            IMGDisplay(app, ones(520, 740)*0.9, app.ImageDisp);
            loaded  = load('EfficientNet.mat', 'net');
            app.net = loaded.net;
            set(app.ProgressBarAxes, 'visible', 'off');
            set(app.ProgressBarAxes, 'xtick', []);
            set(app.ProgressBarAxes, 'ytick', []);
            UpdateProgress(app);
            set(app.ProgressBarAxes, 'XLim', [0 1]);
            DefaultButtonPushed(app);
            updateAlgorithmControls(app); % Ensure correct panel visibility matches default dropdown value
        end

        % Callback function: ImageDisp, UploadImagesButton
        function UploadImage(app, ~)
            app.InitialLabel.Visible = 'off';

            [files, path] = uigetfile( ...
                {'*.png;*.jpg;*.jpeg;*.tif;*.tiff;*.lsm', ...
                 'Microscopy Images (*.png, *.jpg, *.jpeg, *.tif, *.tiff, *.lsm)'}, ...
                'Select Images', ...
                'MultiSelect', 'on');

            if isequal(files, 0)
                return;
            end

            if ~iscell(files)
                files = {files};
            end

            % Detect a true 3D stack. Existing 2D files continue through
            % the original upload path below
            is3D = false;

            if isscalar(files)
                fullFilename = fullfile(path, files{1});
                [~,~,extension] = fileparts(fullFilename);

                if strcmpi(extension, '.lsm')
                    is3D = true;
                elseif strcmpi(extension, '.tif') || strcmpi(extension, '.tiff')
                    info = imfinfo(fullFilename);
                    is3D = numel(info) > 1;
                end
            else
                for k = 1:numel(files)
                    fullFilename = fullfile(path, files{k});
                    [~,~,extension] = fileparts(fullFilename);

                    if strcmpi(extension, '.lsm')
                        uialert(app.UIFigure, ...
                            'Please select a 3D stack by itself.', ...
                            '3D Stack Selection');
                        return;
                    elseif strcmpi(extension, '.tif') || strcmpi(extension, '.tiff')
                        info = imfinfo(fullFilename);
                        if numel(info) > 1
                            uialert(app.UIFigure, ...
                                'Please select a 3D stack by itself.', ...
                                '3D Stack Selection');
                            return;
                        end
                    end
                end
            end

            if is3D
                try
                    fullFilename = fullfile(path, files{1});

                    [imageStack, xMeta, yMeta, zMeta] = ...
                        loadMicroglia3D(fullFilename);

                    app.Microglia3DFilename = string(fullFilename);
                    app.Microglia3DImage = imageStack;
                    app.Microglia3DMetadataX = xMeta;
                    app.Microglia3DMetadataY = yMeta;
                    app.Microglia3DMetadataZ = zMeta;
                    app.Microglia3DLabelVolume = [];
                    app.Microglia3DSomaMask = [];
                    app.Microglia3DFeatures = table();
                    app.Microglia3DClassified = table();

                    % Keep 2D and 3D data separate
                    app.Images = {};
                    app.Filenames = {};
                    app.AnalysisData = {};

                    app.DropDown.Value = 'Microglia3D';
                    updateAlgorithmControls(app);
                    updateMicrogliaVoxelFields(app);

                    numberOfSlices = size(imageStack, 3);
                    middleSlice = ceil(numberOfSlices / 2);

                    app.StackSliceSlider.Limits = [1 max(2, numberOfSlices)];
                    app.StackSliceSlider.MajorTicks = [1 numberOfSlices];
                    app.StackSliceSlider.Value = middleSlice;

                    app.StackSliceLabel.Visible = 'on';
                    app.StackSliceSlider.Visible = 'on';
                    app.MIPButton.Visible = 'on';

                    app.SegmentedSliceLabel.Visible = 'off';
                    app.SegmentedSliceSlider.Visible = 'off';
                    app.SegmentedMIPButton.Visible = 'off';
                    app.Show2DButton.Visible = 'off';
                    app.Show3DButton.Visible = 'off';
                    app.Classify3DButton.Visible = 'off';

                    app.HistogramAxes.Visible = 'off';
                    cla(app.HistogramAxes);

                    displayMicrogliaSlice(app, middleSlice);

                    app.Segment_Button.Visible = 'on';
                    app.Number_of_Cells_Field.Visible = 'off';
                    app.Number_of_Cells_Text.Visible = 'off';

                    fprintf('\nLoaded 3D microscopy stack: %s\n', files{1});
                    fprintf('Stack size: %d x %d x %d\n', ...
                        size(imageStack,2), ...
                        size(imageStack,1), ...
                        numberOfSlices);

                catch ME
                    uialert(app.UIFigure, ...
                        ME.message, ...
                        '3D Image Loading Error');
                end

                return;
            end

            % Clear Microglia3D state before loading a 2D image
            app.Microglia3DFilename = "";
            app.Microglia3DImage = [];
            app.Microglia3DLabelVolume = [];
            app.Microglia3DSomaMask = [];
            app.Microglia3DMetadataX = NaN;
            app.Microglia3DMetadataY = NaN;
            app.Microglia3DMetadataZ = NaN;
            app.Microglia3DFeatures = table();
            app.Microglia3DClassified = table();

            app.StackSliceLabel.Visible = 'off';
            app.StackSliceSlider.Visible = 'off';
            app.MIPButton.Visible = 'off';
            app.SegmentedSliceLabel.Visible = 'off';
            app.SegmentedSliceSlider.Visible = 'off';
            app.SegmentedMIPButton.Visible = 'off';
            app.Show2DButton.Visible = 'off';
            app.Show3DButton.Visible = 'off';
            app.Classify3DButton.Visible = 'off';
            app.HistogramAxes.Position = [457 72 361 267];
            app.ProgressBarAxes.Position = [38 395 397 37];

            % Original 2D upload behaviour - intentionally unchanged
            app.Images = {};
            app.Filenames = {};

            for k = 1:length(files)
                raw = imread(fullfile(path, files{k}));
                if size(raw, 3) == 3
                    raw = rgb2gray(raw);
                end
                raw = im2double(raw);

                app.Images{end+1}    = raw;
                app.Filenames{end+1} = files{k};
            end

            app.CurrentImageIndex = 1;
            app.SegmentedImages   = {};
            app.Masks             = {};
            app.Boxes             = {};
            app.Labels            = {};
            app.Scores            = {};
            app.AnalysisData      = {};
            IMGDisplay(app, app.Images{1}, app.ImageDisp);
            app.Segment_Button.Visible = 'on';
        end

        function StackSliceSliderValueChanged(app, ~)
            displayMicrogliaSlice(app, app.StackSliceSlider.Value);
        end

        function StackSliceSliderValueChanging(app, event)
            displayMicrogliaSlice(app, event.Value);
        end

        function MIPButtonPushed(app, ~)
            if isempty(app.Microglia3DImage)
                return;
            end

            projection = mat2gray(max(app.Microglia3DImage, [], 3));
            IMGDisplay(app, projection, app.ImageDisp);
            app.StackSliceLabel.Text = 'MIP';
        end

        function SegmentedSliceSliderValueChanged(app, ~)
            displaySegmentedMicrogliaSlice( ...
                app, app.SegmentedSliceSlider.Value);
        end

        function SegmentedSliceSliderValueChanging(app, event)
            displaySegmentedMicrogliaSlice(app, event.Value);
        end

        function SegmentedMIPButtonPushed(app, ~)
            showSegmentedMIP(app);
        end

        function Show2DButtonPushed(app, ~)
            if isempty(app.Microglia3DLabelVolume)
                uialert(app.UIFigure, ...
                    'Run Microglia3D segmentation first.', ...
                    'No 3D Segmentation');
                return;
            end

            labelProjection = max( ...
                app.Microglia3DLabelVolume, ...
                [], ...
                3);

            maximumLabel = ...
                double(max(app.Microglia3DLabelVolume(:)));

            figureHandle = figure( ...
                'Name', '2D Microglia Segmentation', ...
                'NumberTitle', 'off', ...
                'Color', 'w', ...
                'Position', [100 100 900 620]);

            axesHandle = axes( ...
                'Parent', figureHandle, ...
                'Position', [0.05 0.08 0.68 0.86]);

            if isempty(app.Microglia3DClassified)
                % Normal segmentation view before classification
                if maximumLabel < 1
                    colouredMasks = zeros( ...
                        size(labelProjection,1), ...
                        size(labelProjection,2), ...
                        3);
                else
                    colouredMasks = label2rgb( ...
                        labelProjection, ...
                        hsv(maximumLabel), ...
                        'k');
                end

                imshow(colouredMasks, 'Parent', axesHandle);

                title(axesHandle, sprintf( ...
                    '2D Segmentation - %d Cells', ...
                    maximumLabel));

            else
                % Classified 2D view
                rgbImage = zeros( ...
                    size(labelProjection,1), ...
                    size(labelProjection,2), ...
                    3);

                objectLabels = unique(app.Microglia3DLabelVolume);
                objectLabels(objectLabels == 0) = [];

                for objectIndex = 1:numel(objectLabels)
                    currentLabel = objectLabels(objectIndex);

                    rowIndex = find( ...
                        app.Microglia3DClassified.ObjectID == currentLabel, ...
                        1);

                    if isempty(rowIndex)
                        continue;
                    end

                    className = ...
                        app.Microglia3DClassified.PredictedMorphology(rowIndex);

                    classColour = ...
                        getMicrogliaClassColour(app, className);

                    objectProjection = max( ...
                        app.Microglia3DLabelVolume == currentLabel, ...
                        [], ...
                        3);

                    for colourChannel = 1:3
                        channel = rgbImage(:,:,colourChannel);
                        channel(objectProjection) = ...
                            classColour(colourChannel);
                        rgbImage(:,:,colourChannel) = channel;
                    end
                end

                imshow(rgbImage, 'Parent', axesHandle);

                title(axesHandle, sprintf( ...
                    '2D Classified Microglia - %d Cells', ...
                    height(app.Microglia3DClassified)));

                addMicrogliaClassLegend(app, figureHandle);
            end
        end

        function Show3DButtonPushed(app, ~)
            if isempty(app.Microglia3DLabelVolume)
                uialert(app.UIFigure, ...
                    'Run Microglia3D segmentation first.', ...
                    'No 3D Segmentation');
                return;
            end

            objectLabels = unique(app.Microglia3DLabelVolume);
            objectLabels(objectLabels == 0) = [];

            numberOfObjects = numel(objectLabels);

            if numberOfObjects == 0
                return;
            end

            figureHandle = figure( ...
                'Name', '3D Microglia Segmentation', ...
                'NumberTitle', 'off', ...
                'Color', 'w', ...
                'Position', [100 100 950 650]);

            axesHandle = axes( ...
                'Parent', figureHandle, ...
                'Position', [0.06 0.10 0.68 0.82]);

            hold(axesHandle, 'on');

            xVoxelSize = app.MG3D_XVoxelField.Value;
            yVoxelSize = app.MG3D_YVoxelField.Value;
            zVoxelSize = app.MG3D_ZVoxelField.Value;

            if isempty(app.Microglia3DClassified)
                objectColours = hsv(numberOfObjects);
            end

            for objectIndex = 1:numberOfObjects
                currentLabel = objectLabels(objectIndex);

                objectMask = ...
                    app.Microglia3DLabelVolume == currentLabel;

                surfaceData = isosurface(objectMask, 0.5);

                if isempty(surfaceData.vertices)
                    continue;
                end

                vertices = surfaceData.vertices;

                vertices(:,1) = vertices(:,1) * xVoxelSize;
                vertices(:,2) = vertices(:,2) * yVoxelSize;
                vertices(:,3) = vertices(:,3) * zVoxelSize;

                if isempty(app.Microglia3DClassified)
                    currentColour = objectColours(objectIndex,:);
                else
                    rowIndex = find( ...
                        app.Microglia3DClassified.ObjectID == currentLabel, ...
                        1);

                    if isempty(rowIndex)
                        currentColour = [1 1 1];
                    else
                        className = ...
                            app.Microglia3DClassified.PredictedMorphology(rowIndex);

                        currentColour = ...
                            getMicrogliaClassColour(app, className);
                    end
                end

                patch( ...
                    axesHandle, ...
                    'Faces', surfaceData.faces, ...
                    'Vertices', vertices, ...
                    'FaceColor', currentColour, ...
                    'EdgeColor', 'none', ...
                    'FaceAlpha', 0.8);
            end

            xlabel(axesHandle, 'X (\mum)');
            ylabel(axesHandle, 'Y (\mum)');
            zlabel(axesHandle, 'Z (\mum)');

            if isempty(app.Microglia3DClassified)
                title(axesHandle, sprintf( ...
                    '3D Segmentation - %d Cells', ...
                    numberOfObjects));
            else
                title(axesHandle, sprintf( ...
                    '3D Classified Microglia - %d Cells', ...
                    height(app.Microglia3DClassified)));

                addMicrogliaClassLegend(app, figureHandle);
            end

            axis(axesHandle, 'equal');
            axis(axesHandle, 'tight');
            grid(axesHandle, 'on');
            view(axesHandle, 3);

            rotate3d(figureHandle, 'on');
            camlight(axesHandle, 'headlight');
            lighting(axesHandle, 'gouraud');

            hold(axesHandle, 'off');
        end

        function Classify3DButtonPushed(app, ~)
            if isempty(app.Microglia3DLabelVolume)
                uialert(app.UIFigure, ...
                    'Run Microglia3D segmentation first.', ...
                    'No 3D Segmentation');
                return;
            end

            if isempty(app.Microglia3DSomaMask)
                uialert(app.UIFigure, ...
                    'The soma mask is missing. Run segmentation again.', ...
                    'Missing Soma Data');
                return;
            end

            try
                app.Classify3DButton.Enable = 'off';
                drawnow;

                xVoxelSize = app.MG3D_XVoxelField.Value;
                yVoxelSize = app.MG3D_YVoxelField.Value;
                zVoxelSize = app.MG3D_ZVoxelField.Value;

                fprintf('\nExtracting 3D morphology features...\n');

                featureTable = extractMicrogliaFeatures3D( ...
                    app.Microglia3DLabelVolume, ...
                    app.Microglia3DSomaMask, ...
                    xVoxelSize, ...
                    yVoxelSize, ...
                    zVoxelSize);

                app.Microglia3DFeatures = featureTable;

                fprintf('\nClassifying microglia morphology...\n');

                classifiedTable = ...
                    classifyMicroglia3D(featureTable);

                app.Microglia3DClassified = ...
                    classifiedTable;

                % Update the right MIP using the morphology class colours
                displayClassifiedMicrogliaMIP(app);

                % Hide the classification button after prediction
                app.Classify3DButton.Visible = 'off';

            catch ME
                app.Classify3DButton.Enable = 'on';

                uialert(app.UIFigure, ...
                    ME.message, ...
                    'Microglia Classification Error');
            end
        end

        % Button pushed function: Segment_Button
        function Segment(app, ~)
            isMG3D = strcmp(app.DropDown.Value, 'Microglia3D');

            if isMG3D
                if isempty(app.Microglia3DImage) || ...
                        strlength(app.Microglia3DFilename) == 0

                    uialert(app.UIFigure, ...
                        'Please upload a 3D TIFF/TIFF or LSM stack first.', ...
                        'No 3D Stack');
                    return;
                end

                xVoxelSize = app.MG3D_XVoxelField.Value;
                yVoxelSize = app.MG3D_YVoxelField.Value;
                zVoxelSize = app.MG3D_ZVoxelField.Value;

                missingAxes = strings(0);

                if xVoxelSize <= 0
                    missingAxes(end+1) = "X";
                end
                if yVoxelSize <= 0
                    missingAxes(end+1) = "Y";
                end
                if zVoxelSize <= 0
                    missingAxes(end+1) = "Z";
                end

                if ~isempty(missingAxes)
                    uialert(app.UIFigure, ...
                        sprintf(['Voxel size information is missing for: %s.\n' ...
                        'Enter the missing value(s) in Settings before segmentation.'], ...
                        strjoin(missingAxes, ', ')), ...
                        'Missing Voxel Size');
                    return;
                end

                try
                    app.Segment_Button.Enable = 'off';

                    % Resize the progress display for the Microglia3D view
                    app.ProgressBarAxes.Position = [85 397 303 50];
                    app.ProgressBarAxes.Visible = 'on';

                    app.Progress = 0.05;
                    UpdateProgress(app);
                    drawnow;

                    % Fixed preprocessing values used by the final pipeline
                    preprocessSettings.gaussianSigmaXY = 1.0;
                    preprocessSettings.gaussianSigmaZ = 0.7;
                    preprocessSettings.backgroundRadius = 18;

                    segmentSettings.lowThresholdMultiplier = ...
                        app.MG3D_LowThresholdField.Value;
                    segmentSettings.highThresholdMultiplier = ...
                        app.MG3D_HighThresholdField.Value;
                    segmentSettings.minimumObjectVolume_um3 = ...
                        app.MG3D_MinVolumeField.Value;
                    segmentSettings.xVoxelSize = xVoxelSize;
                    segmentSettings.yVoxelSize = yVoxelSize;
                    segmentSettings.zVoxelSize = zVoxelSize;

                    somaSettings.somaCoreRadius_um = 1.5;
                    somaSettings.minimumSomaVolume_um3 = ...
                        app.MG3D_MinSomaField.Value;

                    fprintf('\nRunning Microglia3D preprocessing...\n');

                    [preprocessedStack, ~] = ...
                        preprocessMicroglia3D( ...
                        app.Microglia3DImage, ...
                        preprocessSettings);

                    app.Progress = 0.30;
                    UpdateProgress(app);
                    drawnow;

                    fprintf('\nRunning Microglia3D segmentation...\n');

                    [labelVolume, ~, ~] = ...
                        segmentMicroglia3D( ...
                        preprocessedStack, ...
                        segmentSettings);

                    app.Progress = 0.52;
                    UpdateProgress(app);
                    drawnow;

                    fprintf('\nRemoving XY-border objects...\n');

                    [labelVolume, ~] = ...
                        removeXYBorderObjects3D(labelVolume);

                    app.Progress = 0.65;
                    UpdateProgress(app);
                    drawnow;

                    fprintf('\nDetecting soma candidates...\n');

                    [somaMask, ~] = ...
                        detectMicrogliaSomas3D( ...
                        labelVolume, ...
                        xVoxelSize, ...
                        yVoxelSize, ...
                        zVoxelSize, ...
                        somaSettings);

                    app.Progress = 0.78;
                    UpdateProgress(app);
                    drawnow;

                    fprintf('\nSeparating possible merged cells...\n');

                    [analysisLabelVolume, ~] = ...
                        separateMicroglia3D( ...
                        labelVolume, ...
                        somaMask);

                    app.Microglia3DLabelVolume = ...
                        analysisLabelVolume;

                    app.Microglia3DSomaMask = somaMask;
                    app.Microglia3DFeatures = table();
                    app.Microglia3DClassified = table();

                    app.Progress = 0.92;
                    UpdateProgress(app);
                    drawnow;

                    numberOfSlices = size(analysisLabelVolume, 3);
                    middleSlice = ceil(numberOfSlices / 2);

                    app.SegmentedSliceSlider.Limits = ...
                        [1 max(2, numberOfSlices)];
                    app.SegmentedSliceSlider.MajorTicks = ...
                        [1 numberOfSlices];
                    app.SegmentedSliceSlider.Value = middleSlice;

                    % Resize the right axes for the Microglia3D result
                    app.HistogramAxes.Position = [460 50 365 365];

                    app.HistogramAxes.Box = 'off';
                    app.HistogramAxes.Color = 'none';
                    app.HistogramAxes.XColor = 'none';
                    app.HistogramAxes.YColor = 'none';

                    app.SegmentedSliceLabel.Visible = 'on';
                    app.SegmentedSliceSlider.Visible = 'on';
                    app.SegmentedMIPButton.Visible = 'on';

                    app.Show2DButton.Visible = 'on';
                    app.Show2DButton.Enable = 'on';

                    app.Show3DButton.Visible = 'on';
                    app.Show3DButton.Enable = 'on';

                    app.Classify3DButton.Visible = 'on';
                    app.Classify3DButton.Enable = 'on';

                    displaySegmentedMicrogliaSlice( ...
                        app, middleSlice);

                    objectLabels = unique(analysisLabelVolume);
                    objectLabels(objectLabels == 0) = [];
                    numberOfObjects = numel(objectLabels);

                    app.Number_of_Cells_Field.Visible = 'on';
                    app.Number_of_Cells_Text.Visible = 'on';
                    app.Number_of_Cells_Field.Value = numberOfObjects;

                    % Hide graph controls while the axes display the 3D-stack result
                    app.XDropDown.Visible = 'off';
                    app.YDropDown.Visible = 'off';
                    app.XFeatureLabel_2.Visible = 'off';
                    app.YFeatureLabel_2.Visible = 'off';
                    app.PlotIndex.Visible = 'off';
                    app.PlotFeaturesText.Visible = 'off';
                    app.PlotMode.Visible = 'off';
                    app.NextButton.Visible = 'off';
                    app.PreviousButton.Visible = 'off';

                    app.Progress = 1;
                    UpdateProgress(app);
                    drawnow;

                    app.ProgressBarAxes.Visible = 'off';
                    cla(app.ProgressBarAxes);

                    app.Segment_Button.Enable = 'on';

                    fprintf('\nMicroglia3D segmentation complete.\n');
                    fprintf('Final separated cells: %d\n', numberOfObjects);

                catch ME
                    app.Segment_Button.Enable = 'on';
                    app.ProgressBarAxes.Visible = 'off';
                    cla(app.ProgressBarAxes);

                    uialert(app.UIFigure, ...
                        ME.message, ...
                        '3D Segmentation Error');
                end

                return;
            end

            if isempty(app.Images)
                errordlg('Please upload an image first!', 'Error');
                return;
            end

            app.CancelRequested = false;

            app.Number_of_Cells_Field.Visible = 'on';
            app.Number_of_Cells_Text.Visible  = 'on';
            app.PauseButton.Visible           = 'on';

            % Set up properties and variables
            TrackingData  = [];
            tracks        = [];
            trackingTable = [];

            % Determine selected algorithm and read its parameters before the loop
            isSW     = strcmp(app.DropDown.Value, 'Sobel+Watershed');
            sw_sigma = app.SW_SigmaField.Value;
            sw_disk  = str2double(app.SW_DiskSizeField.Value);
            sw_pol   = lower(app.SW_PolarityDropDown.Value);  % 'bright' or 'dark'

            % ML parameters (only used when NOT Sobel+Watershed)
            minsize = [app.SizeMin.Value app.SizeMin.Value];
            maxsize = [app.SizeMax.Value app.SizeMax.Value];

            if ~isSW
                app.net.OverlapThresholdPrediction   = app.OverlaPP.Value / 100;
                app.net.OverlapThresholdRPN          = app.OverlapProp.Value / 100;
                app.net.NumStrongestRegions          = app.MaxProposals.Value;
                app.net.NumStrongestRegionsPrediction = app.MaxProposals.Value;
            end

            if strcmp(app.DenoiseSwitch.Value, 'On')
                denoiseOF = 1;
            else
                denoiseOF = 0;
            end

            % Perform cell segmentation
            try
                if ~iscell(app.Images)
                    app.Images{1} = app.Images;
                end

                for i = 1:size(app.Images, 2) %for each image loaded in

                    app.Progress = ((i-1) + 0.05) / size(app.Images, 2);
                    UpdateProgress(app);

                    % -------------------------------------------------------- %
                    % Branch on selected segmentation algorithm               %
                    % -------------------------------------------------------- %
                    if isSW
                        % ------------------------------------------------------ %
                        % Sobel + Watershed path                                  %
                        % ------------------------------------------------------ %
                        imgGray = im2uint8(app.Images{i});

                        app.SingleImage = (size(app.Images, 2) == 1);

                        [sw_masks, sw_labels, sw_scores, sw_boxes] = ...
                            segmentSobelWatershed(imgGray, sw_sigma, sw_disk, sw_pol);

                        if app.SingleImage
                            app.SingleMasks  = sw_masks;
                            app.SingleLabels = sw_labels;
                            app.SingleScores = sw_scores;
                            app.SingleBoxes  = sw_boxes;

                            % Confidence slider is not meaningful for SW
                            % (all scores == 1.0), so just store all detections
                            app.Masks{1}  = app.SingleMasks;
                            app.Labels{1} = app.SingleLabels;
                            app.Scores{1} = app.SingleScores;
                            app.Boxes{1}  = app.SingleBoxes;
                        else
                            % Multi-image: tracking not supported for SW;
                            % store results directly
                            app.Masks{i}  = sw_masks;
                            app.Labels{i} = sw_labels;
                            app.Scores{i} = sw_scores;
                            app.Boxes{i}  = sw_boxes;
                        end

                    else
                        % ------------------------------------------------------ %
                        % ML network path                                         %
                        % ------------------------------------------------------ %
                        if size(app.Images, 2) == 1
                            app.SingleImage = 1;
                            [app.SingleMasks, app.SingleLabels, app.SingleScores, app.SingleBoxes] = ...
                                segmentCells(app.net, rescale(app.Images{i}), ...
                                    'NumstrongestRegions', Inf, ...
                                    MinSize=minsize, MaxSize=maxsize, ...
                                    SelectStrongest=true, SegmentThreshold=1e-7, ...
                                    Denoise=denoiseOF, ...
                                    DWTThreshold=app.DWTThresh.Value/100, ...
                                    Level=ceil(app.DWTLEv.Value));

                            app.Scores{1} = app.SingleScores(app.SingleScores > app.Confidence.Value/100);
                            app.Labels{1} = app.SingleLabels(app.SingleScores > app.Confidence.Value/100);
                            app.Masks{1}  = app.SingleMasks(:,:, app.SingleScores > app.Confidence.Value/100);
                            app.Boxes{1}  = app.SingleBoxes(app.SingleScores > app.Confidence.Value/100, :);

                        elseif strcmp(app.TrackingOption.Value, 'On')
                            app.SingleImage = 0;
                            [app.Masks{i}, app.Labels{i}, app.Scores{i}, app.Boxes{i}, TrackingData] = ...
                                segmentFrame(app.net, rescale(app.Images{i}), TrackingData, ...
                                    'NumStrongestRegions', Inf, ...
                                    Threshold=app.Confidence.Value/100, ...
                                    Alpha=app.Alpha.Value/100, ...
                                    MinSize=minsize, MaxSize=maxsize);

                            app.Progress = ((i-1) + 0.7) / size(app.Images, 2);
                            UpdateProgress(app);

                            [tracks, trackingTable] = createTracks(tracks, app.Boxes{i}, ...
                                app.Scores{i}, i, trackingTable, ...
                                'MinIoU', app.MinIOUTrack.Value, ...
                                'MaxInvisibleCount', 5, 'MaxDistance', 250);
                        else
                            app.SingleImage = 0;
                            [app.Masks{i}, app.Labels{i}, app.Scores{i}, app.Boxes{i}] = ...
                                segmentCells(app.net, rescale(app.Images{i}), ...
                                    'NumstrongestRegions', Inf, ...
                                    MinSize=minsize, MaxSize=maxsize, ...
                                    SelectStrongest=true, ...
                                    SegmentThreshold=app.Confidence.Value/100, ...
                                    Denoise=denoiseOF, ...
                                    DWTThreshold=app.DWTThresh.Value/100, ...
                                    Level=ceil(app.DWTLEv.Value));
                        end
                    end
                    % -------------------------------------------------------- %
                    % End of segmentation branch                                 %
                    % -------------------------------------------------------- %

                    app.Progress = ((i-1) + 0.85) / size(app.Images, 2);
                    UpdateProgress(app);

                    app.CurrentImageIndex = i;
                    UpdateIm(app);

                    if app.CancelRequested
                        app.CancelRequested = false;
                        app.PauseButton.Visible = 'off';
                        app.Segment_Button.Visible = 'on';
                        return;
                    end

                    if strcmp(app.DropDown.Value, 'Sobel+Watershed')
                        W_fg    = app.Masks{i};
                        fgLbls  = unique(W_fg(W_fg > 0));
                        featureRows = cell(numel(fgLbls), 1);
                        for kk = 1:numel(fgLbls)
                            singleMask     = (W_fg == fgLbls(kk));
                            singleBox      = app.Boxes{i}(kk, :);
                            singleScore    = app.Scores{i}(kk);
                            featureRows{kk} = fixTableNames(app, ExtractFeatures(app.Images{i}, singleMask, ...
                            singleBox, singleScore, 1));
                        end
                        app.AnalysisData{i} = vertcat(featureRows{:});
                    else
                        app.AnalysisData{i} = fixTableNames(app, ExtractFeatures(app.Images{i}, app.Masks{i}, ...
                        app.Boxes{i}, app.Scores{i}, 1));
                    end
                    
                    % Update the cell count field
                    app.Number_of_Cells_Field.Value = size(app.AnalysisData{i}, 1);
                end

                % Build aggregate table
                tableRows = cell(size(app.AnalysisData, 2), 1);
                for i = 1:size(app.AnalysisData, 2)
                    Table_i        = app.AnalysisData{i};
                    ImageID        = table(repmat(i, [size(Table_i,1), 1]), 'VariableNames', {'ImageID'});
                    FileName       = table(repmat(string(app.Filenames{i}), [size(Table_i,1), 1]), 'VariableNames', {'FileName'});
                    tableRows{i}   = [ImageID FileName Table_i];
                end
                BigTable = vertcat(tableRows{:});

                app.AggregateAnalysis = BigTable;
                app.TrackingInfo      = trackingTable;
                app.Tracks            = tracks;

                app.Progress = i / size(app.Images, 2);
                UpdateProgress(app);

                app.PauseButton.Visible      = 'off';
                app.ExportData_Button.Visible = 'on';
                app.PlotIndex.Visible        = 'on';
                app.PlotFeaturesText.Visible = 'on';
                app.PlotMode.Visible         = 'on';
                app.NextButton.Visible       = 'on';
                app.PreviousButton.Visible   = 'on';
                app.XDropDown.Visible        = 'on';
                app.YDropDown.Visible        = 'on';
                app.HistogramAxes.Visible    = 'on';
                app.XFeatureLabel_2.Visible  = 'on';
                app.YFeatureLabel_2.Visible  = 'on';
                app.Segment_Button.Visible   = 'on';

                PlotHistogram(app);
                app.PlotIndex.Value = double(app.CurrentImageIndex);

            catch ME
                uialert(app.UIFigure, ME.message, 'Segmentation Error');
                app.PauseButton.Visible = 'off';
                app.Segment_Button.Visible = 'on';
            end
        end

        function PlotHistogram(app, ~)
            xFeature = app.XDropDown.Value;
            yFeature = app.YDropDown.Value;
            if isempty(app.AnalysisData), return; end

            % Restore graph axes after Microglia3D image display
            cla(app.HistogramAxes, 'reset');

            app.HistogramAxes.Position = [457 72 361 267];
            app.HistogramAxes.Box = 'on';
            app.HistogramAxes.Color = [1 1 1];
            app.HistogramAxes.XColor = [0.15 0.15 0.15];
            app.HistogramAxes.YColor = [0.15 0.15 0.15];

            app.HistogramAxes.XLimMode = 'auto';
            app.HistogramAxes.YLimMode = 'auto';
            app.HistogramAxes.DataAspectRatioMode = 'auto';
            app.HistogramAxes.PlotBoxAspectRatioMode = 'auto';
            app.HistogramAxes.YDir = 'normal';

            % Extract the selected feature's values
            if strcmp(app.PlotMode.Value, 'Aggregate')
                featureData = app.AggregateAnalysis;
            else
                featureData = app.AnalysisData{app.CurrentImageIndex};
            end

            % Decide plot type
            if strcmp(yFeature, 'Frequency')
                % Plot histogram 
                histogram(app.HistogramAxes, featureData.(xFeature));
                xlabel(app.HistogramAxes, xFeature);
                ylabel(app.HistogramAxes, 'Frequency');
            else
                % Create scatter plot
                scatter(app.HistogramAxes, featureData.(xFeature), featureData.(yFeature), 'filled');
                xlabel(app.HistogramAxes, xFeature);
                ylabel(app.HistogramAxes, yFeature);
                app.Number_of_Cells_Field.Value = length(app.Scores{app.CurrentImageIndex});
            end
        end

        % Button pushed function: PreviousButton
        function ToggleSwitchValueChanged(app, ~)
            if isempty(app.AnalysisData), return; end
            if app.CurrentImageIndex == 1
                app.CurrentImageIndex = size(app.AnalysisData, 2);
            else
                app.CurrentImageIndex = app.CurrentImageIndex - 1;
            end
            app.PlotIndex.Value = double(app.CurrentImageIndex);
            PlotHistogram(app);
            UpdateIm(app);
        end

        % Button pushed function: Settings_Button
        function Settings_ButtonPushed(app, ~)
            if strcmp(app.Panel.Visible, 'on')
                app.Panel.Visible = 'off';
            else
                app.Panel.Visible = 'on';
            end
        end

        % Button pushed function: CloseButton
        function CloseButtonPushed(app, ~)
            app.Panel.Visible = 'off';
        end

        % Value changed function: DenoiseSwitch
        function DenoiseSwitchValueChanged(app, ~)
            if strcmp(app.DenoiseSwitch.Value, 'Off')
                app.DWTLEv.Visible         = 'off';
                app.DWTThresh_t.Visible    = 'off';
                app.LevelEditFieldLabel.Visible  = 'off';
                app.LevelEditFieldLabel_2.Visible = 'off';
                app.DWTThresh.Visible      = 'off';
                app.Label_19.Visible       = 'off';
            else
                app.DWTLEv.Visible         = 'on';
                app.DWTThresh_t.Visible    = 'on';
                app.LevelEditFieldLabel.Visible  = 'on';
                app.LevelEditFieldLabel_2.Visible = 'on';
                app.DWTThresh.Visible      = 'on';
                app.Label_19.Visible       = 'on';
            end
        end

        % Value changed function: TrackingOption
        function TrackingOptionValueChanged2(app, ~)
            if strcmp(app.TrackingOption.Value, 'Off')
                app.MinIOUTrack.Visible        = 'off';
                app.IOUTrackText.Visible       = 'off';
                app.RPNOptAlphaText.Visible    = 'off';
                app.Alpha.Visible              = 'off';
                app.LevelEditFieldLabel_6.Visible = 'off';
                app.LevelEditFieldLabel_7.Visible = 'off';
                app.Label_11.Visible           = 'off';
                app.Label_13.Visible           = 'off';
            else
                app.MinIOUTrack.Visible        = 'on';
                app.IOUTrackText.Visible       = 'on';
                app.RPNOptAlphaText.Visible    = 'on';
                app.Alpha.Visible              = 'on';
                app.LevelEditFieldLabel_6.Visible = 'on';
                app.LevelEditFieldLabel_7.Visible = 'on';
                app.Label_11.Visible           = 'on';
                app.Label_13.Visible           = 'on';
            end
        end

        % Button pushed function: PauseButton
        function PauseButtonPushed(app, ~)
            app.CancelRequested = true;
            app.PauseButton.Visible = 'off';
        end

        % Button pushed function: DefaultButton
        function DefaultButtonPushed(app, ~)
            if strcmp(app.DropDown.Value, 'Sobel+Watershed'), return; end
            highlightButton(app, app.DefaultButton);
            app.OverlaPP.Value      = 30;
            app.OverlapProp.Value   = 30;
            app.Confidence.Value    = 50;
            app.Alpha.Value         = 15;
            app.DWTLEv.Value        = 4;
            app.ObjectSize.Value    = [15 200];
            app.MinIOUTrack.Value   = 5;
            app.DenoiseSwitch.Value = 'On';
            app.MaxProposals.Value  = 2500;
            app.DWTThresh.Value     = 2;

            app.OverlaPP_t.Value      = app.OverlaPP.Value;
            app.OverlapProp_t.Value   = app.OverlapProp.Value;
            app.ConfidenceField.Value = app.Confidence.Value;
            app.RPNOptAlphaText.Value = app.Alpha.Value;
            app.SizeMin.Value         = app.ObjectSize.Value(1);
            app.SizeMax.Value         = app.ObjectSize.Value(2);
            app.IOUTrackText.Value    = app.MinIOUTrack.Value;
            app.DWTThresh_t.Value     = app.DWTThresh.Value;
        end

        % Button pushed function: ConservativeButton
        function ConservativeButtonPushed(app, ~)
            if strcmp(app.DropDown.Value, 'Sobel+Watershed'), return; end
            highlightButton(app, app.ConservativeButton);
            app.OverlaPP.Value      = 25;
            app.OverlapProp.Value   = 25;
            app.Confidence.Value    = 75;
            app.Alpha.Value         = 5;
            app.DWTLEv.Value        = 4;
            app.ObjectSize.Value    = [25 150];
            app.MinIOUTrack.Value   = 10;
            app.DenoiseSwitch.Value = 'On';
            app.MaxProposals.Value  = 2500;
            app.DWTThresh.Value     = 2;

            app.OverlaPP_t.Value      = app.OverlaPP.Value;
            app.OverlapProp_t.Value   = app.OverlapProp.Value;
            app.ConfidenceField.Value = app.Confidence.Value;
            app.RPNOptAlphaText.Value = app.Alpha.Value;
            app.SizeMin.Value         = app.ObjectSize.Value(1);
            app.SizeMax.Value         = app.ObjectSize.Value(2);
            app.IOUTrackText.Value    = app.MinIOUTrack.Value;
            app.DWTThresh_t.Value     = app.DWTThresh.Value;
        end

        % Button pushed function: RelaxedButton
        function RelaxedButtonPushed(app, ~)
            if strcmp(app.DropDown.Value, 'Sobel+Watershed'), return; end
            highlightButton(app, app.RelaxedButton);
            app.OverlaPP.Value      = 75;
            app.OverlapProp.Value   = 75;
            app.Confidence.Value    = 25;
            app.Alpha.Value         = 20;
            app.DWTLEv.Value        = 4;
            app.ObjectSize.Value    = [2 250];
            app.MinIOUTrack.Value   = 2;
            app.DenoiseSwitch.Value = 'On';
            app.MaxProposals.Value  = 5000;
            app.DWTThresh.Value     = 2;

            app.OverlaPP_t.Value      = app.OverlaPP.Value;
            app.OverlapProp_t.Value   = app.OverlapProp.Value;
            app.ConfidenceField.Value = app.Confidence.Value;
            app.RPNOptAlphaText.Value = app.Alpha.Value;
            app.SizeMin.Value         = app.ObjectSize.Value(1);
            app.SizeMax.Value         = app.ObjectSize.Value(2);
            app.IOUTrackText.Value    = app.MinIOUTrack.Value;
            app.DWTThresh_t.Value     = app.DWTThresh.Value;
        end

        % Value changed function: PlotMode
        function PlotModeValueChanged(app, ~)
            PlotHistogram(app);
        end

        % Value changed function: PlotIndex
        function PlotIndexValueChanged(app, ~)
            value = ceil(app.PlotIndex.Value);
            if isempty(app.AnalysisData)
                value = 1;
            elseif value > size(app.AnalysisData, 2)
                value = size(app.AnalysisData, 2);
            end
            app.PlotIndex.Value   = value;
            app.CurrentImageIndex = value;
            UpdateIm(app);
            PlotHistogram(app);
        end

        % Value changed function: OverlaPP
        function OverlaPPValueChanged(app, ~)
            value = max(0.01, min(100, app.OverlaPP.Value));
            app.OverlaPP.Value   = value;
            app.OverlaPP_t.Value = value;
        end

        % Value changed function: OverlapProp_t
        function OverlapProp_tValueChanged(app, ~)
            value = max(0.01, min(100, app.OverlapProp_t.Value));
            app.OverlaPP.Value   = value;
            app.OverlaPP_t.Value = value;
        end

        % Value changed function: OverlapProp
        function OverlapPropValueChanged(app, ~)
            value = max(0.01, min(100, app.OverlapProp.Value));
            app.OverlapProp.Value   = value;
            app.OverlapProp_t.Value = value;
        end

        % Value changed function: OverlaPP_t
        function OverlaPP_tValueChanged(app, ~)
            value = max(0.01, min(100, app.OverlaPP_t.Value));
            app.OverlaPP.Value   = value;
            app.OverlaPP_t.Value = value;
        end

        % Value changed function: Confidence
        function ConfidenceValueChanged(app, ~)
            if strcmp(app.DropDown.Value, 'Sobel+Watershed'), return; end
            value = max(0.01, min(100, app.Confidence.Value));
            app.Confidence.Value    = value;
            app.ConfidenceField.Value = value;

            if app.SingleImage == 1
                app.Scores{1} = app.SingleScores(app.SingleScores > app.Confidence.Value/100);
                app.Labels{1} = app.SingleLabels(app.SingleScores > app.Confidence.Value/100);
                app.Masks{1}  = app.SingleMasks(:,:, app.SingleScores > app.Confidence.Value/100);
                app.Boxes{1}  = app.SingleBoxes(app.SingleScores > app.Confidence.Value/100, :);
                app.AnalysisData{1} = fixTableNames(app, ExtractFeatures(app.Images{1}, app.Masks{1}, app.Boxes{1}, app.Scores{1}, 1));

                DispColors = (1.4 - 0.5*rescale(app.Scores{1}) + 0.05*rand([length(app.Scores{1}) 1])) .* [0.466 0.674 0.188];
                app.SegmentedImages = insertObjectMask(app.Images{1}, app.Masks{1}, Color=DispColors);
                app.Number_of_Cells_Field.Value = length(app.Scores{1});
                IMGDisplay(app, app.SegmentedImages, app.ImageDisp); drawnow;
                PlotHistogram(app);
            end
        end

        % Value changed function: ConfidenceField
        function ConfidenceFieldValueChanged(app, ~)
            if strcmp(app.DropDown.Value, 'Sobel+Watershed'), return; end
            value = max(1e-7, min(100, app.ConfidenceField.Value));
            app.Confidence.Value      = value;
            app.ConfidenceField.Value = value;

            if app.SingleImage == 1
                app.Scores{1} = app.SingleScores(app.SingleScores > app.Confidence.Value/100);
                app.Labels{1} = app.SingleLabels(app.SingleScores > app.Confidence.Value/100);
                app.Masks{1}  = app.SingleMasks(:,:, app.SingleScores > app.Confidence.Value/100);
                app.Boxes{1}  = app.SingleBoxes(app.SingleScores > app.Confidence.Value/100, :);
                app.AnalysisData{1} = fixTableNames(app, ExtractFeatures(app.Images{1}, app.Masks{1}, app.Boxes{1}, app.Scores{1}, 1));

                DispColors = (1.4 - 0.5*rescale(app.Scores{1}) + 0.05*rand([length(app.Scores{1}) 1])) .* [0.466 0.674 0.188];
                app.SegmentedImages = insertObjectMask(app.Images{1}, app.Masks{1}, Color=DispColors);
                app.Number_of_Cells_Field.Value = length(app.Scores{1});
                IMGDisplay(app, app.SegmentedImages, app.ImageDisp); drawnow;
            end
        end

        % Value changing function: Confidence
        function ConfidenceValueChanging(app, event)
            if strcmp(app.DropDown.Value, 'Sobel+Watershed'), return; end
            changingValue = max(1e-5, min(100, event.Value));
            app.Confidence.Value      = changingValue;
            app.ConfidenceField.Value = changingValue;

            if app.SingleImage == 1
                app.Masks{1}  = app.SingleMasks(:,:, app.SingleScores > app.Confidence.Value/100);
                app.Scores{1} = app.SingleScores(app.SingleScores > app.Confidence.Value/100);

                DispColors = (1.4 - 0.5*rescale(app.Scores{1}) + 0.05*rand([length(app.Scores{1}) 1])) .* [0.466 0.674 0.188];
                app.SegmentedImages = insertObjectMask(app.Images{1}, app.Masks{1}, Color=DispColors);
                app.Number_of_Cells_Field.Value = length(app.Scores{1});
                IMGDisplay(app, app.SegmentedImages, app.ImageDisp); drawnow;
            end
        end

        % Value changed function: DropDown
        % Loads the selected ML network .mat file, or skips loading for Sobel+Watershed
        function DropDownValueChanged(app, ~)
            value = app.DropDown.Value;

            if strcmp(value, 'Sobel+Watershed') || strcmp(value, 'Microglia3D')
                % These algorithms do not load a neural-network MAT file.
                updateAlgorithmControls(app);
            else
                loaded   = load(value + ".mat", 'net');
                app.net  = loaded.net;
                updateAlgorithmControls(app);
            end
        end

        % Value changed function: DWTLEv
        function DWTLEvValueChanged(app, ~)
            app.DWTLEv.Value = ceil(app.DWTLEv.Value);
        end

        % Value changed function: DWTThresh_t
        function DWTThresh_tValueChanged(app, ~)
            value = max(0, min(10, app.DWTThresh_t.Value));
            app.DWTThresh.Value   = value;
            app.DWTThresh_t.Value = value;
        end

        % Value changed function: IOUTrackText
        function IOUTrackTextValueChanged(app, ~)
            value = max(0.001, min(20, app.IOUTrackText.Value));
            app.IOUTrackText.Value  = value;
            app.MinIOUTrack.Value   = value;
        end

        % Value changed function: MinIOUTrack
        function MinIOUTrackValueChanged(app, ~)
            value = max(0.001, min(20, app.MinIOUTrack.Value));
            app.IOUTrackText.Value = value;
            app.MinIOUTrack.Value  = value;
        end

        % Value changed function: Alpha
        function AlphaValueChanged(app, ~)
            value = max(0.001, min(30, app.Alpha.Value));
            app.Alpha.Value          = value;
            app.RPNOptAlphaText.Value = value;
        end

        % Value changed function: RPNOptAlphaText
        function RPNOptAlphaTextValueChanged(app, ~)
            value = max(0.001, min(30, app.RPNOptAlphaText.Value));
            app.Alpha.Value          = value;
            app.RPNOptAlphaText.Value = value;
        end

        % Value changed function: ObjectSize
        function ObjectSizeValueChanged(app, ~)
            value = ceil(app.ObjectSize.Value);
            if value(1) <= 1,       value(1) = 1;           end
            if value(2) <= value(1), value(2) = value(1)+1; end
            app.ObjectSize.Value = value;
            app.SizeMin.Value    = value(1);
            app.SizeMax.Value    = value(2);
        end

        % Value changed function: DWTThresh
        function DWTThreshValueChanged(app, ~)
            value = max(0, min(10, app.DWTThresh.Value));
            app.DWTThresh.Value   = value;
            app.DWTThresh_t.Value = value;
        end

        % Value changed function: MaxProposals
        function MaxProposalsValueChanged(app, ~)
            value = max(500, ceil(app.MaxProposals.Value));
            app.MaxProposals.Value = value;
        end

        % Button pushed function: NextButton
        function NextButtonPushed2(app, ~)
            if isempty(app.AnalysisData), return; end
            if app.CurrentImageIndex == size(app.AnalysisData, 2)
                app.CurrentImageIndex = 1;
            else
                app.CurrentImageIndex = app.CurrentImageIndex + 1;
            end
            app.PlotIndex.Value = double(app.CurrentImageIndex);
            UpdateIm(app);
            PlotHistogram(app);
        end

        % Button pushed function: Reset
        function ResetButtonPushed(app, ~)
            % Clear loaded images and related data
            app.Images            = {};
            app.Filenames         = {};
            app.SegmentedImages   = {};
            app.Masks             = {};
            app.Boxes             = {};
            app.Labels            = {};
            app.Scores            = {};
            app.AnalysisData      = {};
            app.CurrentImageIndex = 1;
            app.SegmentedImages   = [];
            app.SingleMasks       = [];
            app.SingleBoxes       = [];
            app.SingleScores      = [];
            app.SingleLabels      = [];
            app.AnalysisData      = [];
            app.Progress          = 0;
            UpdateProgress(app);
            app.TrackingInfo      = [];
            app.SingleImage       = 0;
            app.AggregateAnalysis = [];
            app.Tracks            = [];

            % Clear Microglia3D state
            app.Microglia3DFilename = "";
            app.Microglia3DImage = [];
            app.Microglia3DLabelVolume = [];
            app.Microglia3DSomaMask = [];
            app.Microglia3DMetadataX = NaN;
            app.Microglia3DMetadataY = NaN;
            app.Microglia3DMetadataZ = NaN;
            app.Microglia3DFeatures = table();
            app.Microglia3DClassified = table();

            app.StackSliceLabel.Visible = 'off';
            app.StackSliceSlider.Visible = 'off';
            app.MIPButton.Visible = 'off';
            app.SegmentedSliceLabel.Visible = 'off';
            app.SegmentedSliceSlider.Visible = 'off';
            app.SegmentedMIPButton.Visible = 'off';
            app.Show2DButton.Visible = 'off';
            app.Show3DButton.Visible = 'off';
            app.Classify3DButton.Visible = 'off';

            app.HistogramAxes.Position = [457 72 361 267];

            cla(app.HistogramAxes, 'reset');

            app.HistogramAxes.Position = [457 72 361 267];

            app.HistogramAxes.Box = 'on';
            app.HistogramAxes.Color = [1 1 1];
            app.HistogramAxes.XColor = [0.15 0.15 0.15];
            app.HistogramAxes.YColor = [0.15 0.15 0.15];

            app.HistogramAxes.XLimMode = 'auto';
            app.HistogramAxes.YLimMode = 'auto';
            app.HistogramAxes.DataAspectRatioMode = 'auto';
            app.HistogramAxes.PlotBoxAspectRatioMode = 'auto';
            app.HistogramAxes.YDir = 'normal';

            app.ProgressBarAxes.Position = [0 395 435 37];

            % Reset Image display to default grey screen
            IMGDisplay(app, ones(520, 740)*0.9, app.ImageDisp);

            % Reset mode to default if needed
            app.PlotMode.Value = 'Individual';

            % Hide/Reset UI elements
            app.InitialLabel.Visible        = 'on';
            app.Segment_Button.Visible      = 'off';
            app.NextButton.Visible          = 'off';
            app.PreviousButton.Visible      = 'off';
            app.ExportData_Button.Visible   = 'off';
            app.HistogramAxes.Visible       = 'off';
            app.Number_of_Cells_Field.Visible = 'off';
            app.Number_of_Cells_Text.Visible  = 'off';
            app.PlotFeaturesText.Visible    = 'off';
            app.PlotMode.Visible            = 'off';
            app.XDropDown.Visible           = 'off';
            app.YDropDown.Visible           = 'off';
            app.XFeatureLabel_2.Visible     = 'off';
            app.YFeatureLabel_2.Visible     = 'off';
            app.PlotIndex.Visible           = 'off';
            app.ExportPanel.Visible         = 'off';

            cla(app.HistogramAxes);
            app.Number_of_Cells_Field.Value = 0;

            UpdateProgress(app); % Reset progress value
            DefaultButtonPushed(app); % Re-apply default model button
            drawnow;
        end

        % Button pushed function: ExportData_Button
        function ExportData_ButtonPushed(app, ~)
            if strcmp(app.ExportPanel.Visible, 'on')
                app.ExportPanel.Visible = 'off';
            else
                app.ExportPanel.Visible = 'on';
            end
        end

        % Button pushed function: CloseButton_2
        function CloseButton_2Pushed(app, ~)
            app.ExportPanel.Visible = 'off';
        end

        % Button pushed function: ExportButton
        function ExportButtonPushed(app, ~)
            % Ask user to choose folder
            exportDir = uigetdir(pwd, 'Select Export Folder');
            if exportDir == 0, return; end

            % Export CSVs if checked
            if app.CSVFileCheckBox.Value
                try
                    if ~isempty(app.TrackingInfo)
                        writetable(app.TrackingInfo, fullfile(exportDir, 'TrackingInfo.csv'));
                    end
                    writetable(app.AggregateAnalysis, fullfile(exportDir, 'FeatureTable.csv'));
                catch ME
                    uialert(app.UIFigure, ['Failed to export CSV: ' ME.message], 'Export Error');
                    return;
                end
            end

            if app.matFileCheckBox.Value || app.JSONFileCheckBox.Value

                if app.matFileCheckBox.Value
                    try
                        % Copy to local struct so save() can write named variables
                        exportData.Images       = app.Images;
                        exportData.Masks        = app.Masks;
                        exportData.Boxes        = app.Boxes;
                        exportData.Labels       = app.Labels;
                        exportData.Scores       = app.Scores;
                        exportData.Filenames    = app.Filenames;
                        exportData.Tracks       = app.Tracks;
                        exportData.AnalysisData = app.AnalysisData;
                        exportData.TrackingInfo = app.TrackingInfo;
                        exportData.FeatureTable = app.AggregateAnalysis;
                        save(fullfile(exportDir, 'ExportedData.mat'), '-struct', 'exportData');
                    catch ME
                        uialert(app.UIFigure, ['Failed to export MAT: ' ME.message], 'Export Error');
                        return;
                    end
                end
            
                if app.JSONFileCheckBox.Value
                    mkdir(fullfile(exportDir, 'JSONExports'));
                    % Ensure all table column names are valid MATLAB identifiers before JSON encoding
                    fixedAnalysisData = cellfun(@(T) fixTableNames(app, T), app.AnalysisData, ...
                       'UniformOutput', false);
                    
                    exports = {'Images',       app.Images; ...
                               'Masks',        app.Masks; ...
                               'Boxes',        app.Boxes; ...
                               'Labels',       app.Labels; ...
                               'Scores',       app.Scores; ...
                               'Filenames',    app.Filenames; ...
                               'AnalysisTable',fixedAnalysisData; ...
                               'Tracks',       app.Tracks};

                    for e = 1:size(exports, 1)
                        fid = fopen(fullfile(exportDir, 'JSONExports', [exports{e,1} '.JSON']), 'w');
                        fprintf(fid, jsonencode(exports{e,2}));
                        fclose(fid);
                    end
                end
            end
        end

        % Value changed function: SizeMin
        function SizeMinValueChanged(app, ~)
            value = max(1, ceil(app.SizeMin.Value));
            app.ObjectSize.Value(1) = value;
            app.SizeMin.Value       = value;
        end

        % Value changed function: SizeMax
        function SizeMaxValueChanged(app, ~)
            value = min(250, ceil(app.SizeMax.Value));
            app.ObjectSize.Value(2) = value;
            app.SizeMax.Value       = value;
        end

        % Button pushed function: Help_Button
        function Help_ButtonPushed(~, ~)
            web('https://github.com/TechAvi-eng/Multires-ML-Microscopy/blob/main/App%20User%20Guide.pdf');
        end

    end % private methods (callbacks)

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Locate icon files via MATLAB path search, falling back to the
            % directory of this file if which() cannot find them
            pathToMLAPP = fileparts(mfilename('fullpath'));
            if isempty(dir(fullfile(pathToMLAPP, 'settings.png')))
                settingsDir = fileparts(which('settings.png'));
                if ~isempty(settingsDir)
                    pathToMLAPP = settingsDir;
                end
            end

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            colormap(app.UIFigure, 'turbo');
            app.UIFigure.Position = [100 100 838 439];
            app.UIFigure.Name     = 'MATLAB App';
            app.UIFigure.Resize   = 'off';

            % HistogramAxes
            app.HistogramAxes = uiaxes(app.UIFigure);
            ylabel(app.HistogramAxes, 'Frequency')
            app.HistogramAxes.Toolbar.Visible = 'off';
            app.HistogramAxes.Box             = 'on';
            colormap(app.HistogramAxes, 'turbo')
            app.HistogramAxes.Visible         = 'off';
            app.HistogramAxes.Position        = [457 72 361 267];

            % ProgressBarAxes
            app.ProgressBarAxes = uiaxes(app.UIFigure);
            app.ProgressBarAxes.Toolbar.Visible = 'off';
            app.ProgressBarAxes.FontWeight      = 'bold';
            app.ProgressBarAxes.XColor          = [0.9412 0.9412 0.9412];
            app.ProgressBarAxes.YColor          = [0.9412 0.9412 0.9412];
            app.ProgressBarAxes.ZColor          = [0.9412 0.9412 0.9412];
            app.ProgressBarAxes.Position        = [0 395 435 37];

            % UploadImagesButton
            app.UploadImagesButton = uibutton(app.UIFigure, 'push');
            app.UploadImagesButton.ButtonPushedFcn = createCallbackFcn(app, @UploadImage, true);
            app.UploadImagesButton.BackgroundColor = [0.6784 0.8 0.8392];
            app.UploadImagesButton.FontName        = 'Inter';
            app.UploadImagesButton.FontSize        = 18;
            app.UploadImagesButton.FontWeight      = 'bold';
            app.UploadImagesButton.Position        = [96 59 139 31];
            app.UploadImagesButton.Text            = 'Select Images';

            % Settings_Button
            app.Settings_Button = uibutton(app.UIFigure, 'push');
            app.Settings_Button.ButtonPushedFcn = createCallbackFcn(app, @Settings_ButtonPushed, true);
            app.Settings_Button.BackgroundColor = [1 1 1];
            app.Settings_Button.FontName        = 'Inter';
            app.Settings_Button.FontSize        = 14;
            app.Settings_Button.Tooltip         = {'Settings'};
            app.Settings_Button.Position        = [96 17 139 29];
            app.Settings_Button.Text            = '';
            app.Settings_Button.Icon            = fullfile(pathToMLAPP, 'settings.png');

            % Help_Button
            app.Help_Button = uibutton(app.UIFigure, 'push');
            app.Help_Button.ButtonPushedFcn = createCallbackFcn(app, @Help_ButtonPushed, true);
            app.Help_Button.BackgroundColor = [1 1 1];
            app.Help_Button.FontName        = 'Inter';
            app.Help_Button.FontSize        = 18;
            app.Help_Button.Tooltip         = {'Help (Opens User Guide)'};
            app.Help_Button.Position        = [10 12 33 33];
            app.Help_Button.Text            = '?';

            % Segment_Button
            app.Segment_Button = uibutton(app.UIFigure, 'push');
            app.Segment_Button.ButtonPushedFcn = createCallbackFcn(app, @Segment, true);
            app.Segment_Button.BusyAction      = 'cancel';
            app.Segment_Button.BackgroundColor = [0.6784 0.8 0.8392];
            app.Segment_Button.FontName        = 'Inter';
            app.Segment_Button.FontSize        = 18;
            app.Segment_Button.FontWeight      = 'bold';
            app.Segment_Button.Visible         = 'off';
            app.Segment_Button.Position        = [245 59 132 31];
            app.Segment_Button.Text            = 'Segment';

            % Number_of_Cells_Text
            app.Number_of_Cells_Text = uilabel(app.UIFigure);
            app.Number_of_Cells_Text.HorizontalAlignment = 'right';
            app.Number_of_Cells_Text.FontSize             = 18;
            app.Number_of_Cells_Text.FontWeight           = 'bold';
            app.Number_of_Cells_Text.Visible              = 'off';
            app.Number_of_Cells_Text.Position             = [520 397 150 25];
            app.Number_of_Cells_Text.Text                 = 'Number of Cells ';

            % Number_of_Cells_Field
            app.Number_of_Cells_Field = uieditfield(app.UIFigure, 'numeric');
            app.Number_of_Cells_Field.Limits              = [0 Inf];
            app.Number_of_Cells_Field.RoundFractionalValues = 'on';
            app.Number_of_Cells_Field.AllowEmpty          = 'on';
            app.Number_of_Cells_Field.Editable            = 'off';
            app.Number_of_Cells_Field.HorizontalAlignment = 'left';
            app.Number_of_Cells_Field.FontName            = 'Inter';
            app.Number_of_Cells_Field.FontSize            = 18;
            app.Number_of_Cells_Field.Visible             = 'off';
            app.Number_of_Cells_Field.Position            = [670 397 50 25];

            % YDropDown
            app.YDropDown = uidropdown(app.UIFigure);
            app.YDropDown.Items             = {'Frequency','Area','Perimeter','AspectRatio','MeanIntensity','StandardDeviation','Eccentricity','Circularity','Solidity','MaxDiameter','Kurtosis'};
            app.YDropDown.ValueChangedFcn   = createCallbackFcn(app, @PlotHistogram, true);
            app.YDropDown.Visible           = 'off';
            app.YDropDown.FontSize          = 18;
            app.YDropDown.Position          = [684 338 129 24];
            app.YDropDown.Value             = 'Perimeter';

            % XDropDown
            app.XDropDown = uidropdown(app.UIFigure);
            app.XDropDown.Items             = {'Area','Perimeter','AspectRatio','MeanIntensity','StandardDeviation','Eccentricity','Circularity','Solidity','MaxDiameter','Kurtosis'};
            app.XDropDown.ValueChangedFcn   = createCallbackFcn(app, @PlotHistogram, true);
            app.XDropDown.Visible           = 'off';
            app.XDropDown.FontSize          = 18;
            app.XDropDown.Position          = [517 338 131 24];
            app.XDropDown.Value             = 'Area';

            % Credit
            app.Credit = uihyperlink(app.UIFigure);
            app.Credit.FontName = 'Inter';
            app.Credit.URL      = 'https://github.com/TechAvi-eng/Multires-ML-Microscopy';
            app.Credit.Position = [692 -5 166 30];
            app.Credit.Text     = 'Multires ML Microscopy';

            % ExportData_Button
            app.ExportData_Button = uibutton(app.UIFigure, 'push');
            app.ExportData_Button.ButtonPushedFcn = createCallbackFcn(app, @ExportData_ButtonPushed, true);
            app.ExportData_Button.BackgroundColor = [1 1 1];
            app.ExportData_Button.FontName        = 'Inter';
            app.ExportData_Button.FontSize        = 18;
            app.ExportData_Button.Visible         = 'off';
            app.ExportData_Button.Tooltip         = {'Export Data'};
            app.ExportData_Button.Position        = [245 17 132 29];
            app.ExportData_Button.Text            = '';
            app.ExportData_Button.Icon            = fullfile(pathToMLAPP, 'export.png');

            % ImageDisp
            app.ImageDisp = uiimage(app.UIFigure);
            app.ImageDisp.ImageClickedFcn = createCallbackFcn(app, @UploadImage, true);
            app.ImageDisp.Position        = [41 104 390 292];

            % Microglia3D stack navigation controls

            % Original stack controls
            app.StackSliceLabel = uilabel(app.UIFigure);
            app.StackSliceLabel.Visible = 'off';
            app.StackSliceLabel.FontWeight = 'bold';
            app.StackSliceLabel.HorizontalAlignment = 'center';
            app.StackSliceLabel.Position = [390 380 45 20];
            app.StackSliceLabel.Text = 'Z';

            app.StackSliceSlider = uislider(app.UIFigure);
            app.StackSliceSlider.Orientation = 'vertical';
            app.StackSliceSlider.Visible = 'off';
            app.StackSliceSlider.Limits = [1 2];
            app.StackSliceSlider.Value = 1;
            app.StackSliceSlider.ValueChangedFcn = ...
                createCallbackFcn(app, @StackSliceSliderValueChanged, true);
            app.StackSliceSlider.ValueChangingFcn = ...
                createCallbackFcn(app, @StackSliceSliderValueChanging, true);
            app.StackSliceSlider.Position = [405 145 3 225];

            app.MIPButton = uibutton(app.UIFigure, 'push');
            app.MIPButton.Visible = 'off';
            app.MIPButton.ButtonPushedFcn = ...
                createCallbackFcn(app, @MIPButtonPushed, true);
            app.MIPButton.Position = [390 105 45 25];
            app.MIPButton.Text = 'MIP';
            app.MIPButton.Tooltip = {'Maximum Intensity Projection'};

            % Segmented stack controls
            app.SegmentedSliceLabel = uilabel(app.UIFigure);
            app.SegmentedSliceLabel.Visible = 'off';
            app.SegmentedSliceLabel.FontWeight = 'bold';
            app.SegmentedSliceLabel.HorizontalAlignment = 'center';
            app.SegmentedSliceLabel.Position = [771 380 45 20];
            app.SegmentedSliceLabel.Text = 'Z';

            app.SegmentedSliceSlider = uislider(app.UIFigure);
            app.SegmentedSliceSlider.Orientation = 'vertical';
            app.SegmentedSliceSlider.Visible = 'off';
            app.SegmentedSliceSlider.Limits = [1 2];
            app.SegmentedSliceSlider.Value = 1;
            app.SegmentedSliceSlider.ValueChangedFcn = ...
                createCallbackFcn(app, @SegmentedSliceSliderValueChanged, true);
            app.SegmentedSliceSlider.ValueChangingFcn = ...
                createCallbackFcn(app, @SegmentedSliceSliderValueChanging, true);
            app.SegmentedSliceSlider.Position = [786 145 3 225];

            app.SegmentedMIPButton = uibutton(app.UIFigure, 'push');
            app.SegmentedMIPButton.Visible = 'off';
            app.SegmentedMIPButton.ButtonPushedFcn = ...
                createCallbackFcn(app, @SegmentedMIPButtonPushed, true);
            app.SegmentedMIPButton.Position = [771 105 45 25];
            app.SegmentedMIPButton.Text = 'MIP';
            app.SegmentedMIPButton.Tooltip = {'Maximum Intensity Projection'};

            % Segmentation result buttons under the right result area
            app.Show2DButton = uibutton(app.UIFigure, 'push');
            app.Show2DButton.Visible = 'off';
            app.Show2DButton.ButtonPushedFcn = ...
                createCallbackFcn(app, @Show2DButtonPushed, true);
            app.Show2DButton.BackgroundColor = [1 1 1];
            app.Show2DButton.FontName        = 'Inter';
            app.Show2DButton.FontSize        = 18;
            app.Show2DButton.FontWeight      = 'bold';
            app.Show2DButton.Position = [515 60 90 31];
            app.Show2DButton.Text = '2D';
            app.Show2DButton.Tooltip = ...
                {'Display the separated masks as a coloured 2D projection'};

            app.Show3DButton = uibutton(app.UIFigure, 'push');
            app.Show3DButton.Visible = 'off';
            app.Show3DButton.ButtonPushedFcn = ...
                createCallbackFcn(app, @Show3DButtonPushed, true);
            app.Show3DButton.BackgroundColor = [1 1 1];
            app.Show3DButton.FontName        = 'Inter';
            app.Show3DButton.FontSize        = 18;
            app.Show3DButton.FontWeight      = 'bold';
            app.Show3DButton.Position = [635 60 90 31];
            app.Show3DButton.Text = '3D';
            app.Show3DButton.Tooltip = ...
                {'Display separated cells as coloured 3D surfaces'};

            app.Classify3DButton = uibutton(app.UIFigure, 'push');
            app.Classify3DButton.Visible = 'off';
            app.Classify3DButton.Enable = 'off';
            app.Classify3DButton.ButtonPushedFcn = ...
                createCallbackFcn(app, @Classify3DButtonPushed, true);
            app.Classify3DButton.BackgroundColor = [0.6784 0.8 0.8392];
            app.Classify3DButton.FontName        = 'Inter';
            app.Classify3DButton.FontSize        = 18;
            app.Classify3DButton.FontWeight      = 'bold';
            app.Classify3DButton.Position = [555 20 130 31];
            app.Classify3DButton.Text = 'Classify Cells';

            % Reset
            app.Reset = uibutton(app.UIFigure, 'push');
            app.Reset.ButtonPushedFcn = createCallbackFcn(app, @ResetButtonPushed, true);
            app.Reset.Position        = [10 53 33 34];
            app.Reset.Text            = '';
            app.Reset.Icon            = fullfile(pathToMLAPP, 'reset2.jpg');

            % PlotMode
            app.PlotMode = uiswitch(app.UIFigure, 'slider');
            app.PlotMode.Items            = {'Aggregate','Individual'};
            app.PlotMode.ValueChangedFcn  = createCallbackFcn(app, @PlotModeValueChanged, true);
            app.PlotMode.Visible          = 'off';
            app.PlotMode.Tooltip = {'Plot values for a single image or across the entire dataset'};
            app.PlotMode.FontName         = 'Inter';
            app.PlotMode.Position         = [517 41 36 16];
            app.PlotMode.Value            = 'Individual';

            % PreviousButton
            app.PreviousButton = uibutton(app.UIFigure, 'push');
            app.PreviousButton.ButtonPushedFcn = createCallbackFcn(app, @ToggleSwitchValueChanged, true);
            app.PreviousButton.IconAlignment = 'center';
            app.PreviousButton.BackgroundColor = [0.949 0.949 0.949];
            app.PreviousButton.FontName        = 'Inter';
            app.PreviousButton.FontWeight      = 'bold';
            app.PreviousButton.Visible         = 'off';
            app.PreviousButton.Tooltip         = {'Previous Image'};
            app.PreviousButton.Position        = [628 38 66 22];
            app.PreviousButton.Text            = '';
            app.PreviousButton.Icon            = fullfile(pathToMLAPP, 'previousbutton.png');

            % NextButton
            app.NextButton = uibutton(app.UIFigure, 'push');
            app.NextButton.ButtonPushedFcn = createCallbackFcn(app, @NextButtonPushed2, true);
            app.NextButton.IconAlignment = 'center';
            app.NextButton.BackgroundColor = [0.949 0.949 0.949];
            app.NextButton.FontName        = 'Inter';
            app.NextButton.FontWeight      = 'bold';
            app.NextButton.Visible         = 'off';
            app.NextButton.Tooltip         = {'Next Image'};
            app.NextButton.Position        = [732 38 67 23];
            app.NextButton.Text            = '';
            app.NextButton.Icon            = fullfile(pathToMLAPP, 'nextbutton.png');

            % PlotFeaturesText
            app.PlotFeaturesText = uilabel(app.UIFigure);
            app.PlotFeaturesText.FontSize   = 18;
            app.PlotFeaturesText.FontWeight = 'bold';
            app.PlotFeaturesText.Visible    = 'off';
            app.PlotFeaturesText.Position   = [502 367 124 24];
            app.PlotFeaturesText.Text       = 'Plot Features:';

            % PauseButton
            app.PauseButton = uibutton(app.UIFigure, 'push');
            app.PauseButton.ButtonPushedFcn = createCallbackFcn(app, @PauseButtonPushed, true);
            app.PauseButton.IconAlignment = 'top';
            app.PauseButton.BackgroundColor = [1 1 1];
            app.PauseButton.FontName        = 'Inter';
            app.PauseButton.FontSize        = 18;
            app.PauseButton.FontWeight      = 'bold';
            app.PauseButton.Visible         = 'off';
            app.PauseButton.Tooltip         = {'Cancel Current Segmentation'};
            app.PauseButton.Position        = [245 59 132 31];
            app.PauseButton.Text            = '';
            app.PauseButton.Icon            = fullfile(pathToMLAPP, 'stop2.png');

            % PlotIndex
            app.PlotIndex = uieditfield(app.UIFigure, 'numeric');
            app.PlotIndex.Limits             = [1 Inf];
            app.PlotIndex.ValueDisplayFormat = '%11.0g';
            app.PlotIndex.ValueChangedFcn    = createCallbackFcn(app, @PlotIndexValueChanged, true);
            app.PlotIndex.HorizontalAlignment = 'center';
            app.PlotIndex.Visible            = 'off';
            app.PlotIndex.Position           = [702 38 23 22];
            app.PlotIndex.Value              = 1;

            % XFeatureLabel_2
            app.XFeatureLabel_2 = uilabel(app.UIFigure);
            app.XFeatureLabel_2.HorizontalAlignment = 'right';
            app.XFeatureLabel_2.FontSize            = 18;
            app.XFeatureLabel_2.Visible             = 'off';
            app.XFeatureLabel_2.Position            = [489 338 25 24];
            app.XFeatureLabel_2.Text                = 'X ';

            % YFeatureLabel_2
            app.YFeatureLabel_2 = uilabel(app.UIFigure);
            app.YFeatureLabel_2.HorizontalAlignment = 'right';
            app.YFeatureLabel_2.FontSize            = 18;
            app.YFeatureLabel_2.Visible             = 'off';
            app.YFeatureLabel_2.Position            = [663 338 14 24];
            app.YFeatureLabel_2.Text                = 'Y';

            % InitialLabel
            app.InitialLabel = uilabel(app.UIFigure);
            app.InitialLabel.FontWeight = 'bold';
            app.InitialLabel.Position   = [154 239 174 22];
            app.InitialLabel.Text       = 'Please select images to start.';

            % ExportPanel
            app.ExportPanel = uipanel(app.UIFigure);
            app.ExportPanel.Visible         = 'off';
            app.ExportPanel.BackgroundColor = [0.7412 0.8118 0.8314];
            app.ExportPanel.Position        = [335 123 170 195];

            % Create ExportDataLabel
            app.ExportDataLabel = uilabel(app.ExportPanel);
            app.ExportDataLabel.FontSize   = 18;
            app.ExportDataLabel.FontWeight = 'bold';
            app.ExportDataLabel.Position   = [32 165 107 24];
            app.ExportDataLabel.Text       = 'Export Data';

            % Create CloseButton_2
            app.CloseButton_2 = uibutton(app.ExportPanel, 'push');
            app.CloseButton_2.ButtonPushedFcn = createCallbackFcn(app, @CloseButton_2Pushed, true);
            app.CloseButton_2.IconAlignment = 'center';
            app.CloseButton_2.FontSize        = 15;
            app.CloseButton_2.FontWeight      = 'bold';
            app.CloseButton_2.Position        = [113 8 33 29];
            app.CloseButton_2.Text            = '';
            app.CloseButton_2.Icon            = fullfile(pathToMLAPP, 'close.png');

            % Create CSVFileCheckBox
            app.CSVFileCheckBox = uicheckbox(app.ExportPanel);
            app.CSVFileCheckBox.Text       = '   CSV File';
            app.CSVFileCheckBox.FontSize   = 14;
            app.CSVFileCheckBox.FontWeight = 'bold';
            app.CSVFileCheckBox.Position   = [39 115 95 25];
            app.CSVFileCheckBox.Value      = true;

            % Create matFileCheckBox
            app.matFileCheckBox = uicheckbox(app.ExportPanel);
            app.matFileCheckBox.Text       = '   .mat File';
            app.matFileCheckBox.FontSize   = 14;
            app.matFileCheckBox.FontWeight = 'bold';
            app.matFileCheckBox.Position   = [39 85 93 25];

            % Create ExportButton
            app.ExportButton = uibutton(app.ExportPanel, 'push');
            app.ExportButton.ButtonPushedFcn = createCallbackFcn(app, @ExportButtonPushed, true);
            app.ExportButton.FontSize        = 18;
            app.ExportButton.FontWeight      = 'bold';
            app.ExportButton.Position        = [19 8 85 31];
            app.ExportButton.Text            = 'Export';

            % Create JSONFileCheckBox
            app.JSONFileCheckBox = uicheckbox(app.ExportPanel);
            app.JSONFileCheckBox.Text       = '  JSON File';
            app.JSONFileCheckBox.FontSize   = 14;
            app.JSONFileCheckBox.FontWeight = 'bold';
            app.JSONFileCheckBox.Position   = [39 55 170 25];

            % ---------------------------------------------------------------- %
            % Settings Panel                                                   %
            % ---------------------------------------------------------------- %
            % Create Panel
            app.Panel = uipanel(app.UIFigure);
            app.Panel.Visible         = 'off';
            app.Panel.BackgroundColor = [0.749 0.8118 0.8314];
            app.Panel.Position        = [243 1 596 440];

            % Create SettingsLabel
            app.SettingsLabel = uilabel(app.Panel);
            app.SettingsLabel.FontSize   = 18;
            app.SettingsLabel.FontWeight = 'bold';
            app.SettingsLabel.Position   = [263 407 76 24];
            app.SettingsLabel.Text       = 'Settings';

            % Create PresetLabel
            app.PresetLabel = uilabel(app.Panel);
            app.PresetLabel.FontSize   = 15;
            app.PresetLabel.FontWeight = 'bold';
            app.PresetLabel.Tooltip    = {'Presets mainly affect the false positive and negative rates'};
            app.PresetLabel.Position   = [5 372 51 22];
            app.PresetLabel.Text       = 'Preset';

            % Create DefaultButton
            app.DefaultButton = uibutton(app.Panel, 'push');
            app.DefaultButton.ButtonPushedFcn = createCallbackFcn(app, @DefaultButtonPushed, true);
            app.DefaultButton.BackgroundColor = [0.7608 0.8 0.8];
            app.DefaultButton.Position        = [112 372 100 23];
            app.DefaultButton.Text            = 'Default';

            % Create ConservativeButton
            app.ConservativeButton = uibutton(app.Panel, 'push');
            app.ConservativeButton.ButtonPushedFcn = createCallbackFcn(app, @ConservativeButtonPushed, true);
            app.ConservativeButton.BackgroundColor = [0.7608 0.8 0.8];
            app.ConservativeButton.Position        = [249 372 100 23];
            app.ConservativeButton.Text            = 'Conservative';

            % Create RelaxedButton
            app.RelaxedButton = uibutton(app.Panel, 'push');
            app.RelaxedButton.ButtonPushedFcn = createCallbackFcn(app, @RelaxedButtonPushed, true);
            app.RelaxedButton.BackgroundColor = [0.7608 0.8 0.8];
            app.RelaxedButton.Position        = [387 372 100 23];
            app.RelaxedButton.Text            = 'Relaxed';

            % Create ConfidenceThresholdLabel
            app.ConfidenceThresholdLabel = uilabel(app.Panel);
            app.ConfidenceThresholdLabel.FontSize   = 15;
            app.ConfidenceThresholdLabel.FontWeight = 'bold';
            app.ConfidenceThresholdLabel.Tooltip    = {'The minimum confidence to display a result'};
            app.ConfidenceThresholdLabel.Position   = [5 327 105 34];
            app.ConfidenceThresholdLabel.Text       = {'Confidence'; 'Threshold'};

            % Create Confidence
            app.Confidence = uislider(app.Panel);
            app.Confidence.MajorTicks       = [0 100];
            app.Confidence.ValueChangedFcn  = createCallbackFcn(app, @ConfidenceValueChanged, true);
            app.Confidence.ValueChangingFcn = createCallbackFcn(app, @ConfidenceValueChanging, true);
            app.Confidence.Position         = [219 351 156 3];

            % Create ConfidenceField
            app.ConfidenceField = uieditfield(app.Panel, 'numeric');
            app.ConfidenceField.Limits             = [0 100];
            app.ConfidenceField.ValueDisplayFormat = '%5.0f';
            app.ConfidenceField.ValueChangedFcn    = createCallbackFcn(app, @ConfidenceFieldValueChanged, true);
            app.ConfidenceField.HorizontalAlignment = 'center';
            app.ConfidenceField.FontWeight          = 'bold';
            app.ConfidenceField.Placeholder         = '20%';
            app.ConfidenceField.Position            = [144 336 35 25];
            app.ConfidenceField.Value               = 50;

            % Create AdvancedLabel
            app.AdvancedLabel = uilabel(app.Panel);
            app.AdvancedLabel.FontSize   = 18;
            app.AdvancedLabel.FontWeight = 'bold';
            app.AdvancedLabel.Position   = [258 288 95 24];
            app.AdvancedLabel.Text       = 'Advanced ';

            % Create PreProcessingLabel
            app.PreProcessingLabel = uilabel(app.Panel);
            app.PreProcessingLabel.FontSize   = 15;
            app.PreProcessingLabel.FontWeight = 'bold';
            app.PreProcessingLabel.Position   = [5 252 115 22];
            app.PreProcessingLabel.Text       = 'Pre-Processing';

            % Create LevelEditFieldLabel
            app.LevelEditFieldLabel = uilabel(app.Panel);
            app.LevelEditFieldLabel.HorizontalAlignment = 'right';
            app.LevelEditFieldLabel.FontWeight          = 'bold';
            app.LevelEditFieldLabel.Position            = [265 252 35 22];
            app.LevelEditFieldLabel.Text                = 'Level';

            % Create DWTLEv
            app.DWTLEv = uieditfield(app.Panel, 'numeric');
            app.DWTLEv.Limits             = [1 8];
            app.DWTLEv.ValueChangedFcn    = createCallbackFcn(app, @DWTLEvValueChanged, true);
            app.DWTLEv.HorizontalAlignment = 'center';
            app.DWTLEv.FontWeight          = 'bold';
            app.DWTLEv.Tooltip             = {'Level of decomposition for de-noising: higher levels remove more noise at the cost of performance'};
            app.DWTLEv.Position            = [308 252 20 22];
            app.DWTLEv.Value               = 4;

            % Create DWTThresh
            app.DWTThresh = uislider(app.Panel);
            app.DWTThresh.Limits            = [0 10];
            app.DWTThresh.MajorTicks        = [0 10];
            app.DWTThresh.MajorTickLabels   = {'0','10'};
            app.DWTThresh.ValueChangedFcn   = createCallbackFcn(app, @DWTThreshValueChanged, true);
            app.DWTThresh.Tooltip = {''};
            app.DWTThresh.Position          = [453 262 116 3];

            % Create LevelEditFieldLabel_2
            app.LevelEditFieldLabel_2 = uilabel(app.Panel);
            app.LevelEditFieldLabel_2.HorizontalAlignment = 'right';
            app.LevelEditFieldLabel_2.FontWeight          = 'bold';
            app.LevelEditFieldLabel_2.Tooltip             = {'Tunes de-noising: larger values remove more noise at the expense of image quality'};
            app.LevelEditFieldLabel_2.Position            = [344 252 62 22];
            app.LevelEditFieldLabel_2.Text                = 'Threshold';

            % Create SegmentationNetworkLabel
            app.SegmentationNetworkLabel = uilabel(app.Panel);
            app.SegmentationNetworkLabel.FontSize   = 15;
            app.SegmentationNetworkLabel.FontWeight = 'bold';
            app.SegmentationNetworkLabel.Position   = [5 195 170 22];
            app.SegmentationNetworkLabel.Text       = 'Segmentation Network';

            % Create DropDown
            app.DropDown = uidropdown(app.Panel);
            app.DropDown.Items            = {'EfficientNet','ResNet50','ResNet101','CascadeEfficientNet','Sobel+Watershed','Microglia3D'};
            app.DropDown.ValueChangedFcn  = createCallbackFcn(app, @DropDownValueChanged, true);
            app.DropDown.FontSize         = 14;
            app.DropDown.FontWeight       = 'bold';
            app.DropDown.BackgroundColor  = [0.7608 0.8 0.8];
            app.DropDown.Position         = [240 189 134 28];
            app.DropDown.Value            = 'EfficientNet';

            % Create OverlaPP
            app.OverlaPP = uislider(app.Panel);
            app.OverlaPP.MajorTicks       = [0 100];
            app.OverlaPP.ValueChangedFcn  = createCallbackFcn(app, @OverlaPPValueChanged, true);
            app.OverlaPP.Tooltip = {''};
            app.OverlaPP.Position         = [172 176 115 3];

            % Create OverlapProp
            app.OverlapProp = uislider(app.Panel);
            app.OverlapProp.MajorTicks      = [0 100];
            app.OverlapProp.ValueChangedFcn = createCallbackFcn(app, @OverlapPropValueChanged, true);
            app.OverlapProp.Tooltip = {''};
            app.OverlapProp.Position        = [172 127 116 3];

            % Create LevelEditFieldLabel_3
            app.LevelEditFieldLabel_3 = uilabel(app.Panel);
            app.LevelEditFieldLabel_3.HorizontalAlignment = 'right';
            app.LevelEditFieldLabel_3.FontWeight          = 'bold';
            app.LevelEditFieldLabel_3.Tooltip             = {'Maximum overlap allowed between cells'};
            app.LevelEditFieldLabel_3.Position            = [61 162 49 22];
            app.LevelEditFieldLabel_3.Text                = 'Overlap';

            % Create LevelEditFieldLabel_4
            app.LevelEditFieldLabel_4 = uilabel(app.Panel);
            app.LevelEditFieldLabel_4.HorizontalAlignment = 'center';
            app.LevelEditFieldLabel_4.FontWeight          = 'bold';
            app.LevelEditFieldLabel_4.Tooltip             = {'Maximum overlap allowed between proposals: reduces processing time, but may miss adjacent cells'};
            app.LevelEditFieldLabel_4.Position            = [58 113 56 30];
            app.LevelEditFieldLabel_4.Text                = {'Proposal'; 'Overlap'};

            % Create OverlaPP_t
            app.OverlaPP_t = uieditfield(app.Panel, 'numeric');
            app.OverlaPP_t.ValueDisplayFormat = '%11.0f';
            app.OverlaPP_t.ValueChangedFcn    = createCallbackFcn(app, @OverlaPP_tValueChanged, true);
            app.OverlaPP_t.HorizontalAlignment = 'center';
            app.OverlaPP_t.FontWeight          = 'bold';
            app.OverlaPP_t.Tooltip             = {''};
            app.OverlaPP_t.Position            = [120 161 34 25];
            app.OverlaPP_t.Value               = 30;

            % Create OverlapProp_t
            app.OverlapProp_t = uieditfield(app.Panel, 'numeric');
            app.OverlapProp_t.Limits             = [0 100];
            app.OverlapProp_t.ValueDisplayFormat = '%11.0f';
            app.OverlapProp_t.ValueChangedFcn    = createCallbackFcn(app, @OverlapProp_tValueChanged, true);
            app.OverlapProp_t.HorizontalAlignment = 'center';
            app.OverlapProp_t.FontWeight          = 'bold';
            app.OverlapProp_t.Tooltip             = {''};
            app.OverlapProp_t.Position            = [120 116 34 25];
            app.OverlapProp_t.Value               = 30;

            % Create LevelEditFieldLabel_5
            app.LevelEditFieldLabel_5 = uilabel(app.Panel);
            app.LevelEditFieldLabel_5.HorizontalAlignment = 'center';
            app.LevelEditFieldLabel_5.FontWeight          = 'bold';
            app.LevelEditFieldLabel_5.Position            = [424 189 122 30];
            app.LevelEditFieldLabel_5.Text                = 'Maximum Proposals';

            % Create MaxProposals
            app.MaxProposals = uieditfield(app.Panel, 'numeric');
            app.MaxProposals.Limits             = [500 Inf];
            app.MaxProposals.ValueDisplayFormat = '%11.0f';
            app.MaxProposals.ValueChangedFcn    = createCallbackFcn(app, @MaxProposalsValueChanged, true);
            app.MaxProposals.HorizontalAlignment = 'center';
            app.MaxProposals.FontWeight          = 'bold';
            app.MaxProposals.Tooltip             = {''};
            app.MaxProposals.Position            = [547 191 39 25];
            app.MaxProposals.Value               = 2500;

            % Create DenoiseSwitch
            app.DenoiseSwitch = uiswitch(app.Panel, 'slider');
            app.DenoiseSwitch.ValueChangedFcn = createCallbackFcn(app, @DenoiseSwitchValueChanged, true);
            app.DenoiseSwitch.Tooltip         = {'Enable de-noising of images (recommended)'};
            app.DenoiseSwitch.Position        = [156 253 45 20];
            app.DenoiseSwitch.Value           = 'On';

            % Create TrackingLabel_2
            app.TrackingLabel_2 = uilabel(app.Panel);
            app.TrackingLabel_2.FontSize   = 15;
            app.TrackingLabel_2.FontWeight = 'bold';
            app.TrackingLabel_2.Position   = [5 62 66 22];
            app.TrackingLabel_2.Text       = 'Tracking';

            % Create LevelEditFieldLabel_6
            app.LevelEditFieldLabel_6 = uilabel(app.Panel);
            app.LevelEditFieldLabel_6.HorizontalAlignment = 'center';
            app.LevelEditFieldLabel_6.FontWeight          = 'bold';
            app.LevelEditFieldLabel_6.Visible             = 'off';
            app.LevelEditFieldLabel_6.Tooltip             = {'Minimum overlap needed for two predictions to be tracked as one object'};
            app.LevelEditFieldLabel_6.Position            = [49 10 74 44];
            app.LevelEditFieldLabel_6.Text                = 'Min Overlap';

            % Create LevelEditFieldLabel_7
            app.LevelEditFieldLabel_7 = uilabel(app.Panel);
            app.LevelEditFieldLabel_7.HorizontalAlignment = 'center';
            app.LevelEditFieldLabel_7.FontWeight          = 'bold';
            app.LevelEditFieldLabel_7.Visible             = 'off';
            app.LevelEditFieldLabel_7.Tooltip             = {'Speed up processing during tracking by pruning proposals at the expense of recall'};
            app.LevelEditFieldLabel_7.Position            = [296 11 145 44];
            app.LevelEditFieldLabel_7.Text                = {'RPN '; 'Optimization '; 'Factor'};

            % Create Alpha
            app.Alpha = uislider(app.Panel);
            app.Alpha.Limits           = [0 30];
            app.Alpha.MajorTicks       = [0 30];
            app.Alpha.MajorTickLabels  = {'0','30'};
            app.Alpha.ValueChangedFcn  = createCallbackFcn(app, @AlphaValueChanged, true);
            app.Alpha.Visible          = 'off';
            app.Alpha.Tooltip          = {'Minimum overlap needed for two predictions to be tracked as one object'};
            app.Alpha.Position         = [453 39 114 3];

            % Create RPNOptAlphaText
            app.RPNOptAlphaText = uieditfield(app.Panel, 'numeric');
            app.RPNOptAlphaText.Limits             = [0 30];
            app.RPNOptAlphaText.ValueDisplayFormat = '%11.0f';
            app.RPNOptAlphaText.ValueChangedFcn    = createCallbackFcn(app, @RPNOptAlphaTextValueChanged, true);
            app.RPNOptAlphaText.HorizontalAlignment = 'center';
            app.RPNOptAlphaText.FontWeight          = 'bold';
            app.RPNOptAlphaText.Visible             = 'off';
            app.RPNOptAlphaText.Tooltip             = {'Minimum overlap needed for two predictions to be tracked as one object'};
            app.RPNOptAlphaText.Position            = [413 21 23 25];
            app.RPNOptAlphaText.Value               = 15;

            % Create IOUTrackText
            app.IOUTrackText = uieditfield(app.Panel, 'numeric');
            app.IOUTrackText.Limits             = [0 20];
            app.IOUTrackText.ValueDisplayFormat = '%11.0f';
            app.IOUTrackText.ValueChangedFcn    = createCallbackFcn(app, @IOUTrackTextValueChanged, true);
            app.IOUTrackText.HorizontalAlignment = 'center';
            app.IOUTrackText.FontWeight          = 'bold';
            app.IOUTrackText.Visible             = 'off';
            app.IOUTrackText.Tooltip             = {'Minimum overlap needed for two predictions to be tracked as one object'};
            app.IOUTrackText.Position            = [131 21 23 25];
            app.IOUTrackText.Value               = 5;

            % Create MinIOUTrack
            app.MinIOUTrack = uislider(app.Panel);
            app.MinIOUTrack.Limits          = [0 20];
            app.MinIOUTrack.MajorTicks      = [0 20];
            app.MinIOUTrack.ValueChangedFcn = createCallbackFcn(app, @MinIOUTrackValueChanged, true);
            app.MinIOUTrack.Visible         = 'off';
            app.MinIOUTrack.Tooltip         = {'Minimum overlap needed for two predictions to be tracked as one object'};
            app.MinIOUTrack.Position        = [172 39 118 3];

            % Create DWTThresh_t
            app.DWTThresh_t = uieditfield(app.Panel, 'numeric');
            app.DWTThresh_t.Limits             = [0 10];
            app.DWTThresh_t.ValueDisplayFormat = '%11.0f';
            app.DWTThresh_t.ValueChangedFcn    = createCallbackFcn(app, @DWTThresh_tValueChanged, true);
            app.DWTThresh_t.HorizontalAlignment = 'center';
            app.DWTThresh_t.FontWeight          = 'bold';
            app.DWTThresh_t.Position            = [410 252 28 22];
            app.DWTThresh_t.Value               = 2;

            % Create CloseButton
            app.CloseButton = uibutton(app.Panel, 'push');
            app.CloseButton.ButtonPushedFcn = createCallbackFcn(app, @CloseButtonPushed, true);
            app.CloseButton.FontSize        = 15;
            app.CloseButton.FontWeight      = 'bold';
            app.CloseButton.Position        = [558 401 29 30];
            app.CloseButton.Text            = '';
            app.CloseButton.Icon            = fullfile(pathToMLAPP, 'close.png');

            % Create LevelEditFieldLabel_8
            app.LevelEditFieldLabel_8 = uilabel(app.Panel);
            app.LevelEditFieldLabel_8.HorizontalAlignment = 'center';
            app.LevelEditFieldLabel_8.FontWeight          = 'bold';
            app.LevelEditFieldLabel_8.Tooltip             = {'Number of potential cells evaluated: increases recall, but greatly increases processing time'};
            app.LevelEditFieldLabel_8.Position            = [369 113 67 30];
            app.LevelEditFieldLabel_8.Text                = 'Cell Size';

            % Create SizeMin
            app.SizeMin = uieditfield(app.Panel, 'numeric');
            app.SizeMin.Limits             = [0 Inf];
            app.SizeMin.ValueDisplayFormat = '%11.0f';
            app.SizeMin.ValueChangedFcn    = createCallbackFcn(app, @SizeMinValueChanged, true);
            app.SizeMin.HorizontalAlignment = 'center';
            app.SizeMin.FontWeight          = 'bold';
            app.SizeMin.Tooltip             = {'Min Size'};
            app.SizeMin.Position            = [437 142 35 25];
            app.SizeMin.Value               = 15;

            % Create Label_2
            app.Label_2 = uilabel(app.Panel);
            app.Label_2.FontSize   = 10;
            app.Label_2.FontWeight = 'bold';
            app.Label_2.Position   = [1 311 610 17];
            app.Label_2.Text       = '________________________________________________________________________________________________________________________';

            % Create TrackingOption
            app.TrackingOption = uiswitch(app.Panel, 'slider');
            app.TrackingOption.ValueChangedFcn = createCallbackFcn(app, @TrackingOptionValueChanged2, true);
            app.TrackingOption.Position        = [156 63 45 20];

            % Create Label_6
            app.Label_6 = uilabel(app.Panel);
            app.Label_6.FontWeight = 'bold';
            app.Label_6.Position   = [1 219 618 22];
            app.Label_6.Text       = '____________________________________________________________________________________________________';

            % Create Label_7
            app.Label_7 = uilabel(app.Panel);
            app.Label_7.FontWeight = 'bold';
            app.Label_7.Position   = [1 84 618 22];
            app.Label_7.Text       = '____________________________________________________________________________________________________';

            % Create Label_8
            app.Label_8 = uilabel(app.Panel);
            app.Label_8.FontWeight = 'bold';
            app.Label_8.Position   = [291 328 25 22];
            app.Label_8.Text       = '%';

            % Create Label_9
            app.Label_9 = uilabel(app.Panel);
            app.Label_9.FontWeight = 'bold';
            app.Label_9.Position   = [223 155 25 22];
            app.Label_9.Text       = '%';

            % Create Label_10
            app.Label_10 = uilabel(app.Panel);
            app.Label_10.FontWeight = 'bold';
            app.Label_10.Position   = [223 108 25 22];
            app.Label_10.Text       = '%';

            % Create Label_11
            app.Label_11 = uilabel(app.Panel);
            app.Label_11.FontWeight = 'bold';
            app.Label_11.Visible    = 'off';
            app.Label_11.Position   = [223 18 25 22];
            app.Label_11.Text       = '%';

            % Create Label_13
            app.Label_13 = uilabel(app.Panel);
            app.Label_13.FontWeight = 'bold';
            app.Label_13.Visible    = 'off';
            app.Label_13.Position   = [503 18 25 22];
            app.Label_13.Text       = '%';

            % Create ObjectSize
            app.ObjectSize = uislider(app.Panel, 'range');
            app.ObjectSize.Limits          = [0 250];
            app.ObjectSize.MajorTicks      = [0 250];
            app.ObjectSize.ValueChangedFcn = createCallbackFcn(app, @ObjectSizeValueChanged, true);
            app.ObjectSize.Position        = [453 127 116 3];
            app.ObjectSize.Value           = [0 250];

            % Create SizeMax
            app.SizeMax = uieditfield(app.Panel, 'numeric');
            app.SizeMax.Limits             = [0 Inf];
            app.SizeMax.ValueDisplayFormat = '%11.0f';
            app.SizeMax.ValueChangedFcn    = createCallbackFcn(app, @SizeMaxValueChanged, true);
            app.SizeMax.HorizontalAlignment = 'center';
            app.SizeMax.FontWeight          = 'bold';
            app.SizeMax.Tooltip             = {'Max Size'};
            app.SizeMax.Position            = [547 142 39 25];
            app.SizeMax.Value               = 250;

            app.Label_14 = uilabel(app.Panel); app.Label_14.FontSize=10; app.Label_14.FontWeight='bold'; app.Label_14.Position=[1 311 610 17]; app.Label_14.Text='________________________________________________________________________________________________________________________';
            app.Label_15 = uilabel(app.Panel); app.Label_15.FontSize=10; app.Label_15.FontWeight='bold'; app.Label_15.Position=[1 311 610 17]; app.Label_15.Text='________________________________________________________________________________________________________________________';
            app.Label_16 = uilabel(app.Panel); app.Label_16.FontSize=10; app.Label_16.FontWeight='bold'; app.Label_16.Position=[1 311 610 17]; app.Label_16.Text='________________________________________________________________________________________________________________________';
            app.Label_17 = uilabel(app.Panel); app.Label_17.FontSize=10; app.Label_17.FontWeight='bold'; app.Label_17.Position=[2 311 610 17]; app.Label_17.Text='________________________________________________________________________________________________________________________';
            app.Label_18 = uilabel(app.Panel); app.Label_18.FontSize=10; app.Label_18.FontWeight='bold'; app.Label_18.Position=[1 311 610 17]; app.Label_18.Text='________________________________________________________________________________________________________________________';

            % Create Label_19
            app.Label_19 = uilabel(app.Panel);
            app.Label_19.FontWeight = 'bold';
            app.Label_19.Position   = [503 240 25 22];
            app.Label_19.Text       = '%';

            % ---------------------------------------------------------------- %
            % SW: Sobel+Watershed parameter sub-panel                          %
            %         Positioned below the "Advanced" divider (y ~ 155-220)    %
            %         and visible only when 'Sobel+Watershed' is selected.     %
            % ---------------------------------------------------------------- %
            app.SW_Panel = uipanel(app.Panel);
            app.SW_Panel.Visible         = 'off';
            app.SW_Panel.BackgroundColor = [0.69 0.78 0.80];
            app.SW_Panel.BorderType      = 'line';
            app.SW_Panel.Title           = '';
            app.SW_Panel.Position        = [5 60 580 125];   % sits in the Advanced area

            app.SW_TitleLabel = uilabel(app.SW_Panel);
            app.SW_TitleLabel.FontSize   = 13;
            app.SW_TitleLabel.FontWeight = 'bold';
            app.SW_TitleLabel.Position   = [10 98 300 22];
            app.SW_TitleLabel.Text       = 'Sobel + Watershed Parameters';

            % Sigma
            app.SW_SigmaLabel = uilabel(app.SW_Panel);
            app.SW_SigmaLabel.FontWeight = 'bold';
            app.SW_SigmaLabel.Tooltip    = {'Standard deviation for Gaussian blur ( > 0)'};
            app.SW_SigmaLabel.Position   = [10 63 130 22];
            app.SW_SigmaLabel.Text       = 'Gaussian Sigma:';

            app.SW_SigmaField = uieditfield(app.SW_Panel, 'numeric');
            app.SW_SigmaField.Limits              = [0.1 Inf];
            app.SW_SigmaField.Value               = 1.5;
            app.SW_SigmaField.HorizontalAlignment = 'center';
            app.SW_SigmaField.FontWeight          = 'bold';
            app.SW_SigmaField.Tooltip             = {'Gaussian blur sigma — larger values smooth more aggressively'};
            app.SW_SigmaField.Position            = [145 63 50 22];

            % Disk Size
            app.SW_DiskSizeLabel = uilabel(app.SW_Panel);
            app.SW_DiskSizeLabel.FontWeight = 'bold';
            app.SW_DiskSizeLabel.Tooltip    = {'Morphological disk radius — correlates with cell area in pixels (see report)'};
            app.SW_DiskSizeLabel.Position   = [10 33 130 22];
            app.SW_DiskSizeLabel.Text       = 'Disk Size:';

            app.SW_DiskSizeField = uidropdown(app.SW_Panel);
            app.SW_DiskSizeField.Items           = {'1', '2', '3', '4'};
            app.SW_DiskSizeField.Value           = '1';
            app.SW_DiskSizeField.FontWeight      = 'bold';
            app.SW_DiskSizeField.BackgroundColor = [0.7608 0.8 0.8];
            app.SW_DiskSizeField.Tooltip         = {'Morphological disk radius — approximates single-cell size in pixels'};
            app.SW_DiskSizeField.Position        = [145 33 50 22];

            % Polarity
            app.SW_PolarityLabel = uilabel(app.SW_Panel);
            app.SW_PolarityLabel.FontWeight = 'bold';
            app.SW_PolarityLabel.Tooltip    = {'Whether cells appear brighter or darker than the background'};
            app.SW_PolarityLabel.Position   = [10 5 130 22];
            app.SW_PolarityLabel.Text       = 'Cell Polarity:';

            app.SW_PolarityDropDown = uidropdown(app.SW_Panel);
            app.SW_PolarityDropDown.Items           = {'bright', 'dark'};
            app.SW_PolarityDropDown.Value           = 'dark';
            app.SW_PolarityDropDown.FontWeight      = 'bold';
            app.SW_PolarityDropDown.BackgroundColor = [0.7608 0.8 0.8];
            app.SW_PolarityDropDown.Tooltip         = {'bright = cells lighter than background; dark = cells darker'};
            app.SW_PolarityDropDown.Position        = [145 5 80 22];

            % Microglia3D settings panel
            % Only user-adjustable parameters are shown here
            app.MG3D_Panel = uipanel(app.Panel);
            app.MG3D_Panel.Visible = 'off';
            app.MG3D_Panel.BackgroundColor = [0.69 0.78 0.80];
            app.MG3D_Panel.BorderType = 'line';
            app.MG3D_Panel.Position = [5 5 580 180];

            app.MG3D_TitleLabel = uilabel(app.MG3D_Panel);
            app.MG3D_TitleLabel.FontSize = 13;
            app.MG3D_TitleLabel.FontWeight = 'bold';
            app.MG3D_TitleLabel.Position = [10 150 250 20];
            app.MG3D_TitleLabel.Text = 'Microglia3D Parameters';

            app.MG3D_XVoxelLabel = uilabel(app.MG3D_Panel);
            app.MG3D_XVoxelLabel.Position = [10 100 100 20];
            app.MG3D_XVoxelLabel.Text = 'X voxel size (um):';

            app.MG3D_XVoxelField = uieditfield(app.MG3D_Panel, 'numeric');
            app.MG3D_XVoxelField.Limits = [0 Inf];
            app.MG3D_XVoxelField.Value = 0;
            app.MG3D_XVoxelField.Position = [110 100 40 20];

            app.MG3D_YVoxelLabel = uilabel(app.MG3D_Panel);
            app.MG3D_YVoxelLabel.Position = [10 65 100 20];
            app.MG3D_YVoxelLabel.Text = 'Y voxel size (um):';

            app.MG3D_YVoxelField = uieditfield(app.MG3D_Panel, 'numeric');
            app.MG3D_YVoxelField.Limits = [0 Inf];
            app.MG3D_YVoxelField.Value = 0;
            app.MG3D_YVoxelField.Position = [110 65 40 20];

            app.MG3D_ZVoxelLabel = uilabel(app.MG3D_Panel);
            app.MG3D_ZVoxelLabel.Position = [10 30 110 20];
            app.MG3D_ZVoxelLabel.Text = 'Z voxel size (um):';

            app.MG3D_ZVoxelField = uieditfield(app.MG3D_Panel, 'numeric');
            app.MG3D_ZVoxelField.Limits = [0 Inf];
            app.MG3D_ZVoxelField.Value = 0;
            app.MG3D_ZVoxelField.Position = [110 30 40 20];

            app.MG3D_LowThresholdLabel = uilabel(app.MG3D_Panel);
            app.MG3D_LowThresholdLabel.Position = [160 82 140 20];
            app.MG3D_LowThresholdLabel.Text = 'Low threshold multiplier:';

            app.MG3D_LowThresholdField = uieditfield(app.MG3D_Panel, 'numeric');
            app.MG3D_LowThresholdField.Limits = [0 Inf];
            app.MG3D_LowThresholdField.Value = 0.7;
            app.MG3D_LowThresholdField.Position = [300 82 40 20];

            app.MG3D_HighThresholdLabel = uilabel(app.MG3D_Panel);
            app.MG3D_HighThresholdLabel.Position = [160 47 140 20];
            app.MG3D_HighThresholdLabel.Text = 'High threshold multiplier:';

            app.MG3D_HighThresholdField = uieditfield(app.MG3D_Panel, 'numeric');
            app.MG3D_HighThresholdField.Limits = [0 Inf];
            app.MG3D_HighThresholdField.Value = 1.15;
            app.MG3D_HighThresholdField.Position = [300 47 40 20];

            app.MG3D_MinVolumeLabel = uilabel(app.MG3D_Panel);
            app.MG3D_MinVolumeLabel.Position = [350 82 170 20];
            app.MG3D_MinVolumeLabel.Text = 'Minimum cell volume (um^3):';

            app.MG3D_MinVolumeField = uieditfield(app.MG3D_Panel, 'numeric');
            app.MG3D_MinVolumeField.Limits = [0 Inf];
            app.MG3D_MinVolumeField.Value = 200;
            app.MG3D_MinVolumeField.Position = [530 82 40 20];

            app.MG3D_MinSomaLabel = uilabel(app.MG3D_Panel);
            app.MG3D_MinSomaLabel.Position = [350 47 170 20];
            app.MG3D_MinSomaLabel.Text = 'Minimum soma volume (um^3):';

            app.MG3D_MinSomaField = uieditfield(app.MG3D_Panel, 'numeric');
            app.MG3D_MinSomaField.Limits = [0 Inf];
            app.MG3D_MinSomaField.Value = 100;
            app.MG3D_MinSomaField.Position = [530 47 40 20];

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = Multires_ML_Microscopy
            % Create UIFigure and components
            createComponents(app)
            % Register the app with App Designer
            registerApp(app, app.UIFigure)
            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)
            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end

