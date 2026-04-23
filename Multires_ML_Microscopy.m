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
% External function dependencies (must be on the MATLAB path):
%   - segmentCells.m            : ML-based cell segmentation for single/multi-image input
%   - segmentFrame.m            : Single-frame segmentation with temporal tracking state
%   - segmentSobelWatershed.m   : Classical Sobel edge + Watershed segmentation
%   - createTracks.m            : IoU-based cell tracking across image sequences
%   - ExtractFeatures.m         : Morphological and intensity feature extraction per cell
%
% Required network files (must be in the working directory or on the MATLAB path):
%   - EfficientNet.mat          : Default Mask R-CNN network, loaded on startup
%   - ResNet50.mat              : Optional alternative network
%   - ResNet101.mat             : Optional alternative network
%   - CascadeEfficientNet.mat   : Optional alternative network
%
% Tested on MATLAB R2025b (version 25.2)

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
            isSW = strcmp(app.DropDown.Value, 'Sobel+Watershed');

            % Sobel+Watershed parameter panel
            if isSW
                app.SW_Panel.Visible = 'on';
            else
                app.SW_Panel.Visible = 'off';
            end

            mlVisibility = 'on';
            if isSW
                mlVisibility = 'off';
            end

            % All controls that are meaningless or actively harmful for SW
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
        end

        function T = fixTableNames(~, T)
            % Replace spaces in column names to produce valid MATLAB identifiers,
            % matching exactly what jsonencode/table2struct would do internally
            % so that no automatic renaming warning is triggered.
            T.Properties.VariableNames = matlab.lang.makeValidName( ...
                T.Properties.VariableNames);
        end

    end % private methods


    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            IMGDisplay(app, ones(520, 740)*0.9, app.ImageDisp);
            loaded  = load('EfficientNet.mat', 'net');
            app.net = loaded.net;
            set(app.ProgressBarAxes, 'visible', 'off');
            set(app.ProgressBarAxes, 'xtick', []);
            UpdateProgress(app);
            set(app.ProgressBarAxes, 'XLim', [0 1]);
            DefaultButtonPushed(app);
            updateAlgorithmControls(app); % Ensure correct panel visibility matches default dropdown value
        end

        % Callback function: ImageDisp, UploadImagesButton
        function UploadImage(app, ~)
            app.InitialLabel.Visible = 'off';

            [files, path] = uigetfile({'*.png;*.jpg;*.tif'}, 'Select Images', 'MultiSelect', 'on');
            if isequal(files, 0), return; end
            if ~iscell(files), files = {files}; end

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

        % Button pushed function: Segment_Button
        function Segment(app, ~)
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

            if strcmp(value, 'Sobel+Watershed')
                % No .mat file to load; just update the visible controls
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
            app.Number_of_Cells_Text.Position             = [496 395 148 24];
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
            app.Number_of_Cells_Field.Position            = [654 395 57 24];

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
            app.DropDown.Items            = {'EfficientNet','ResNet50','ResNet101','CascadeEfficientNet','Sobel+Watershed'};
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