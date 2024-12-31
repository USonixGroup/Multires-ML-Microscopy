%PeopleDetectorACF Automation algorithm to detect people using ACF.
%   PeopleDetectorACF is an automation algorithm for detecting people using
%   Aggregated Channel Features in the Video Labeler and the Ground Truth
%   Labeler Apps. Ground Truth Labeler App requires that you have the
%   Automated Driving Toolbox(TM).
%
%   See also imageLabeler, vision.labeler.AutomationAlgorithm,
%   detectPeopleACF, videoLabeler

% Copyright 2017-2024 The MathWorks, Inc.

classdef PeopleDetectorACF < vision.labeler.AutomationAlgorithm
    
    %----------------------------------------------------------------------
    % Algorithm Description
    properties(Constant)
        
        %Name
        %   Character vector specifying name of algorithm.
        Name = vision.getMessage('vision:labeler:PeopleDetectorACFName');
        
        %Description
        %   Character vector specifying short description of algorithm
        Description = vision.getMessage('vision:labeler:PeopleDetectorACFDesc');
        
        %UserDirections
        %   Cell array of character vectors specifying directions for
        %   algorithm users to follow in order to use algorithm.
        UserDirections = {...
            vision.labeler.AutomationAlgorithm.getDefaultUserDirections('selectroidef', vision.getMessage('vision:labeler:People')),...
            vision.labeler.AutomationAlgorithm.getDefaultUserDirections('rundetector', vision.getMessage('vision:labeler:People')),...
            vision.labeler.AutomationAlgorithm.getDefaultUserDirections('review'),...
            vision.labeler.AutomationAlgorithm.getDefaultUserDirections('rerun'),...
            vision.labeler.AutomationAlgorithm.getDefaultUserDirections('accept')...
            };
    end
    
    %---------------------------------------------------------------------
    % Properties
    properties
        
        %Detector
        %   Classification accuracy threshold
        Detector
        
        %SettingsHandles
        %   Cell array of handles for the UIcontrol settings objects
        SettingsHandles
        
        %ACFModelIdx
        %   Index of ACF classification model to be used
        ACFModelIdx = 1
        
        %ACFModelNames
        %   Cell array of character vectors containing ACF classification
        %   models
        ACFModelNames = {
            'inria-100x41'
            'caltech-50x21'}
        
        %OverlapThreshold
        %   Overlap ratio threshold for bounding boxes
        OverlapThreshold = 0.65
        
        %ScoreThreshold
        %   Classification score threshold
        ScoreThreshold = 0
        
    end
    
    %----------------------------------------------------------------------
    % Setup
    methods
        
        function isValid = checkLabelDefinition(~, labelDef)
            
            % Only labels for rectangular ROI's are considered valid.
            isValid = labelDef.Type == labelType.Rectangle;
            
        end
        
        function isReady = checkSetup(this)
            
            % Expect there to be at least one label to automate.
            isReady = ~isempty(this.SelectedLabelDefinitions);
            
        end
        
        function settingsDialog(this)
            
            dialogSettings(1) = struct(...
                'Tag', 'ACFModelIdx',...
                'Style', 'popupmenu',...
                'String', {this.ACFModelNames},...
                'Value', this.ACFModelIdx,...
                'Range', [],...
                'PromptStr', vision.getMessage('vision:labeler:ACFModelName'));
            
            dialogSettings(2) = struct(...
                'Tag', 'OverlapThreshold',...
                'Style', 'slider',...
                'String', this.OverlapThreshold,...
                'Value', this.OverlapThreshold,...
                'Range', [0 1],...
                'PromptStr', vision.getMessage('vision:labeler:ACFOverlapThreshold'));
            
            dialogSettings(3) = struct(...
                'Tag', 'ScoreThreshold',...
                'Style', 'edit',...
                'String',  this.ScoreThreshold,...
                'Value', this.ScoreThreshold,...
                'Range', [],...
                'PromptStr', vision.getMessage('vision:labeler:ACFClassifyScoreThreshold'));
            
            createSettingsDialog(this, dialogSettings);
            
        end
    end
    
    %----------------------------------------------------------------------
    % Execution
    methods
        
        function initialize(this, ~)
            
            % Setup the detector for the specified model
            modelName = this.ACFModelNames{this.ACFModelIdx};
            this.Detector = peopleDetectorACF(modelName);
            
        end
        
        function automatedLabels = run(this, I)
            
            automatedLabels = [];
            
            % Detect people using aggregate channel features
            [bboxes, scores] = detect(this.Detector, I,...
                'SelectStrongest', false);
            
            % Apply non-maximum suppression to select the strongest bounding boxes.
            [selectedBboxes, selectedScores] = selectStrongestBbox(bboxes, scores,...
                'RatioType', 'Min',...
                'OverlapThreshold', this.OverlapThreshold);
            
            % Consider only detections that meet specified score threshold
            selectedBboxes = selectedBboxes(selectedScores > this.ScoreThreshold, :);
            
            if ~isempty(selectedBboxes)
                
                % Add the selected label at the bounding box position(s)
                automatedLabels = struct(...
                    'Type', labelType.Rectangle,...
                    'Name', this.SelectedLabelDefinitions.Name,...
                    'Position', selectedBboxes);
                
            end
            
        end
        
    end
    
    %----------------------------------------------------------------------
    % Settings methods
    methods(Access = private)
        
        function createSettingsDialog(algObj, dialogSettings)
            
            numSettings = size(dialogSettings, 2);
            numPanels = numSettings * 2;
            
            algObj.SettingsHandles = cell(numSettings, 1);
            
            [parentDlg, dlgPanels] = createParentDialog(numSettings);
            
            for idx = 1:numSettings
                
                % Create control and prompt subpanels
                panels = createSettingPanels(dlgPanels.settings, idx, numPanels);
                
                createPrompt(panels.PromptPanel, dialogSettings(idx).PromptStr);
                algObj.SettingsHandles{idx} = createControl(panels.ControlPanel, dialogSettings(idx));
                
            end
            
            % Create the OK and Cancel buttons and define callbacks
            okBtn = createBtns(dlgPanels.btns);
               
            okBtn.ButtonPushedFcn = {@acceptSettings, numSettings};
            
            uiwait(parentDlg);
            
            function [parentDlg, dlgPanels] = createParentDialog(numSettings)
                
                dlgName = sprintf('%s %s', algObj.Name, ...
                    vision.getMessage('vision:labeler:Settings'));
                % Set the size of the dialog
                width = 325;
                height = numSettings * 80 + 50;
          
                parentDlg = uifigure('Name', dlgName, ...
                    'Resize', 'off',...
                    'Visible', 'off');
                matlab.graphics.internal.themes.figureUseDesktopTheme(parentDlg);

                dlgPanels.settings = uipanel(...
                    'Parent', parentDlg,...
                    'Units', 'pixels',...
                    'BorderType', 'none',...
                    'Position', [0 60 width height-60]);    
            
                dlgPanels.btns = uipanel(...
                    'Parent', parentDlg,...
                    'Units', 'pixels',...
                    'BorderType', 'none',...
                    'Position', [0 0 width 60]);
                
                
                parentDlg.Position(3:4) = [width height];
                % Position the dialog in the center of the screen and display
                movegui(parentDlg, 'center');
                parentDlg.Visible = 'on';
          
            end
            
            function [settingPanels] = createSettingPanels(parentDlg, idx, numPanels)
                
                xLocation = 0.1;
                width = 1 - 2 * xLocation;
                height = 1/numPanels;
                
                % Setting prompt panel object
                promptYLocation = round((numPanels - (idx * 2 - 1))/numPanels, 2);
           
                settingPanels.PromptPanel = uipanel(...
                    'Units','normalized',...
                    'Parent', parentDlg,...
                    'BorderType', 'none',...
                    'Position',  [xLocation promptYLocation width height]);
                
                % Setting control panel object
                controlYLocation = round((numPanels - (idx * 2))/numPanels, 2);
                settingPanels.ControlPanel = uipanel(...
                    'Units','Normalized',...
                    'Parent', parentDlg,...
                    'BorderType', 'none',...
                    'Position', [xLocation controlYLocation width height]);
            end
            
            function createPrompt(promptPanel, promptStr)
                
                % Create UI Control object for the prompt
               
                set(promptPanel,'Units','pixels');
                w = promptPanel.Position(3);
                % Create uicomponent object for the prompt 
                uilabel(...
                    'Parent', promptPanel,...
                    'HorizontalAlignment', 'left',...
                    'Position', [1 1 w - 4 25],...
                    'Text', promptStr);
            end
            
            function [hControl] = createControl(controlPanel, dialogSetting)
                    
                % Create uicomponent object for the setting control
                set(controlPanel,'Units','pixels');
                w = controlPanel.Position(3) - 4;
                switch dialogSetting.Style
                    
                    case 'popupmenu'
                        idx = dialogSetting.Value;
                        hControl = uidropdown('Parent', controlPanel,...
                            'Value',dialogSetting.String{idx} ,...
                            'Items', dialogSetting.String(:),...
                            'Position', [1 1 w 25],...
                            'Tag', dialogSetting.Tag);
        
                    case 'edit'
                        hControl = uieditfield('numeric','Parent', controlPanel, ...
                            'Value',dialogSetting.String,...
                            'HorizontalAlignment','center',...
                            'Position', [1 15 65 20],...
                            'Tag', dialogSetting.Tag);
                        
                    case 'slider'
                        
                        hControl = uislider('Parent', controlPanel, ...
                            'Limits',[dialogSetting.Range(1) dialogSetting.Range(2)],...
                            'MajorTicks',[],...
                            'MinorTicks',[],...
                            'Value', dialogSetting.Value,...
                            'Tag',dialogSetting.Tag);
                        h = hControl.Position(4);
                        w = w - 50;
                        hControl.Position =  [2 12 w h];

                        % Object to display current slider value
                        hSliderDisplay = uilabel(...
                            'Parent', controlPanel,...
                            'Position', [w+7 1 48.75 25],...
                            'HorizontalAlignment', 'left',...
                            'Tag', [dialogSetting.Tag 'Display'],...
                            'Text', num2str(dialogSetting.Value));
                        
                        hControl.ValueChangedFcn = {@sliderCallback, hSliderDisplay};
                end
                
                         
                function sliderCallback(hSlider, ~, hSliderDisplay)
                     
                    hSliderDisplay.Text = sprintf('%0.2f', hSlider.Value);
                        
                end
            end
            
            function [okBtn] = createBtns(btnsSection)
                
                % Button string values
                cancelBtnStr = vision.getMessage('vision:labeler:SettingsCancelButton');
                okBtnStr = vision.getMessage('vision:labeler:SettingsOKButton');
                
                btnWidth = max(strlength({okBtnStr cancelBtnStr})) + 5;
                
                % Get the center of the section
                btnWidth = btnWidth * 6;
               
                btnsSectionWidth = get(btnsSection, 'Position');
                centerXLocation = btnsSectionWidth(3)/2;
                
                okBtnXLocation = centerXLocation - (btnWidth + 1);
                cancelBtnXLocation = centerXLocation + 1;
                
                % OK button object
                okBtn = uibutton('Parent', btnsSection,...
                    'Position', [okBtnXLocation 28 btnWidth 28],...
                    'Text', okBtnStr);

                % Cancel button object
                cancelBtn = uibutton('Parent', btnsSection,...
                    'Position', [cancelBtnXLocation 28 btnWidth 28],...
                    'Text', cancelBtnStr);
                cancelBtn.ButtonPushedFcn = @(e,d)onCancel(cancelBtn,e,d);
                  
            end
            
            function acceptSettings(~, ~, numSettings)
                
                for ndx = 1:numSettings
                    
                    setting = algObj.SettingsHandles{ndx};
                    
                    switch setting.Type
                        case 'uidropdown'
                            idx = find(strcmp(dialogSettings(1).String,setting.Value));
                            algObj.(setting.Tag) = idx;
                        case 'uinumericeditfield'
                            algObj.(setting.Tag) = setting.Value;
                        case 'uislider'
                            algObj.(setting.Tag) = round(setting.Value, 2);
                    end
                    
                end
               
                % Close the dialog
           
                f = ancestor(algObj.SettingsHandles{1},'figure');
                delete(f);
               
            end
            
        end
    end
end


function onCancel(btn,~,~)  
    f = ancestor(btn,'figure');
    delete(f);    
end
