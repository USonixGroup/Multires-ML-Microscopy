
classdef CameraPropertiesPanel < controllib.ui.internal.dialog.AbstractDialog
% CameraPropertiesPanel - Creates camera specific properties and associated
% tearaway.
 
% Copyright 2014-2020 The MathWorks, Inc.
    
    properties
        CurrentSliderValue
        DevicePropObjects
        UIfigure
        SavedResolutionValue
        MinValue
        MaxValue
    end
    
    properties(Access=private)
        CamObj
        ImagePreviewDisplay
        NumCameraProps
    end
 
    methods
        function this = CameraPropertiesPanel(camObj, imPreviewDisplay)
            this.createComponents();
            % Initialization
            this.CamObj = camObj;
            this.ImagePreviewDisplay = imPreviewDisplay;
            this.addButtons();
            
 
            this.updateCameraObject(camObj, imPreviewDisplay);
            this.UIfigure.Visible = 'on';
        end
    end
    
     methods
         
         function createComponents(this)
             this.UIfigure = this.getWidget;
             this.UIfigure.Name = vision.getMessage('vision:caltool:PropertiesButton');
             this.UIfigure.Color = [1 1 1];
             this.UIfigure.CloseRequestFcn = @(fig,event)saveResolution(this);
             this.UIfigure.Resize = matlab.lang.OnOffSwitchState.on;
             this.UIfigure.Visible =  'on';
         end
         
         function saveResolution(this)
             this.SavedResolutionValue = this.DevicePropObjects.Resolution.ComboControl.Value;
             delete(this.UIfigure)
         end
            
         function addButtons(this)
 
             
             % Get the camera controller.
             camController = this.CamObj.getCameraController;
             
             % Get all settable properties.
             props = set(this.CamObj);
             propNames = fieldnames(props);
             sortedProps = sort(propNames(2:end));
             propNames(2:end) = sortedProps;
             
             % Have Mode properties listed before the actual value
             % properties.
             out = strfind(propNames, 'Mode');
             indices = find(~cellfun(@isempty,out));
             if ~isempty(indices)
                 for idx = 1:length(indices)
                     modePropName = propNames{indices(idx)};
                     tempID = strfind(modePropName, 'Mode');
                     expectedPropName = modePropName(1:tempID-1);
                     if strcmpi(expectedPropName, propNames{indices(idx)-1})
                         propNames{indices(idx)-1} = modePropName;
                         propNames{indices(idx)} = expectedPropName;
                     end
                 end
             end
           
             numProperties = length(propNames);
             this.NumCameraProps =  numProperties;
            monitorPos = get(0, 'MonitorPositions');
            location = [monitorPos(1,3), monitorPos(1,4)];
                if numProperties < 5
                    idx = numProperties;
                    k = 2 + idx; % as min num of properties is 2
                else
                   idx = numProperties/2;
                   k = 15 - idx; % as max num of properties in many cases will be 15
                end
             location = [location(1)-location(2) , location(2)-100*k];
             this.UIfigure.Position = [location(1)   location(2)   480  69*idx ];
             pos =  this.UIfigure.Position ;
             for idx =  1:numProperties
                 if ~isempty(props.(propNames{idx}))
                     this.DevicePropObjects.(propNames{idx}).ComboControl =  uidropdown(this.UIfigure,...
                         'Items', props.(propNames{idx}){1}, 'Value', this.CamObj.(propNames{idx}),...
                          'Tag', strcat(propNames{idx}, 'Combo') );
                      this.DevicePropObjects.(propNames{idx}).ComboControl.Position = [180 pos(4)-30*(idx) 91 22];
                     addlistener(this.DevicePropObjects.(propNames{idx}).ComboControl, 'ValueChanged', @(~,~)updateCameraObjectProps(this, propNames{idx}));
                     this.DevicePropObjects.(propNames{idx}).LabelControl =  uilabel(this.UIfigure,'Text', propNames{idx});
                     this.DevicePropObjects.(propNames{idx}).LabelControl.Position = [40 pos(4)-30*(idx) 140 30];
                 else
                     range = camController.getPropertyRange(propNames{idx});
                     this.MinValue = double(range(1));
                     this.MaxValue = double(range(2));
                     this.DevicePropObjects.(propNames{idx}).SliderControl = uislider(this.UIfigure,...
                         'Value', this.CamObj.(propNames{idx}), 'Limits',[this.MinValue, this.MaxValue],...
                         'Tag', strcat(propNames{idx}, 'Slider'),'MajorTickLabelsMode', 'auto',...
                         'MajorTicks',[],'MinorTicks', [], 'MajorTickLabels',{});
                     this.DevicePropObjects.(propNames{idx}).SliderControl.Position = [280 1.01*pos(4)-idx*30 155 3];
                     this.DevicePropObjects.(propNames{idx}).SliderControl.MajorTickLabels = {};
                     addlistener(this.DevicePropObjects.(propNames{idx}).SliderControl,'ValueChanged',@(hobj,~)this.sliderEditControlCallback(hobj, propNames{idx}));
                     % Create text field to enter slider.
                     this.CurrentSliderValue = this.CamObj.(propNames{idx});
                     this.DevicePropObjects.(propNames{idx}).EditControl = uieditfield(this.UIfigure, 'numeric', 'Value',...
                         this.CurrentSliderValue, ...
                         'Tag', strcat( propNames{idx}, 'Edit'),...
                         'RoundFractionalValues', matlab.lang.OnOffSwitchState.on);
                     this.DevicePropObjects.(propNames{idx}).EditControl.Position = [180 pos(4)-idx*30 81 20];
                     addlistener(this.DevicePropObjects.(propNames{idx}).EditControl,'ValueChanged',@(hobj,~)this.sliderEditControlCallback(hobj , propNames{idx}));
                     
                     updateSliderAvailability(this, strcat(propNames{idx}, 'Mode'));
                     this.DevicePropObjects.(propNames{idx}).LabelControl = uilabel(this.UIfigure,'Text', propNames{idx});
                 this.DevicePropObjects.(propNames{idx}).LabelControl.Position = [40 pos(4)-30*(idx) 160 20];
                 end
 
                 
             end
         end
        
        function updateCameraObject(this, camObj, imPreviewDisplay)
            this.CamObj = camObj;
            this.ImagePreviewDisplay = imPreviewDisplay;
        end
    end
    
    methods(Access=private)
        function updateSliderAvailability(this, propName)
            % Update a slider/edit field based on combo box value.
            
            if strfind(propName, 'Resolution') % Resolution is special. 
                [width, height] = this.getResolution;
                this.CamObj.closePreview();
                replaceImage(this.ImagePreviewDisplay, width, height);
                preview(this.CamObj, this.ImagePreviewDisplay.ImHandle);
                return;
            end
            
            idx = strfind(propName, 'Mode');
            if isempty(idx)
                return;
            end
            editPropName = propName(1:idx-1);
            try
                if isfield(this.DevicePropObjects, propName)
                    if strcmpi(this.DevicePropObjects.(propName).ComboControl.Value, 'auto')
                        this.DevicePropObjects.(editPropName).SliderControl.Enable = false;
                        this.DevicePropObjects.(editPropName).EditControl.Enable = false;
                    elseif strcmpi(this.DevicePropObjects.(propName).ComboControl.Value, 'manual')
                        this.DevicePropObjects.(editPropName).SliderControl.Enable = true;
                        this.DevicePropObjects.(editPropName).EditControl.Enable = true;                    
                    end
                end
            catch
                % Do nothing and continue.
            end
        end
        
        function updateCameraObjectProps(this, propName)
            propObject = this.DevicePropObjects.(propName);
            if any(ismember(fieldnames(propObject), 'EditControl'))
                this.CamObj.(propName) = this.DevicePropObjects.(propName).EditControl.Value;
            else
                this.CamObj.(propName) = this.DevicePropObjects.(propName).ComboControl.Value;
                updateSliderAvailability(this, propName);
            end
        end
        
        function sliderEditControlCallback(this, obj, propNames)
            if isa(obj,'matlab.ui.control.Slider')
                this.CurrentSliderValue = obj.Value;
                this.DevicePropObjects.(propNames).EditControl.Value = this.CurrentSliderValue ;
            elseif isa(obj, 'matlab.ui.control.NumericEditField')
                camController = this.CamObj.getCameraController;
                range = camController.getPropertyRange(propNames);
                minValue = range(1);
                maxValue = range(2);
                value = obj.Value;
                if isnan(value)
                    % TODO: Do we need an unnecessary error message?
                    this.DevicePropObjects.(propNames).EditControl.Value = this.CurrentSliderValue;
                    return;
                end
                % Valid value - continue.
                if value < minValue
                    value = minValue;
                elseif value > maxValue
                    value = maxValue;
                end
                
                this.CurrentSliderValue = double(value);
                this.DevicePropObjects.(propNames).EditControl.Value = this.CurrentSliderValue;
                this.DevicePropObjects.(propNames).SliderControl.Value = this.CurrentSliderValue;
            end
            updateCameraObjectProps(this, propNames)
        end
        
        function [width, height] = getResolution(this)
            res = this.CamObj.Resolution;
            idx = strfind(res, 'x');
            width = str2double(res(1:idx-1));
            height = str2double(res(idx+1:end));
        end 
            
    end
end
