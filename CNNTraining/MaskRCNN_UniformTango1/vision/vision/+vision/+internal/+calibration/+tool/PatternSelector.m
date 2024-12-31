% PatternSelector A compound UI control for selecting the calibration
% pattern.

% Copyright 2021 The MathWorks, Inc.

classdef PatternSelector < handle
    properties
        Parent;
        Container;

        PatternSelectorGridLayout

        PatternSelectorPanel;
        Label;
        PatternsPopup;
        
        PatternList;
        Location;
        AvailablePatterns;
        NativePatterns;
        IsStereo = false;
    end

    properties
        UnsavedTemplates        matlab.desktop.editor.Document = matlab.desktop.editor.Document.empty;

        PatternDetectors        {}

        PatternDetectorFiles

        CurrentDetector

        CurrentDetectorFileName
    end

    methods
        %------------------------------------------------------------------
        function this = PatternSelector(container, parent, location, initPattern, varargin)
            this.Container = container;
            this.Parent = parent;
            this.Location = location;

            if nargin > 4
                this.IsStereo = varargin{1};
            end

            % Update native patterns
            updateNativePatterns(this);

            addPanel(this);
            addLabel(this);
            addPatternsPopup(this, initPattern);

            % Add user added patterns to the available patterns
            updateAvailablePatterns(this);
        end
        
        %------------------------------------------------------------------
        function pattern = getSelectedPattern(this)
            idx = get(this.PatternsPopup,'value');
            pattern = this.AvailablePatterns{idx};
        end

        %------------------------------------------------------------------
        function disable(this)
            if isvalid(this.Label)
                this.Label.Enable         = 'off';
                this.PatternsPopup.Enable = 'off';
            end
        end

        %------------------------------------------------------------------
        function updateAvailablePatterns(this)

            s = settings;

            if this.IsStereo
                patternList = s.vision.stereoCalibrator.PatternList.ActiveValue;
            else
                patternList = s.vision.calibrator.PatternList.ActiveValue;
            end

            % Reset custom pattern related containers
            this.PatternDetectors = {this.NativePatterns(:).Detector};
            this.PatternDetectorFiles = {this.NativePatterns(:).FileName};
            this.AvailablePatterns = {this.NativePatterns(:).Name};

            % Check pattern validity and remove invalid patterns
            invalidPatterns = false(1, numel(patternList));
            for idx = 1:numel(patternList)

                metaClass = [];
                try
                    if patternList{idx} ~= ""
                        % Verify correctness of pattern detector class
                        [path,fileName] = ...
                            vision.internal.calibration.tool.PatternSelector.getFileParts(patternList{idx});
                        vision.internal.calibration.tool.PatternSelector.checkPatternDetectorFile(patternList{idx}, fileName);

                        % Construct pattern detector object
                        metaClass = meta.class.fromName(fileName);
                    end
                catch checkFileExp
                    errorMsg = sprintf("%s %s: \n%s\n\n%s", vision.getMessage('vision:caltool:PatternPreText'), ...
                        patternList{idx}, checkFileExp.message, vision.getMessage('vision:caltool:RemovedCustomPattern'));
                    uialert(this.Container.FigureHandle, errorMsg, ...
                        vision.getMessage('vision:caltool:LoadingDetectorFailedTitle'));

                end

                if isempty(metaClass)
                    % The path is no longer valid
                    invalidPatterns(idx) = true;
                else
                    % Get super class
                    metaSuperClass = metaClass.SuperclassList;
                    superclasses = {metaSuperClass.Name};

                    % Check for correct subclassing
                    if ~(ismember('vision.calibration.PatternDetector', superclasses) || ...
                            ismember('vision.internal.calibration.PatternDetector', superclasses))
                        invalidPatterns(idx) = true;
                    else
                        % Add to pattern popup list
                        patternName = getPatternName(this, metaClass);
                        this.AvailablePatterns{end + 1} = convertStringsToChars(patternName);

                        % Add directoty to path and instantiate algorithm
                        addpath(path);

                        classHandle = str2func(['@()',fileName,'();']);
                        this.PatternDetectors{end + 1} = classHandle();
                        this.PatternDetectorFiles{end + 1} = fileName;
                    end
                end

            end
            this.PatternsPopup.Items = this.AvailablePatterns;
            
            patternList(invalidPatterns) = [];

            if this.IsStereo
                s.vision.stereoCalibrator.PatternList.PersonalValue = patternList;
            else
                s.vision.calibrator.PatternList.PersonalValue = patternList;
            end
        end

        %------------------------------------------------------------------
        function name = getPatternName(~, metaClass)
            try
                name = eval([metaClass.Name, '.Name']);
            catch
                % Use default name
                name = vision.getMessage('vision:caltool:CustomPatternName');
            end
        end

        %------------------------------------------------------------------
        function updatePatternSelector(this, selection)
            
            this.CurrentDetector = this.PatternDetectors{selection};
            this.CurrentDetectorFileName = this.PatternDetectorFiles{selection};
        end
        
        %------------------------------------------------------------------
        function updatePatternSelectorLocation(this, locPattern)
            
            % Update location of pattern selector panel
            this.Location = locPattern;
            this.PatternSelectorPanel.Position = locPattern;
        end

        %------------------------------------------------------------------
        function selectCurrentDetector(this)
            s = settings;

            if this.IsStereo
                currentPattern = s.vision.stereoCalibrator.CurrentPattern.ActiveValue;
            else
                currentPattern = s.vision.calibrator.CurrentPattern.ActiveValue;
            end

            % Select the saved current pattern from previous sessions if
            % it is still valid
            [~, idx] = ismember(currentPattern, this.PatternsPopup.Items);
            if idx ~= 0
                this.PatternsPopup.Value = this.PatternsPopup.Items{idx};
                feval(this.PatternsPopup.ValueChangedFcn, this.PatternsPopup, []);
            end
        end

    end

    methods
        %------------------------------------------------------------------
        function addAndSelectImportedPattern(this, fullPath, fileName)
            if addToList(this, fullPath, fileName)
                % Select the imported pattern from the list
                recentPatternIdx = numel(this.PatternsPopup.Items);
                this.PatternsPopup.Value = this.PatternsPopup.Items{recentPatternIdx};
                feval(this.PatternsPopup.ValueChangedFcn, this.PatternsPopup, []);
            end
        end
         
        %------------------------------------------------------------------
        function updateUnsavedPatternTemplates(this)

            if isvalid(this) && ~isempty(this.UnsavedTemplates)
                templatesStillOpen = arrayfun(@(x) x.Opened, this.UnsavedTemplates);
                this.UnsavedTemplates(~templatesStillOpen) = [];

                if ~isempty(this.UnsavedTemplates)
                    templatesSaved = ~arrayfun(@(x) x.Modified, this.UnsavedTemplates);
                    for idx = 1:numel(this.UnsavedTemplates)
                        if templatesSaved(idx)

                            fullPath = this.UnsavedTemplates(idx).Filename;
                            [~,fileName] = ...
                                vision.internal.calibration.tool.PatternSelector.getFileParts(fullPath);

                            if addToList(this, fullPath, fileName) && (idx == numel(this.UnsavedTemplates))
                                % Select the last pattern from the list
                                lastPatternIdx = numel(this.PatternsPopup.String);
                                this.PatternsPopup.Value = this.PatternsPopup.Value{lastPatternIdx};
                                feval(this.PatternsPopup.ValueChangedFcn, this.PatternsPopup, []);

                            end
                        end
                    end

                    this.UnsavedTemplates(templatesSaved) = [];
                end

            end

        end

        %------------------------------------------------------------------
        function validatePatternName(this, fileName)
            metaClass = meta.class.fromName(fileName);
            patternName = getPatternName(this, metaClass);

            validateattributes(patternName, {'char', 'string'}, ...
                {'nonempty', 'scalartext'}, fileName, 'Pattern Name');

            availablePatternNames = this.PatternsPopup.Items;

            if any(ismember(availablePatternNames, patternName))
                error(message('vision:caltool:DuplicatePatternName'));
            end
        end
        
        %------------------------------------------------------------------
        function success = addToList(this, fullPath, fileName)
            success = false;
            try
                fullPath = vision.internal.calibration.tool.PatternSelector.checkPatternDetectorFile(fullPath, fileName);
                validatePatternName(this, fileName)
            catch checkFileExp
                errorMsg = sprintf("%s %s: \n\n%s", vision.getMessage('vision:caltool:PatternPreText'), ...
                    fullPath, checkFileExp.message);
                uialert(this.Container.FigureHandle, errorMsg, ...
                    vision.getMessage('vision:caltool:LoadingDetectorFailedTitle'));
                return;
            end

            s = settings;
            if this.IsStereo
                patternList = s.vision.stereoCalibrator.PatternList.ActiveValue;
            else
                patternList = s.vision.calibrator.PatternList.ActiveValue;
            end

            % Add to existing detector list
            if ~ismember(fullPath, patternList) % Avoid duplicate addition
                if this.IsStereo
                    s.vision.stereoCalibrator.PatternList.PersonalValue(end+1) = string(fullPath);
                else
                    s.vision.calibrator.PatternList.PersonalValue(end+1) = string(fullPath);
                end
                success = true;
            end

            updateAvailablePatterns(this);
        end
    end
    
    methods (Static)
        %------------------------------------------------------------------
        function fullPath = checkPatternDetectorFile(fullPath, fileName)

            [~, ~, fcnExt] = fileparts(fullPath);
            fcnPath = fileparts(char(fullPath));

            if ~exist(fullPath,'file')
                error(message('vision:caltool:FileDoesNotExist'));
            end

            if ~strcmpi(fcnExt, '.m')
                error(message('vision:caltool:FileNotMFile'));
            end

            if isempty(fcnPath)
                error(message('vision:caltool:PathNotValid'));
            end

            whichPath = which(fileName);

            if isempty(whichPath)
                addpath(fcnPath);
            elseif ~strcmpi(whichPath, fullPath)
                error(message('vision:caltool:MultipleFilesOnPath'));
            end

            metaClass = meta.class.fromName(fileName);

            metaSuperclass = metaClass.SuperclassList;
            superclasses   = {metaSuperclass.Name};

            if metaClass.Abstract
                error(message('vision:caltool:AbstractClass'));
            end

            if ~ismember({'vision.calibration.PatternDetector'},superclasses)
                error(message('vision:caltool:InvalidParentClass'));
            end
            
        end

        %------------------------------------------------------------------
        function [path,name] = getFileParts(fullpath)

            [path,name,~] = fileparts(char(fullpath));

            if contains(path,'+')
                % There is a plus in the path. Let's assume it is for a
                % package scoped function
                dirs = strsplit(path,filesep);

                if ~isempty(dirs)
                    for idx = numel(dirs):-1:1
                        if contains(dirs{idx},'+')
                            str = strrep(dirs{idx},'+','');
                            name = [str, '.', name]; %#ok<AGROW>
                        else
                            % We hit the top level directory for the
                            % package. Put the path back together to this
                            % folder
                            path = '';
                            for i = 1:idx
                                path = fullfile(path, dirs{i});
                            end
                            break;
                        end
                    end
                end
            end

        end

    end

    methods(Access=private)
        %------------------------------------------------------------------
        function addPanel(this)

            this.PatternSelectorPanel = uipanel( this.Parent,...
                'Title', vision.getMessage('vision:caltool:PatternSelectorPanelTitle'), ...
                'Units', 'pixels',...
                'Tag', 'PatternSelectionPanel');

            this.PatternSelectorGridLayout = uigridlayout(this.PatternSelectorPanel);
            this.PatternSelectorGridLayout.RowHeight = {'fit'};
            this.PatternSelectorGridLayout.ColumnWidth = {'fit','fit'};

        end

        %------------------------------------------------------------------
        function addLabel(this)

            this.Label = uilabel(this.PatternSelectorGridLayout,...
                'Text', vision.getMessage('vision:caltool:ChoosePatternLabel'),...
                'HorizontalAlignment', 'left');
        end

        %------------------------------------------------------------------
        function addPatternsPopup(this, initPattern)
            this.AvailablePatterns = {this.NativePatterns(:).Name};
            
            % Add the pop-up menu
            this.PatternsPopup = uidropdown(this.PatternSelectorGridLayout,...
                'Items', this.AvailablePatterns,...
                'Value', initPattern, ...
                'ValueChangedFcn', @this.doSelectionChanged,...
                'Tag', 'PatternSelectorPopup',...
                'Tooltip', vision.getMessage('vision:caltool:PatternPopupTooltip'));

        end

        %------------------------------------------------------------------
        function updateNativePatterns(this)

            if this.IsStereo
                location = 'stereo';
            else
                location = 'monocular';
            end

            fileList = dir(fullfile(matlabroot, 'toolbox','vision','vision','+vision','+calibration',...
                ['+', location], '*.m'));

            this.NativePatterns = struct('Name','','Detector','','FileName','');

            for idx = 1:numel(fileList)
                [~, fileName] = fileparts(fileList(idx).name);

                fullFileName = ['vision.calibration.', location, '.', fileName];
                
                metaClass = meta.class.fromName(fullFileName);

                % Add to pattern popup list
                patternName = getPatternName(this, metaClass);
                classHandle = str2func(['@()',fullFileName,'();']);

                this.NativePatterns(idx).Name      = convertStringsToChars(patternName);
                this.NativePatterns(idx).Detector  = classHandle();
                this.NativePatterns(idx).FileName  = fullFileName;
            end

            % Reorder native pattern detector list.
            if this.IsStereo
                cameraType = "stereo";
            else
                cameraType = "monocular";
            end
            nativeDetectorsList = vision.internal.calibration.tool.getNativeDetectorsList(cameraType);
            nativePatternsFilenames = {this.NativePatterns(:).FileName};
            idxOrder = [];
            for i = 1:length(nativeDetectorsList)
                idx = find(strcmp(nativeDetectorsList{i}, nativePatternsFilenames));
                if idx ~= 0
                    idxOrder(end+1) = idx; %#ok<AGROW>
                end
            end

            assert(length(idxOrder) == length(this.NativePatterns),...
                vision.getMessage('vision:caltool:MissingNativePattern'));

            this.NativePatterns = this.NativePatterns(idxOrder);
        end

        %------------------------------------------------------------------
        function doSelectionChanged(this, src, ~)
            patternName = string(src.Value);

            s = settings;
            if this.IsStereo
                s.vision.stereoCalibrator.CurrentPattern.PersonalValue = patternName;
            else
                s.vision.calibrator.CurrentPattern.PersonalValue = patternName;
            end

            updateDialog(this.Container, patternName);
        end
    end

end