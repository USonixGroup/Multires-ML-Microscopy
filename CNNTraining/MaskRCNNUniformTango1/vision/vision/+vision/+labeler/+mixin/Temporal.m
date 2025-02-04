%vision.labeler.mixin.Temporal Mixin for adding temporal context.
%   The Temporal mixin is a base class for AutomationAlgorithm objects that
%   define temporal properties needed for writing a temporal automation algorithm.
%
%   Note that the Temporal mixin can only be added to automation algorithms
%   to be used by the Video Labeler and the Ground Truth Labeler Apps.
%   Ground Truth Labeler App requires that you have the Automated Driving
%   System Toolbox(TM). Labels of type PixelLabel cannot be imported into
%   an AutomationAlgorithm using the Temporal context, using the checkSetup
%   or initialize methods.
%
%   To define a temporal automation algorithm class to be used by the
%   Ground Truth Labeler App, inherit from the 
%   vision.labeler.AutomationAlgorithm and vision.labeler.mixin.Temporal
%   base classes. Alternatively, <a href="matlab:vision.labeler.AutomationAlgorithm.openTemplateInEditor('temporal')">open this template class</a> and follow the
%   outlined steps.
%
%
%   Application Programming Interface Specification
%   -----------------------------------------------
%   The Temporal mixin class defines the following pre-defined properties:
%
%   <a href="matlab:help('vision.labeler.mixin.Temporal.StartTime')">StartTime</a>            - Start time stamp of algorithm interval
%   <a href="matlab:help('vision.labeler.mixin.Temporal.CurrentTime')">CurrentTime</a>          - Time stamp of current frame
%   <a href="matlab:help('vision.labeler.mixin.Temporal.EndTime')">EndTime</a>              - End time stamp of algorithm interval
%   <a href="matlab:help('vision.labeler.mixin.Temporal.StartFrameIndex')">StartFrameIndex</a>      - Start frame index of algorithm interval
%   <a href="matlab:help('vision.labeler.mixin.Temporal.EndFrameIndex')">EndFrameIndex</a>        - End frame index of algorithm interval
%   <a href="matlab:help('vision.labeler.mixin.Temporal.AutomationDirection')">AutomationDirection</a>  - The direction of automation - 'forward' or 'reverse'
%
%   Client class can optionally implement the following method:
%   
%   <a href="matlab:help('vision.labeler.mixin.Temporal.supportsReverseAutomation')">supportsReverseAutomation</a>    - Set reverse automation support flag  

%   See also vision.labeler.AutomationAlgorithm, videoLabeler.


% Copyright 2017-2023 The MathWorks, Inc.

classdef (Abstract) Temporal < handle
    
    %----------------------------------------------------------------------
    % Use these properties to query information about the state of
    % algorithm execution.
    %----------------------------------------------------------------------
    properties (Dependent = true, GetAccess = public, SetAccess = private)
        %StartTime Start time stamp of algorithm interval
        %   Time stamp corresponding to start of algorithm interval. This
        %   is the time stamp corresponding to the first frame over which
        %   algorithm is executed.
        StartTime
        
        %EndTime End time stamp of algorithm interval
        %   Time stamp corresponding to end of algorithm interval. This is
        %   the time stamp corresonding to the last frame over which
        %   algorithm is executed.
        EndTime
        
        %StartFrameIndex Start frame index of algorithm interval
        %   Frame index corresponding to first frame over which algorithm
        %   is executed.
        StartFrameIndex
        
        %EndFrameIndex End frame index of algorithm interval
        %   Frame index corresponding to last frame over which algorithm is
        %   executed.
        EndFrameIndex
    end
    
    properties (GetAccess = public, SetAccess = private)
        %CurrentTime Time stamp of current frame
        %   Time stamp of current frame, updated as algorithm is executing.
        CurrentTime        
        
        %AutomationDirection Specifies the direction of automation
        %   The direction of automation can be either 'forward' or
        %   'reverse'.
        AutomationDirection = 'forward';   
    end
    
    %----------------------------------------------------------------------
    % Implementation
    %----------------------------------------------------------------------
    properties (Access = private)
        %TimeRange Time stamps of algorithm execution
        %   Time range over which algorithm is executed, specified as a
        %   two-element vector [tStart tEnd].
        TimeRange
        
        %TimeRangeIndices Indices corresponding to time range
        %   Indices for time range over which algorithm is operating,
        %   specified as a two-element vector [idxStart idxEnd]. These are
        %   indices into the image sequence or video data source.
        TimeRangeIndices
        
        %ImportedLabels Labels imported for automation
        %   All labels imported into the automation workflow.
        ImportedLabels
    end
    
    methods
        %------------------------------------------------------------------
        function flag = supportsReverseAutomation(~)      
        %supportsReverseAutomation Set reverse automation support flag. 
        %   flag = supportReverseAutomation(algObj) returns true or false
        %   to indicate whether the temporal automation algorithm, algObj,
        %   supports automation in the reverse direction. A true value
        %   enables the Ground Truth Labeler to open the algorithm in
        %   reverse mode.
        %
        %   Inputs:
        %   -------
        %   algObj   - Temporal automation algorithm object.         
        %
        %   See also vision.labeler.mixin.Temporal.AutomationDirection.        
            flag = false;
        end        
        %------------------------------------------------------------------
        function startTime = get.StartTime(algObj)
            
            startTime = algObj.TimeRange(1);
        end
        
        %------------------------------------------------------------------
        function endTime = get.EndTime(algObj)
            
            endTime = algObj.TimeRange(2);
        end
        
        %------------------------------------------------------------------
        function startIdx = get.StartFrameIndex(algObj)
            
            startIdx = algObj.TimeRangeIndices(1);
        end
        
        %------------------------------------------------------------------
        function endIdx = get.EndFrameIndex(algObj)
            
            endIdx = algObj.TimeRangeIndices(2);
        end
    end
    
    methods (Static, Hidden, Sealed)
        function text = getDefaultUserDirections(name,varargin)
            switch name
                case 'selectroidef'
                    labelName = varargin{1};
                    text = vision.getMessage('vision:labeler:SelectROIDefinitionInstruction', labelName);
                case 'rundetector'
                    labelName = varargin{1};
                    text = vision.getMessage('vision:labeler:RunDetectorInstruction', labelName);
                case 'review'
                    text = vision.getMessage('vision:labeler:ReviewInstructionTemporal');
                case 'rerun'
                    text = vision.getMessage('vision:labeler:RerunInstruction');
                case 'accept'
                    text = vision.getMessage('vision:labeler:AcceptInstruction');
            end
        end
    end
    
    methods (Access = {?vision.internal.labeler.tool.LabelerTool, ...
            ?vision.internal.labeler.tool.AlgorithmSetupHelper, ...
            ?vision.internal.imageLabeler.multiUser.view.AutomationTab,...
            ?vision.internal.labeler.multiUser.view.tabs.AutomationTab,...
            ?vision.internal.labeler.multiUser.controller.LabelerToolMultiUser, ...
            ?vision.internal.labeler.multiUser.view.View, ...
            ?matlab.unittest.TestCase}, Sealed)
        %------------------------------------------------------------------
        % Setup
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        function setAlgorithmTimes(this, interval, intervalIndices)
            
            this.TimeRange          = interval;
            this.TimeRangeIndices   = intervalIndices;
        end
        
        %------------------------------------------------------------------
        function setAutomationDirection(this, isAutomationFwd)
            
            if isAutomationFwd
                this.AutomationDirection = 'forward';
            else
                this.AutomationDirection = 'reverse';
            end
        end
        
        %------------------------------------------------------------------
        function importLabels(this, impLabels)
            
            if size(this.SignalName,1)==1
                if ~isempty(impLabels)
                    assert(isstruct(impLabels) && isfield(impLabels,'Type') && isfield(impLabels,'Name') && isfield(impLabels,'Position'),'Expected a struct with fields Type, Name and Position.');
                    impLabels = struct2table(impLabels,'AsArray', true);
                end
            else
                if ~isempty(impLabels)
                    for i=1:size(impLabels,2)
                        if ~isempty(impLabels{i})
                            assert(isstruct(impLabels{i}) && isfield(impLabels{i},'Type') && isfield(impLabels{i},'Name') && isfield(impLabels{i},'Position'),'Expected a struct with fields Type, Name and Position.');
                            impLabels{i} = struct2table(impLabels{i},'AsArray', true);
                        end
                    end
                end
            end
            this.ImportedLabels = impLabels;
        end
        
        %------------------------------------------------------------------
        function isReady = verifyAlgorithmSetup(this)
            
            isReady = checkSetup(this, this.ImportedLabels);
        end
        
        %------------------------------------------------------------------
        function doInitialize(this, I)
            if size(this.SignalName,1)==1
                if ~isempty(this.ImportedLabels) % g2746954
                    for i = 1 : numel(this.ValidLabelDefinitions)
                        if this.ValidLabelDefinitions(i).Type == labelType.Cuboid
                            % Compare label names, to get vector of indices of
                            % the imported ROI labels of Type Cuboid
                            idx = find(strcmp(this.ImportedLabels.Name, this.ValidLabelDefinitions(i).Name));
                            % if indices are not empty, loop through each of
                            % the index and change the label type
                            if any(idx)
                                for n = idx'
                                    this.ImportedLabels.Type{n} = labelType.Cuboid;
                                end
                            end
                        end
                    end
                end
            else
                for i = 1:size(I,1)
                    if ~isempty(this.ImportedLabels) && ~isempty(this.ImportedLabels{i}) % g2746954
                        for j = 1 : numel(this.ValidLabelDefinitions)
                            if this.ValidLabelDefinitions(j).Type == labelType.Cuboid
                                idx = find(strcmp(this.ImportedLabels{i}.Name, this.ValidLabelDefinitions(j).Name));
                                if any(idx)
                                    for n = idx'
                                        % Additional check for the position of
                                        % the label to ensure only Type of the
                                        % Cuboid label is being changed
                                        if size(this.ImportedLabels{i}.Position(n,:), 2) == 9
                                            this.ImportedLabels{i}.Type{n} = labelType.Cuboid;
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
            initialize(this, I, this.ImportedLabels);
        end
        
        %------------------------------------------------------------------
        function updateCurrentTime(this, time)

            if time < this.TimeRange(1)
                time = this.TimeRange(1);
            elseif time > this.TimeRange(2)
                time = this.TimeRange(2);
            end
            
            this.CurrentTime = time;
        end
        
    end
end
