% This class defines a template for creating a custom pattern detector, to
% be used for stereo Camera calibration. To access help for this class,
% enter the following at the MATLAB Command Prompt:
%
%   >> doc vision.calibration.PatterDetector
%
% For a reference example, see the following PatternDetector class for
% detecting checkerboards:
%
%   >> edit vision.calibration.stereo.CheckerboardDetector
%
% To use the pattern detector with the Stereo Camera Calibrator App, save
% this file and switch to the Stereo Camera Calibrator if it is still open,
% and select the detector from the detector list.
%
% To import it manually:
%   - Open the Stereo Camera Calibrator App.
%   - Select Add Images and choose the calibration images.
%   - In the Image Properties dialog, select Import, navigate to the class
%     file location, and choose the file.
%   - Choose the detector from the drop-down list to process the calibration images.
 
% Copyright 2021 The MathWorks, Inc.
 
classdef MyStereoPatternDetector < vision.calibration.PatternDetector
    
    %----------------------------------------------------------------------
    properties(Constant)
        % Name: Provide a name for the pattern detector
        Name = 'MyCustomPattern';
    end
     
    %----------------------------------------------------------------------
    properties
        % WorldUnits: Specify a unit measure for the world points
        WorldUnits = 'millimeters';
         
        % Panel: Store the uipanel handle provided using propertiesPanel method
        Panel;
    end
     
    %----------------------------------------------------------------------
    % Define properties to be used during the execution of the methods.
    % These are user-defined properties that can used to manage the
    % detection of image points or the generation of the corresponding
    % world points in the class methods.
    properties       
        %------------------------------------------------------------------
        % Add/Modify code here
        %
        InfoLabelBox;
        %------------------------------------------------------------------
    end
     
    methods       
        %------------------------------------------------------------------
        % Implement the detectPatternPoints method to process the
        % calibration files to detect pattern keypoints to be used for
        % calibration. The detection should produce an
        % M-by-2-numImages-by-2 array of imagePoints and a logical array,
        % pairsUsed of the same size as numImages indicating the detection
        % success.
        %
        % For more help,
        %   >> doc vision.calibration.PatternDetector.detectPatternPoints
        function [imagePoints, pairsUsed] = detectPatternPoints(this, imageFileNames1, imageFileNames2, varargin)           
             
            %--------------------------------------------------------------
            % Add code here
            %--------------------------------------------------------------
             
        end
         
        %------------------------------------------------------------------
        % Implement the generateWorldPoints method to generate x-y world
        % coordinates corresponding to the planar pattern keypoints. The
        % world frame is assumed to be attached to the pattern with the XY
        % plane associated with the pattern plane.
        %
        % For more help,
        %   >> doc vision.calibration.PatternDetector.generateWorldPoints
        function worldPoints = generateWorldPoints(this, varargin)
             
            %--------------------------------------------------------------
            % Add code here
            %--------------------------------------------------------------
             
        end
    end
     
    methods
        %------------------------------------------------------------------
        % Optional: This method will be used to populate the Properties
        % panel in the Image and Pattern Properties dialog. This can be
        % used to provide UI elements to gather information needed for the
        % execution of the detector.
        %
        % For more help,
        %   >> doc vision.calibration.PatternDetector.propertiesPanel
        function propertiesPanel(this, panel)
            this.Panel = panel;
             
            %--------------------------------------------------------------
            % Add/Modify code here
            %
            configureUIComponents(this);
            %--------------------------------------------------------------
        end
         
        %------------------------------------------------------------------
        % Optional: This method will be used to render the origin, X-axis
        % and Y-axis labels in the calibration images displayed in the
        % apps. This method is invoked after the pattern keypoints have
        % been successfully detected in the images.
        %
        % For more help,
        %   >> doc vision.calibration.PatternDetector.drawImageAxesLabels
        function [originLabel, xLabel, yLabel] = drawImageAxesLabels(this, imagePoints)
            %--------------------------------------------------------------
            % Add/Modify code here
            originLabel = struct('Orientation',[],'Location',[]);
            xLabel      = struct('Orientation',[],'Location',[]);
            yLabel      = struct('Orientation',[],'Location',[]);
            %--------------------------------------------------------------
        end
    end
     
    methods (Access = private)
        function configureUIComponents(this)
            %--------------------------------------------------------------
            % Add/Modify code here
            %
            % Example static text in the Properties panel that can be replaced
            % by user-defined UI elements to define properties relevant to the
            % execution of the class methods in the Image and Pattern Properties
            % dialog.
            labelBoxPosition = [0.1 0.2 0.8 0.4];
            this.InfoLabelBox = uilabel('Parent',this.Panel, ...
                'Position', labelBoxPosition,...
                'HorizontalAlignment', 'left',...
                'Text', 'All the properties have been set in the Custom Pattern class');
            %--------------------------------------------------------------
        end
    end
      
end