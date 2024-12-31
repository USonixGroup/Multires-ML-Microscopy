% PatternDetector Interface for defining custom planar pattern detectors.
%
%   PatternDetector specifies the interface for defining custom planar
%   pattern detectors to be used for single and stereo camera calibration
%   with the calibrator apps and from the command line.
%
%   To define a custom pattern detector, you must construct a class that
%   inherits from the vision.calibration.PatternDetector class. This is an
%   abstract class that defines the methods and properties that will be
%   be invoked during the calibration process for the detection
%   of calibration pattern keypoints in images and to generate
%   corresponding points in world coordinates.
%
%   To define a custom pattern detector for single camera calibration,
%   <a href="matlab:vision.OpenPatternDetectorTemplate('monocular')">open this template class</a>.
%   
%   To define a custom pattern detector for stereo camera calibration,
%   <a href="matlab:vision.OpenPatternDetectorTemplate('stereo')">open this template class</a>.
%
%   Application Programming Interface Specification
%   -----------------------------------------------
%
%   Clients of PatternDetector define the following predefined properties:
%
%   <a href="matlab:help('vision.calibration.PatternDetector.Name')">Name</a>          - Pattern name
%   <a href="matlab:help('vision.calibration.PatternDetector.WorldUnits')">WorldUnits</a>    - World point units
%   <a href="matlab:help('vision.calibration.PatternDetector.Panel')">Panel</a>         - uipanel object that can contain any user-defined UI
%                   property components
%
%   Clients of PatternDetector are required to implement the following
%   methods to define the detection and generation of keypoints:
%
%   <a href="matlab:help('vision.calibration.PatternDetector.detectPatternPoints')">detectPatternPoints</a>  - Detect pattern keypoints in images
%   <a href="matlab:help('vision.calibration.PatternDetector.generateWorldPoints')">generateWorldPoints</a>  - Generate keypoint locations in world coordinates
%
%   Clients of PatternDetector can also implement the following methods:
%
%   <a href="matlab:help('vision.calibration.PatternDetector.propertiesPanel')">propertiesPanel</a>     - Define properties panel with optional parameters
%   <a href="matlab:help('vision.calibration.PatternDetector.drawImageAxesLabels')">drawImageAxisLabels</a> - Define location and orientation of origin, X-axis and Y-axis labels
%   Constructor         - A class constructor can be defined, but must take
%                         no arguments
%
%   For a reference example, see the following PatternDetector class
%   implementations for checkerboards:
%
%       % Single camera Calibration
%       edit vision.calibration.monocular.CheckerboardDetector
%
%       % Stereo camera Calibration
%       edit vision.calibration.stereo.CheckerboardDetector
%
%   See also cameraCalibrator, stereoCameraCalibrator.

% Copyright 2021 The MathWorks, Inc.

classdef PatternDetector < handle
   
    %----------------------------------------------------------------------
    properties(Abstract, Constant)
        % Name Pattern name
        %   A string scalar or character vector specifying the name of the pattern.
        Name
    end
    
    %----------------------------------------------------------------------
    properties (Abstract)        
        % WorldUnits World points units
        %   A string scalar or character vector that describes the unit of
        %   measure for the pattern keypoints in world coordinates.
        WorldUnits;

        % Panel Parent uipanel object
        %   A uipanel object that contains any user-defined UI components
        %   to be used for displaying and specifying detector properties.
        %   The propertiesPanel method will be invoked with a uipanel
        %   object as an argument which should be used to set this
        %   property.
        Panel;
    end
    
    %----------------------------------------------------------------------
    % Override these methods to define the image pattern detection and
    % world point generation behavior in the calibrator apps.
    % detectPatternPoints() and generateWorldPoints() must be overridden,
    % while propertiesPanel() and drawImageAxesLabels() are optional.
    %----------------------------------------------------------------------
    methods (Abstract)        
        % detectPatternPoints
        %   detectPatternPoints is invoked to detect the planar pattern keypoints
        %   in the calibration images.
        %
        %   Single Camera Calibrator
        %   ------------------------
        %
        %   [imagePoints, imagesUsed] = detectPatternPoints(detectorObj, imageFileNames)
        %   detects the calibration pattern in images specified by
        %   imageFileNames string array. imagePoints is an M-by-2-by-numImages
        %   array of x-y coordinates, where numImages is the number of
        %   images in which the pattern was detected. imagesUsed is a
        %   logical vector of the same size as imageFileNames and a value
        %   of true in the array indicates that the pattern was detected in
        %   the corresponding image. If the complete pattern cannot be
        %   detected, a partially detected pattern can be returned with
        %   [NaN, NaN] as x-y coordinates of missing keypoints in
        %   imagePoints.
        %
        %   Stereo Camera Calibrator
        %   ------------------------
        %
        %   [imagePoints, pairsUsed] = detectPatternPoints(detectorObj, imageFileNames1, imageFileNames2)
        %   detects calibration patterns in stereo pairs of images
        %   specified by imageFileNames1 and imageFileNames2 string arrays.
        %   imagePoints is an M-by-2-by-numPairs-by-2 array of x-y
        %   coordinates. imagePoints(:,:,:,1) contains the points from the first
        %   set of images, imageFileNames1 and imagePoints(:,:,:,2) contains the
        %   points from the second set, imageFileNames2. pairsUsed is a
        %   logical array of the same size as imageFileNames1 and a value of
        %   true in the array indicates that the pattern was detected in
        %   the corresponding image pair. Stereo camera calibration does
        %   not support partial pattern detection.
        %
        %   Notes
        %   -----
        %   - For command line usage, use varargin input to parse any
        %     additional parameters or options needed for detection. For
        %     example, the parameters or options obtained using the
        %     properties panel can be provided through this interface while
        %     using it from the command line. The function syntax will be:
        %
        %     % Single camera calibration
        %     [imagePoints, imagesUsed] = detectPatternPoints(detectorObj, imageFileNames, varargin)
        %
        %     % Stereo camera calibration
        %     [imagePoints, pairsUsed] = detectPatternPoints(detectorObj, imageFileNames1, imageFileNames2, varargin)
        [imagePoints, logicalArray] = detectPatternPoints(this, varargin);
    end
        
    %----------------------------------------------------------------------
    methods (Abstract) 
       % generateWorldPoints
       %   generateWorldPoints is invoked to generate world coordinates for the
       %   planar pattern keypoints.
       %
       %   worldPoints = generateWorldPoints(detectorObj) returns an M-by-2 array
       %   containing x-y coordinates of the keypoints of the calibration
       %   pattern in world units. The number of points, M, should be the
       %   same as the number of pattern keypoints detected in the
       %   calibration images using the detectPatternPoints method.
       %
       %   Notes:
       %   ------
       %   - The pattern keypoints in worldPoints must be arranged so that
       %     they correspond to the keypoints detected in the calibration
       %     images using the detectPatternPoints method.
       %
       %   - For command line usage, use varargin input to parse any
       %     additional parameters or options needed to generate world
       %     points. For example, the parameters or options obtained using
       %     the properties panel can be provided through this interface
       %     while using it from the command line. The function syntax
       %     will be:
       %
       %     worldPoints = generateWorldPoints(detectorObj, varargin)
        worldPoints = generateWorldPoints(this);
    end
    
    %----------------------------------------------------------------------
    methods
        function propertiesPanel(this, panel)
            % propertiesPanel
            %   propertiesPanel will be invoked when a custom pattern is selected
            %   from the available patterns in the calibrator apps. This method can be
            %   used to define UI elements in the Properties panel in the Image and Pattern
            %   Properties dialog for setting detector related properties.
            %
            %   propertiesPanel(detectorObj, panel) is invoked to display these
            %   properties in the parent uipanel object specified in the panel
            %   argument.
        end
        
        function [originLabel, xLabel, yLabel] = drawImageAxesLabels(this, imagePoints)
             % drawImageAxesLabels
             %   drawImageAxesLabels will be invoked from the calibrator apps while
             %   rendering the origin, X-axis and Y-axis labels in the image representing
             %   the world frame attached to the pattern.
             %
             %   [originLabel, xLabel, yLabel] = drawImageAxesLabels(detectorObj, imagePoints)
             %   returns the orientation and location of the origin, X-axis and
             %   Y-axis labels in the image frame. imagePoints is an M-by-2 array of
             %   x-y image coordinates corresponding to the pattern keypoints
             %   determined by invoking the detectPatternPoints method.
             %
             %   Here, originLabel, xLabel and yLabel are structs with the following
             %   fields:
             %       Orientation - Rotation of the axis label in the image frame in degrees
             %       Location    - [x,y] location of the axis label in the image frame
             %
             %   Notes
             %   -----
             %   - If this method is not implemented or if empty structs are returned,
             %     the origin and axes labels are not rendered in images displayed by the
             %     calibrator apps.
            originLabel = struct('Orientation',[],'Location',[]);
            xLabel      = struct('Orientation',[],'Location',[]);
            yLabel      = struct('Orientation',[],'Location',[]);
        end
    end
end