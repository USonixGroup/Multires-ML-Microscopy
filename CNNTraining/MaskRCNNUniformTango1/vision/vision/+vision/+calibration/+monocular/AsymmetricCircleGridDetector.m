classdef AsymmetricCircleGridDetector < vision.internal.calibration.webTool.CircleGridDetectorImpl
% AsymmetricCircleGridDetector Interface for asymmetric circle grid detection for monocular camera calibration.
%   
%   AsymmetricCircleGridDetector specifies the interface for an asymmetric
%   circle grid detector to be used for single camera calibration in the Camera
%   Calibrator App and from the command line.
%
%   For more information on defining a custom calibration pattern detector
%   using this interface, see the <a href="matlab:help('vision.calibration.PatternDetector')">PatternDetector</a> class:
%
%       edit vision.calibration.PatternDetector

% Copyright 2021 The MathWorks, Inc.

    %----------------------------------------------------------------------
    properties(Constant)
        Name = vision.getMessage('vision:caltool:AsymmetricCircleGrid');
        
        PatternType = 'asymmetric';
    end
end