function cameraCalibrator(varargin)

% Copyright 2012-2023 The MathWorks, Inc.

if nargin > 0
    [varargin{:}] = convertStringsToChars(varargin{:});
end

import vision.internal.calibration.tool.*;
shouldAddImages = false;
shouldOpenSession = false;

% A single argument means either 'close' or load a session.
if nargin == 1
    %{
    if strcmpi(varargin{1}, 'close')
        % Handle the 'close' request
        CameraCalibrationTool.deleteAllTools();    
        return;
    %}
    if exist(varargin{1}, 'file') || exist([varargin{1}, '.mat'], 'file')
        % Load a session
        sessionFileName = varargin{1};
        [sessionPath, sessionFileName] = parseSessionFileName(sessionFileName);  
        shouldOpenSession = true;
    else
        error(message('vision:caltool:unknownMatFileLoaded', varargin{1}));
    end
end

if nargin > 0 && ~shouldOpenSession
    % Adding images from folders
    narginchk(2, 4);
    [fileNames, squareSize, units, highDistortion] = parseInputs(varargin{:});
    shouldAddImages = true;
end

% Create a new Camera Calibrator
tool = vision.internal.calibration.webTool.CameraCalibrationTool;
tool.show();

if shouldAddImages
    % Set up checkerboard detector
    detector = vision.calibration.monocular.CheckerboardDetector();
    detectorFile = 'vision.calibration.monocular.CheckerboardDetector';


    detector.SquareSize = squareSize;
    detector.WorldUnits = units;
    detector.IsDistortionHigh = highDistortion;
    
    addImagesToNewSession(tool, fileNames, detector, detectorFile);
elseif shouldOpenSession
    processOpenSession(tool, sessionPath, sessionFileName);
end

%--------------------------------------------------------------------------
function [fileNames, squareSize, units, highDistortion] = parseInputs(varargin)
import vision.internal.calibration.tool.*;

folder = varargin{1};
validateattributes(folder, {'char'}, {'vector'}, mfilename, 'folder');
if ~exist(folder, 'dir')
    error(message('vision:caltool:stereoFolderDoesNotExist', folder));
end
folder = vision.internal.getFullPath(folder);

squareSize = varargin{2};
vision.internal.calibration.checkSquareSize(squareSize, mfilename);

units = 'mm';
highDistortion = false; % Typically set to true for fisheye lenses

if nargin == 3
    if isnumeric(varargin{3})
        highDistortion = checkHighDistortion(varargin{3});
    else
        units = checkSquareSizeUnits(varargin{3});
    end
elseif nargin == 4
    units = checkSquareSizeUnits(varargin{3});
    highDistortion = checkHighDistortion(varargin{4});
end

fileNames = vision.internal.getAllImageFilesFromFolder(folder);
if isempty(fileNames)
    error(message('vision:caltool:noImagesFound', folder));
end

%--------------------------------------------------------------------------
function squareSizeUnits = checkSquareSizeUnits(squareSizeUnits)
squareSizeUnits = validatestring(squareSizeUnits, {'mm', 'cm', 'in',...
    'millimeters', 'centimeters', 'inches'}, mfilename, 'units');

% In 17a we switched from short unit names to full unit names
% to better support translation of the tool.  Substitute new strings
% upon loading of an old session file.
switch squareSizeUnits
    case {'mm'}
        squareSizeUnits = 'millimeters';
    case {'cm'}
        squareSizeUnits = 'centimeters';
    case {'in'}
        squareSizeUnits = 'inches';
end

%--------------------------------------------------------------------------
function tf = checkHighDistortion(highDistortion)
validateattributes(highDistortion, {'logical', 'numeric'},...
    {'scalar','binary'}, mfilename, 'HighDistortion');
tf = true;