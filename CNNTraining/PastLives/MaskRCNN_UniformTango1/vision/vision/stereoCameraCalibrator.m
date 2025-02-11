function stereoCameraCalibrator(varargin)

%   Copyright 2014-2023 The MathWorks, Inc.

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
    end
end

if nargin > 0 && ~shouldOpenSession
    % Adding images from folders
    narginchk(3, 4);
    [fileNames, squareSize, units] = parseInputs(varargin{:});
    shouldAddImages = true;
end

% Create a new Stereo Calibrator
isStereo = true;
tool = vision.internal.calibration.webTool.CameraCalibrationTool(isStereo);
tool.show();

if shouldAddImages
    % Set up checkerboard detector
    detector = vision.calibration.stereo.CheckerboardDetector();
    detectorFile = 'vision.calibration.stereo.CheckerboardDetector';

    detector.SquareSize = squareSize;
    detector.WorldUnits = units;
    
    addImagesToNewSession(tool, fileNames, detector, detectorFile);
elseif shouldOpenSession
    processOpenSession(tool, sessionPath, sessionFileName);
end

%--------------------------------------------------------------------------
function [fileNames, squareSize, units, highDistortion] = parseInputs(varargin)
import vision.internal.calibration.tool.*;
folder1 = varargin{1};
folder2 = varargin{2};

validateattributes(folder1, {'char'}, {'vector'}, mfilename, 'folder1');
validateattributes(folder2, {'char'}, {'vector'}, mfilename, 'folder2');
errorMsg = checkStereoFolders(folder1, folder2);

if ~isempty(errorMsg)
    error(errorMsg);
end

folder1 = vision.internal.getFullPath(folder1);
folder2 = vision.internal.getFullPath(folder2);

squareSize = varargin{3};
vision.internal.calibration.checkSquareSize(squareSize, mfilename);

% High distortion mode for detecting checekerboards is disabled for stereo
% camera calibration. Will reconsider adding it once we support fisheye
% cameras for stereo.
highDistortion = false;

if nargin < 4
    units = 'mm';
else
    units = checkSquareSizeUnits(varargin{4});
end

fileNames1 = vision.internal.getAllImageFilesFromFolder(folder1);
fileNames2 = vision.internal.getAllImageFilesFromFolder(folder2);

errorMsg = checkStereoFileNames(fileNames1, fileNames2, folder1, folder2);
if ~isempty(errorMsg)
    error(errorMsg);
end

fileNames = [fileNames1; fileNames2];

%--------------------------------------------------------------------------
function squareSizeUnits = checkSquareSizeUnits(squareSizeUnits)
availableUnits = vision.internal.calibration.availableWorldUnits;
squareSizeUnits = validatestring(squareSizeUnits, [{'mm', 'cm', 'in'},...
    availableUnits], mfilename, 'units');

% In 17a we switched from short unit names to full unit names
% to better support translation of the tool.  Substitute new strings
% upon loading of an old session file.
switch squareSizeUnits
    case {'mm'}
        squareSizeUnits = availableUnits{1};
    case {'cm'}
        squareSizeUnits = availableUnits{2};
    case {'in'}
        squareSizeUnits = availableUnits{3};
end
