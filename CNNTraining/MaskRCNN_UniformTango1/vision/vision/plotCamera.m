% plotCamera Plot a camera in 3-D coordinates.
%   cam = plotCamera() returns a camera visualization object rendered
%   in the current axes. 
% 
%   cam = plotCamera(cameraTable) returns an array of camera visualization
%   objects rendered in the current axes. cameraTable is a table containing
%   properties of the camera visualization objects. The columns of the
%   table can be any of the camera visualization object properties
%   described below, except for 'Parent'. Additionally, if the table
%   contains a 'ViewId' table, then the view ids will be used as camera
%   labels.
%
%   cam = plotCamera(..., Name, Value) creates a camera visualization
%   object with the property values specified in the argument list.
%   The following properties are supported:
%
%   'AbsolutePose'   A scalar rigidtform3d object, specifying the absolute
%                    pose of the camera in the world coordinate system
%                    specified as a rigidtform3d object.
%
%                    Default: rigidtform3d()
%
%   'Size'           The width of camera's base specified as a scalar.
%
%                    Default: 1
%
%   'Label'          Camera label specified as a string.
%
%                    Default: ''
%
%   'Color'          The color of the camera specified as a string or a 
%                    3-element vector of RGB values with the range of [0, 1].
%
%                    Default: [1, 0, 0]   
%
%   'Opacity'        A scalar in the range of [0, 1] specifying the opacity
%                    of the camera.
%
%                    Default: 0.2
%
%   'Visible'        A logical scalar, specifying whether the camera is visible.
% 
%                    Default: true
%
%   'AxesVisible'    A logical scalar, specifying whether to display
%                    camera's axes.
%
%                    Default: false
%
%   'ButtonDownFcn'  Callback function that executes when you click the camera.
%
%                    Default: ''
%
%   'Parent'         Specify an output axes for displaying the visualization.
%
%                    Default: gca
%
%   Example - Create an Animated Camera
%   -----------------------------------
%   % Plot a camera pointing along the Y-axis
%   origRot = [1     0    0;
%              0     0    1;
%              0    -1    0];
%   t = [10 0 20];
%   pose = rigidtform3d(origRot, t);
% 
%   % Setting opacity of the camera to zero for faster animation.
%   cam = plotCamera('AbsolutePose', pose, 'Opacity', 0);
% 
%   % Set view properties
%   grid on
%   axis equal
%   axis manual
%
%   % Make the space large enough for the animation.
%   xlim([-15, 20]);
%   ylim([-15, 20]);
%   zlim([15, 25]);
% 
%   % Make the camera fly in a circle
%   for theta = 0:3:1800
%       % Rotation about camera's y-axis
%       updateTform = rigidtform3d([0 theta 0], [10 * cosd(theta), 10 * sind(theta), 20]);
%       cam.AbsolutePose = rigidtform3d(origRot * updateTform.R', updateTform.Translation);
%       drawnow();
%   end
%
%   See also showExtrinsics, estworldpose, estrelpose, estimateExtrinsics,
%            extr2pose, rigidtform3d.

%   Copyright 2014-2022 The MathWorks, Inc.

function cam = plotCamera(varargin)

if nargin > 0
    [varargin{:}] = convertStringsToChars(varargin{:});
end

params = parseInputs(varargin{:});

% Record the current 'hold' state so that we can restore it later.
holdState = get(params(1).Parent,'NextPlot');

for i = 1:numel(params)
    hCamera(i) = vision.graphics.Camera.plotCameraImpl(...
        double(params(i).Size), params(i).AbsolutePose, params(i).Parent); %#ok<AGROW>
        
    if isfield(params, 'ViewId')
        hCamera(i).Label = num2str(params(i).ViewId); %#ok<AGROW>
    else
        hCamera(i).Label         = params(i).Label; %#ok<AGROW>
    end
    
    hCamera(i).Visible       = params(i).Visible; %#ok<AGROW>
    hCamera(i).Color         = params(i).Color; %#ok<AGROW>
    hCamera(i).Opacity       = params(i).Opacity; %#ok<AGROW>
    hCamera(i).AxesVisible   = params(i).AxesVisible; %#ok<AGROW>
    hCamera(i).ButtonDownFcn = params(i).ButtonDownFcn; %#ok<AGROW>
    hold(hCamera(i).Parent, 'on');
end

% Restore the hold state.
set(params(1).Parent, 'NextPlot', holdState);

if nargout > 0
    cam = hCamera;
end

%--------------------------------------------------------------------------
function params = parseInputs(varargin)
import vision.graphics.*;

parser = inputParser;
parser.addParameter('AbsolutePose', rigidtform3d, @Camera.checkAbsolutePose);
% Specifiying Location and Orientation is allowed for backward compatibility
% but will not be documented. 
parser.addParameter('Location', [0,0,0], @Camera.checkLocation);
parser.addParameter('Orientation', eye(3), @Camera.checkOrientation);
parser.addParameter('Size', 1, @Camera.checkCameraSize);
parser.addParameter('Color', [1 0 0], @Camera.checkColor);
parser.addParameter('Label', '', @Camera.checkLabel);
parser.addParameter('Visible', true, @Camera.checkVisible);
parser.addParameter('AxesVisible', false, @Camera.checkAxesVisible);
parser.addParameter('Opacity', 0.2, @Camera.checkOpacity);
parser.addParameter('ButtonDownFcn', '', @Camera.checkCallback);

parser.addParameter('Parent', [], @checkParent);

if ~isempty(varargin) && istable(varargin{1})
    camTable = varargin{1};
    checkCameraTable(camTable);
    
    parser.parse(varargin{2:end});
            
    params = table2struct(camTable);
    
    singleParams = parser.Results;
    if isempty(singleParams.Parent)
        singleParams.Parent = gca();
    end

    singleParams = updateAbsolutePose(singleParams, parser.UsingDefaults);

    singleParamNames = fields(parser.Results);
    for i = 1:numel(singleParamNames)
        if ~isfield(params, singleParamNames{i})
            [params.(singleParamNames{i})] = deal(...
                singleParams.(singleParamNames{i}));
        end
    end    
else
    parser.parse(varargin{:});
    params = parser.Results;
    % Set parent to gca if it is not specified.
    % This must be done after parsing the parameters. Otherwise, a new axes may
    % be created even if parameter validation fails.
    if isempty(params.Parent)
        params.Parent = gca();
    end
    
    params = updateAbsolutePose(params, parser.UsingDefaults);
end

%--------------------------------------------------------------------------
function params = updateAbsolutePose(params, usingDefaultValues)
if ~ismember('Location', usingDefaultValues)
    params.AbsolutePose.Translation =  double(params.Location(:).'); 
end

if ~ismember('Orientation', usingDefaultValues)
    % Force location to be a row vector
    params.Orientation = double(params.Orientation);

    params.AbsolutePose.Rotation =  params.Orientation; 
end
%--------------------------------------------------------------------------
function tf = checkParent(parent)
tf = vision.internal.inputValidation.validateAxesHandle(parent);

%--------------------------------------------------------------------------
function checkCameraTable(camTable)
import vision.graphics.*;
validator = vision.internal.inputValidation.TableValidator;
validator.CanBeEmpty = false;
validator.OptionalVariableNames = {'ViewId', 'AbsolutePose', 'Location', 'Orientation', ...
    'Size', 'Label', 'Color', 'Opacity', 'Visible', 'AxesVisible', ...
    'ButtonDownFcn'};

validator.ValidationFunctions('AbsolutePose') = @Camera.checkAbsolutePose;
validator.ValidationFunctions('Location') = @Camera.checkLocation;
validator.ValidationFunctions('Orientation') = @Camera.checkOrientation;
validator.ValidationFunctions('Size') = @Camera.checkCameraSize;
validator.ValidationFunctions('Label') = @Camera.checkLabel;
validator.ValidationFunctions('Visible') = @Camera.checkVisible;
validator.ValidationFunctions('Opacity') = @Camera.checkOpacity;
validator.ValidationFunctions('ButtonDownFcn') = @Camera.checkCallback;
validator.ValidationFunctions('ViewId') = ...
    @(id)validateattributes(id, {'numeric'}, ...
    {'integer', 'scalar', 'nonnegative'}, ...
    'plotCamera', 'ViewId');

validator.validate(camTable, 'plotCamera', 'cameraTable');