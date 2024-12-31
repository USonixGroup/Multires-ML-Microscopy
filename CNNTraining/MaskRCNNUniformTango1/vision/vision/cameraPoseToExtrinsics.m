% cameraPoseToExtrinsics Convert camera pose to extrinsics
%   
%--------------------------------------------------------------------------
%   cameraPoseToExtrinsics is not recommended. Use pose2extr instead.
%--------------------------------------------------------------------------
%
%   tform = cameraPoseToExtrinsics(cameraPose) returns a rigid3d object
%   that represents the transformation from world coordinates to camera
%   coordinates. cameraPose is the orientation and location of the camera
%   in world coordinates, specified as a rigid3d object. The output is
%   computed as follows:
%
%   tform.Rotation = cameraPose.Rotation'
%   tform.Translation = -cameraPose.Translation * cameraPose.Rotation'
%
%   [rotationMatrix, translationVector] = cameraPoseToExtrinsics(orientation, location)
%   returns a 3-by-3 rotation matrix and a 1-by-3 translation vector from 
%   world coordinates into camera coordinates. orientation is a 3-by-3 
%   matrix and location is a 3-element vector representing the orientation 
%   and location of the camera in world coordinates. The outputs are 
%   computed as follows:
%   
%   rotationMatrix    = orientation'
%   translationVector = -location * orientation'
% 
%   Class Support
%   -------------
%   orientation and location must be of the same class, and can be double or single. 
%   rotationMatrix and translationMatrix are the same class as orientation and location.
% 
%   Example
%   -------
%   orientation = eye(3);
%   location = [0 0 10];
%
%   % Get the rotation matrix and translation vector to transform from 
%   % world coordinates to camera coordinates.
%   [rotationMatrix, translationVector] = cameraPoseToExtrinsics(orientation, location)
%
%   % Get the transformation from world coordinates to camera coordinates
%   % as a rigid3d object.
%   cameraPose = rigid3d(orientation, location);
%   tform = cameraPoseToExtrinsics(cameraPose)
% 
%   See also extr2pose, estimateExtrinsics, estrelpose, estworldpose.

% Copyright 2016-2022 MathWorks, Inc

%#codegen

function [out1, out2] = cameraPoseToExtrinsics(varargin)

narginchk(1, 2)

if nargin == 2
    % Validate orientation and location inputs.
    orientation = varargin{1};
    locationIn = varargin{2};
    validateInputs(orientation, locationIn, 'orientation', 'location');
    location = locationIn(:)';
    
    % Get rotation matrix and translation vector outputs.
    out1 = orientation';
    out2 = -location * out1;
else
    % Validate camera pose input.
    cameraPose = varargin{1};
    validateattributes(cameraPose, {'rigid3d', 'rigidtform3d'}, {'scalar'}, ...
        mfilename, 'cameraPose')
    
    nargoutchk(0, 1)
    
    % Get tform output.
    R = cameraPose.Rotation';
    t = -cameraPose.Translation * R;
    out1 = rigid3d(R, t);
end

%--------------------------------------------------------------------------
function validateInputs(R, t, varNameR, varNameT)
vision.internal.inputValidation.validateRotationMatrix(R, mfilename, ...
    varNameR);
vision.internal.inputValidation.validateTranslationVector(t, mfilename, ...
    varNameT);

coder.internal.errorIf(~isa(R, class(t)), 'vision:points:ptsClassMismatch',...
    varNameR, varNameT);



