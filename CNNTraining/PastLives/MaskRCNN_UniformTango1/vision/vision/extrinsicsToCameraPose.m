% extrinsicsToCameraPose Convert extrinsics into camera pose
%   
%--------------------------------------------------------------------------
%   extrinsicsToCameraPose is not recommended. Use extr2pose instead.
%--------------------------------------------------------------------------
%
%   cameraPose = extrinsicsToCameraPose(tform) returns a rigid3d object
%   that represents the camera pose in world coordinates. tform is the
%   transformation from world coordinates to camera coordinates, specified
%   as a rigid3d object. The output is computed as follows:
%
%   cameraPose.Rotation = tform.Rotation'
%   cameraPose.Translation = -tform.Translation * tform.Rotation'
%
%   [orientation, location] = extrinsicsToCameraPose(rotationMatrix, translationVector)
%   returns a 3-by-3 camera orientation matrix and a 1-by-3 location vector 
%   in the world coordinates. rotationMatrix is a 3-by-3 matrix and 
%   translationVector is a 3-element vector representing rotation and 
%   translation from world coordinates into camera coordinates. The outputs
%   are computed as follows:
% 
%   orientation = rotationMatrix'
%   location    = -translationVector * rotationMatrix'
% 
%   Class Support
%   -------------
%   rotationMatrix and translationMatrix must be of the same class, and can be
%   double or single. orientation and location are the same class as 
%   rotationMatrix and translationMatrix.
% 
%   Example
%   -------
%   rotationMatrix = eye(3);
%   translationVector = [0 0 -10];
%
%   % Get the camera orientation matrix and location vector in world 
%   % coordinates.
%   [orientation, location] = extrinsicsToCameraPose(rotationMatrix, translationVector)
%
%   % Get the camera pose in world coordinates as a rigid3d object.
%   tform = rigid3d(rotationMatrix, translationVector);
%   cameraPose = extrinsicsToCameraPose(tform)
% 
%   See also pose2extr, estimateExtrinsics, estrelpose, estworldpose, plotCamera.

% Copyright 2016-2022 MathWorks, Inc

%#codegen

function [out1, out2] = extrinsicsToCameraPose(varargin)

narginchk(1, 2)

if nargin == 2
    % Validate rotationMatrix and translationVector inputs.
    R = varargin{1};
    tIn = varargin{2};
    validateInputs(R, tIn, 'rotationMatrix', 'translationVector');
    
    % Get orientation and location outputs.
    t = tIn(:)';
    out1 = R';
    out2 = -t*out1;
else
    % Validate tform input.
    tform = varargin{1};
    validateattributes(tform, {'rigid3d', 'rigidtform3d'}, {'scalar'}, ...
        mfilename, 'tform');
    
    nargoutchk(0, 1)
    
    % Get cameraPose output.
    R = tform.Rotation';
    t = -tform.Translation * tform.Rotation';
    out1 = rigid3d(R, t);
end

%--------------------------------------------------------------------------
function validateInputs(R, t, varNameR, varNameT)
vision.internal.inputValidation.validateRotationMatrix(R, mfilename, ...
    varNameR);
vision.internal.inputValidation.validateTranslationVector(t, mfilename, ...
    varNameT);

coder.internal.errorIf(~isa(R, class(t)), 'vision:points:ptsClassMismatch', ...
    varNameR, varNameT);

