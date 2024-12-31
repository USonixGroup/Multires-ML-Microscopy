function [bboxB,indices] = bboxwarp(bboxA,tform,ref,varargin)
%BBOXWARP Apply geometric transformation to bounding boxes.
%   bboxB = BBOXWARP(bboxA,tform,ref) transforms axis-aligned rectangles,
%   rotated rectangles, and cuboids in bboxA according to the geometric
%   transformation defined by tform. tform is an affinetform2d or
%   affinetform3d object. bboxA is an M1-by-N matrix defining M1 bounding
%   boxes and N is either 4, 5, or 9. N=4 is associated with axis-aligned
%   rectangle bounding boxes, N=5 is associated with rotated rectangle
%   bounding boxes, and N=9 is associated with cuboid bounding boxes. ref
%   is a spatial referencing object that defines the output view into which
%   the boxes are transformed. The transformed bounding boxes are returned
%   in bboxB, an M2-by-N matrix defining M2 bounding boxes, where M2 <= M1.
%
%   This function supports 2-D and 3-D bounding boxes which includes
%   axis-aligned rectangle, rotated rectangle and cuboid.
%
%   [...,indices] = BBOXWARP(bboxA,tform,ref) also returns a vector of
%   indices to indicate which boxes in bboxA correspond to the warped
%   versions in bboxB.
%
%   [...] = BBOXWARP(...,Name,Value) specifies additional name-value pair
%   arguments described below:
%
%   'OverlapThreshold'   Specify a positive overlap threshold that is less
%                        than or equal to 1. The amount of overlap between
%                        transformed boxes and the area defined by the
%                        output view is defined as area(bbox intersect W) /
%                        area(bbox), where bbox is the result of
%                        transforming boxes in bboxA and W is the bounding
%                        rectangle defined by the input spatial reference
%                        object, ref. If the overlap is above the
%                        threshold, the transformed boxes are clipped to
%                        the bounding rectangle border. Otherwise, they are
%                        discarded. Note that lowering the threshold may
%                        discard parts of objects.
%
%                        Default: 1
%
%   Class Support
%   -------------
%   bboxA can be any nonsparse numeric type. The class of bboxB is the same
%   as the class of bboxA when bboxA contains floating point data.
%   Otherwise, the class of bboxB is single. For rectangular bboxA inputs,
%   tform must be an affinetform2d object and ref must be an imref2d
%   object. For cuboid bboxA inputs, tform must be an affinetform3d
%   object and ref must be an imref3d object.
%
%   Example: Transform images and associated bounding boxes.
%   --------------------------------------------------------
%   % Read image.
%   I = imread('peppers.png');
%
%   % Define bounding boxes and labels.
%   bboxA = [
%       410 230 100 90
%       186 78  80  60
%       ];
%
%   labelsA = [
%        "garlic"
%        "onion"
%      ];
%
%   % Define transform to horizontally flip and translate image.
%   tform = affinetform2d([-1 0 50; 0 1 50; 0 0 1]);
%
%   % Create an output view for imwarp.
%   rout = affineOutputView(size(I),tform);
%
%   % Warp image.
%   J = imwarp(I,tform,'OutputView',rout);
%
%   % Warp the boxes.
%   [bboxB,indices] = bboxwarp(bboxA,tform,rout);
%   labelsB = labelsA(indices);
%
%   % Display the results.
%   annotatedI = insertObjectAnnotation(I,'Rectangle',bboxA,labelsA);
%   annotatedJ = insertObjectAnnotation(J,'Rectangle',bboxB,labelsB);
%   figure
%   montage({annotatedI, annotatedJ})
%
%   See also bboxcrop, bboxresize, imwarp, imref2d, affinetform2d,
%            affinetform3d, affineOutputView.

% Copyright 2019-2023 The MathWorks, Inc.

params = iParseInputs(bboxA,tform,ref,varargin{:});

bboxA = iManageEmptyInputs(bboxA);

% Cast inputs to floating point for processing.
[bboxA, tform] = iCastToFloat(bboxA,tform);

axisAligned2D = size(bboxA,2) == 4;
rotated2D = size(bboxA,2) == 5;
cuboid = size(bboxA,2) == 9;

if axisAligned2D
    [bboxB,indices] = iWarp2DAxisAlginedCase(bboxA,tform,ref,params);
elseif rotated2D
    if params.OverlapThreshold < 1
        error(message('vision:bbox:rotatedBoxOverlapThreshold'));
    end
    [bboxB,indices] = iWarpRotated2D(bboxA,tform,ref);
elseif cuboid
    if params.OverlapThreshold < 1
        error(message('vision:bbox:rotatedBoxOverlapThreshold'));
    end
    [bboxB,indices] = iWarpRotated3D(bboxA,tform,ref);
else
    assert(false,'Unexpected bbox format');
end

end


function [bboxB,indices] = iWarpRotated2D(bboxA,tform,ref)
    [S,R,~,~] = iGetSRTTransformation(tform);

    % Apply scale transformation
    bboxB = bboxA;
    bboxB(:,3) = S(1) * bboxA(:,3);
    bboxB(:,4) = S(2) * bboxA(:,4);

    % Rotation angle is additive offset in this representation
    bboxB(:,5) = bboxB(:,5) + R;

    % Position of X and Y center moves as a result of overall
    % transformation
    bboxB(:,[1 2]) = tform.transformPointsForward(bboxB(:,[1 2]));
    % Move to polygon representation of box
    [X,Y] = vision.internal.bbox.bbox2poly(bboxB);

    % Valid rotated boxes are boxes where all of the vertices are
    % completely in bounds.
    valid = all(ref.contains(X,Y),1);
    indices = reshape(find(valid),[],1);
    bboxB = bboxB(indices,:);
   
    % Convert from world to intrinsic coordinate system.
    [bboxB(:,1), bboxB(:,2)] = ref.worldToIntrinsicAlgo(bboxB(:,1), bboxB(:,2));
end

function [bboxB,indices] = iWarpRotated3D(bboxA,tform,ref)
    [S,eulZYX,~,~] = iGetSRTTransformation3d(tform);

    % Boxes are in order: [X,Y,Z,Width,Height,Depth,thetaX,thetaY,thetaZ]
    % Where theta are Euler angles in Z,Y,X order.

    % Apply scale transformation
    bboxB = bboxA;
    bboxB(:,4) = S(1) * bboxA(:,4);
    bboxB(:,5) = S(2) * bboxA(:,5);
    bboxB(:,6) = S(3) * bboxA(:,6);

    % Apply rotation transformation

    % Rotation angles are additive offset in this representation
    bboxB(:,7) = bboxB(:,7) + eulZYX(3);
    bboxB(:,8) = bboxB(:,8) + eulZYX(2);
    bboxB(:,9) = bboxB(:,9) + eulZYX(1);

    % Apply overall transformation to center point
    bboxB(:,[1 2 3]) = tform.transformPointsForward(bboxB(:,[1 2 3]));

    % Move to polygon representation of cuboids
    [X,Y,Z] = vision.internal.bbox.bbox2poly(bboxB);

    % Valid rotated boxes are boxes where all of the vertices are
    % completely in bounds.
    valid = all(ref.contains(X,Y,Z),1);
    indices = reshape(find(valid),[],1);
    bboxB = bboxB(indices,:);

end


function [S,R,T,Mrot] = iGetSRTTransformation(tform)

D = tform.Dimensionality;
T = tform.T(3,1:D);
M = tform.T(1:D,1:D);

Sx = hypot(M(1,1),M(1,2));
Sy = hypot(M(2,1),M(2,2));

Mrot(1,:) = M(1,:) ./ Sx;
Mrot(2,:) = M(2,:) ./ Sy;

R = atan2d(Mrot(1,2),Mrot(2,2));

S = [Sx,Sy];

containsReflection = det(Mrot) < 0;

MrotRebuilt = [cosd(R), sind(R);...
               -sind(R), cosd(R)];

temp = [Sx 0; 0 Sy] * MrotRebuilt;
Mrebuilt = eye(D+1,'like',T);
Mrebuilt(1:D,1:D) = temp;
Mrebuilt(D+1,1:D) = T;

% Compare the rebuilt transform from the primitive transformations against
% what was actually put in. If different, there is shear.
containsShear = any(imabsdiff(Mrebuilt,tform.T) > 10*eps(class(Mrebuilt)),[1 2]);

% We don't support reflection or shear with rotated 2D boxes.
if containsReflection || containsShear
    error(message('vision:bbox:unsupportedTransformForRotatedBox'));
end

end


function [S,eulZYX,T,Mrot] = iGetSRTTransformation3d(tform)
% N.B. Euler angles are returned in [rz,ry,rx] order.

D = tform.Dimensionality;
T = tform.T(D+1,1:D);
M = tform.T(1:D,1:D);

Sx = norm(M(1,:));
Sy = norm(M(2,:));
Sz = norm(M(3,:));

Mrot(1,:) = M(1,:) ./ Sx;
Mrot(2,:) = M(2,:) ./ Sy;
Mrot(3,:) = M(3,:) ./ Sz;

% Tranpose matrix into pre-multiply form for conversion to Euler angles.
Mrot = Mrot';

% Check if the matrix is singular
sy = sqrt(Mrot(1,1).*Mrot(1,1) + Mrot(2,1).*Mrot(2,1));
singular = sy < 10 * eps(class(Mrot));

% Calculate Euler angles in radians. This computation is adapted from
% rotm2eul from the Robotics System Toolbox
eul = [atan2(Mrot(3,2), Mrot(3,3)), atan2(-Mrot(3,1), sy), atan2(Mrot(2,1), Mrot(1,1))];

% Singular matrices need special treatment
if singular
    eul = [atan2(-Mrot(2,3), Mrot(2,2)), ...
        atan2(-Mrot(3,1), sy), zeros(1,1,'like',T)];
end

S = [Sx,Sy,Sz];

containsReflection = det(Mrot) < 0;

% Reorder euler angles into ZYX order.
eulZYX(:,1:3) = eul(:,3:-1:1);

% Convert Euler angles (in radians) to rotation matrix (pre-multiply form).
MrotRebuilt = vision.internal.eul.eulerAngleToRotationMatrix(eulZYX,'ZYX');

% Transform rotation matrix into post-multiply form.
MrotRebuilt = MrotRebuilt';

% Return Euler angles in degrees.
eulZYX = rad2deg(eulZYX);

% Rebuild transform.
temp = [Sx 0 0; 0 Sy 0; 0 0 Sz] * MrotRebuilt;
Mrebuilt = eye(D+1,'like',T);
Mrebuilt(1:D,1:D) = temp;
Mrebuilt(D+1,1:D) = T;

% Compare the rebuilt transform from the primitive transformations against
% what was actually put in. If different, there is shear.
containsShear = any(imabsdiff(Mrebuilt,tform.T) > 10*eps(class(Mrebuilt)),[1 2]);

% We don't support reflection or shear with cuboid boxes.
if containsReflection || containsShear
    error(message('vision:bbox:unsupportedTransformForCuboidBox'));
end

end

function [bboxB,indices] = iWarp2DAxisAlginedCase(bboxA,tform,ref,params)

% Convert box from [x y w h] to [xmin ymin xmax ymax].
bboxCorners = vision.internal.bbox.xywh2minmax(bboxA);

% Convert axis-aligned box to box vertices.
[X,Y] = vision.internal.bbox.bbox2poly(bboxCorners);

% Warp box points.
[X,Y] = tform.transformPointsForward(X,Y);

% Convert from world to intrinsic coordinate system.
[xi,yi] = ref.worldToIntrinsicAlgo(X,Y);

% Convert to axis-aligned bounding box.
bboxCorners = vision.internal.bbox.poly2bbox(xi,yi);

% Create a cropping window from the output view.
win = iCroppingWindowFromOutputView(ref);

% Crop boxes to window.
[bboxB, valid] = vision.internal.bbox.crop(bboxCorners,win,...
    params.OverlapThreshold);

% Convert minmax to xywh
bboxB = [bboxB(:,1:2) bboxB(:,3:4) - bboxB(:,[1 2])];

% Return indices bounding boxes from bboxA that are retained in bboxB.
indices = reshape(find(valid),[],1);

end

%--------------------------------------------------------------------------
function params = iParseInputs(bboxA,tform,ref,varargin)
p = inputParser;
p.addRequired('bboxA',@(x)vision.internal.inputValidation.checkBBox(x,mfilename,'bboxA',1));
p.addRequired('tform',@iCheckTransform);
p.addRequired('ref',@iCheckSpatialReference);
p.addParameter('OverlapThreshold',1,@iCheckOverlapThreshold);

p.parse(bboxA,tform,ref,varargin{:})

userInput = p.Results;

params.OverlapThreshold = double(userInput.OverlapThreshold);

iCheckDimensionalityOfInputsMatches(bboxA,tform,ref);

end

%--------------------------------------------------------------------------
function iCheckDimensionalityOfInputsMatches(bbox,tform,ref)

if size(bbox,2) == 0
    % For empty where we can't tell what the intended dimensionality is
    % from the bbox data, we still don't allow cases where spatial referencing
    % and tform don't agree.
    if ~(((isa(tform,'affinetform2d') || isa(tform,'affine2d')) && isa(ref,'imref2d')) || ...
         ((isa(tform,'affinetform3d') || isa(tform,'affine3d')) && isa(ref,'imref3d')))
        error(message('vision:bbox:dimensionalityMismatchInInputs'))
    end
elseif (size(bbox,2) == 4 || size(bbox,2) == 5)
    if ~((isa(tform,'affinetform2d') || isa(tform,'affine2d')) && isa(ref,'imref2d'))
        error(message('vision:bbox:dimensionalityMismatchInInputs'))
    end
elseif size(bbox,2) == 9
    if ~((isa(tform,'affinetform3d') || isa(tform,'affine3d')) && isa(ref,'imref3d'))
        error(message('vision:bbox:dimensionalityMismatchInInputs'))
    end
else
    assert(false,'Unexpected bbox dimensionality');
end

end

%--------------------------------------------------------------------------
function iCheckSpatialReference(ref)
validateattributes(ref,{'imref2d','imref3d'},{'scalar'},mfilename,'ref',3);
end

%--------------------------------------------------------------------------
function iCheckTransform(tform)
validateattributes(tform,{'affinetform2d','affinetform3d','affine2d','affine3d'},...
    {'scalar'},mfilename,'tform',2);
end

%--------------------------------------------------------------------------
function iCheckOverlapThreshold(x)

validateattributes(x,{'numeric'},{'scalar','positive','<=',1,'real','finite','nonsparse'},...
    mfilename,'OverlapThreshold');

% Numeric objects are not supported.
vision.internal.inputValidation.validateNotObject(x,'vision','OverlapThreshold');

end

%--------------------------------------------------------------------------
function win = iCroppingWindowFromOutputView(ref)
% Use the output view intrinsic limits to define the cropping window. The
% format of the output win is [xmin ymin xmax ymax].
win = [
    ref.XIntrinsicLimits(1) ...
    ref.YIntrinsicLimits(1) ...
    ref.XIntrinsicLimits(2) ...
    ref.YIntrinsicLimits(2)];
end

%--------------------------------------------------------------------------
function [x, tform] = iCastToFloat(x, tform)
if ~isfloat(x)
    % Cast non-floats to single for processing.
    x = single(x);
end

% Ensure x and tform have the same type.
tform.T = cast(tform.T, 'like', x);
end

%--------------------------------------------------------------------------
function X = iManageEmptyInputs(X)
if isempty(X)
    if size(X,2) == 0
        % If given 0x0 empty, we can't know the intended dimensionality of
        % boxes.
        X = reshape(X,0,4);
    end
end
end

