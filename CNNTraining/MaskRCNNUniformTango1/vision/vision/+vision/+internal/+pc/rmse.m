%vision.internal.pc.rmse Compute rmse registration quality metric
%
%   r = rmse(moving, fixed) returns the root mean squared error of the
%   Euclidean distance between the moving point cloud and the fixed point
%   cloud. A lower root mean squared error is indicative of closer
%   alignment between the two point clouds.
%
%   r = rmse(moving, fixed, tform) returns the root mean squared error of
%   the Euclidean distance between the moving point cloud transformed by
%   tform and the fixed point cloud. A lower root mean squared error is
%   indicative of better registration.
%
%   r = rmse(movingPts, fixedPts, tform) returns the root mean squared
%   error of the Euclidean distance between the moving points transformed
%   by tform and the fixed points. A lower root mean squared error is
%   indicative of better registration.
%
%   See also pcregisterndt, pcregistericp, pcregistercorr, pcregistercpd,
%            pcregisterloam.

% Copyright 2020-2021 The MathWorks, Inc.
%#codegen
function r = rmse(moving, fixed, tform)

if nargin==2
    [~, dists] = multiQueryKNNSearchImpl(fixed, moving.Location, 1);
else
    if isa(moving, 'pointCloud')
        movingReg = pctransform(moving, tform);
        [~, dists] = multiQueryKNNSearchImpl(fixed, movingReg.Location, 1);
    else
        fixedPtCloud = pointCloud(fixed);
        fixedReg = pctransform(fixedPtCloud, invert(tform));
        [~, dists] = multiQueryKNNSearchImpl(fixedReg, moving, 1);
    end
end

r = sqrt(sum(dists,'all') / numel(dists));
end