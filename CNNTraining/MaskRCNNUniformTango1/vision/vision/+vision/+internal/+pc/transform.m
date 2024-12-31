%vision.internal.pc.transform Transform a point cloud using a 4-by-4
% transformation matrix

% Copyright 2020-2023 The MathWorks, Inc.

%#codegen

function ptCloudOut = transform(ptCloudIn, T)

R = T(1:3, 1:3);
t = T(4, 1:3);

% Apply forward transform to coordinates
if coder.target('MATLAB')
    if ismatrix(ptCloudIn.Location)
        loc = ptCloudIn.Location * R;
        loc(:,1) = loc(:,1) + t(1);
        loc(:,2) = loc(:,2) + t(2);
        loc(:,3) = loc(:,3) + t(3);
    else
        loc = reshape(ptCloudIn.Location,[],3) * R;
        loc(:,1) = loc(:,1) + t(1);
        loc(:,2) = loc(:,2) + t(2);
        loc(:,3) = loc(:,3) + t(3);
        loc = reshape(loc, size(ptCloudIn.Location));
    end
else
        count = coder.internal.indexInt(numel(ptCloudIn.Location)/3);
        loc = coder.nullcopy(ptCloudIn.Location);
        zOffset = coder.internal.indexInt(2*count);
        for ptIter = 1:count
            rotMatOut = [ptCloudIn.Location(ptIter), ptCloudIn.Location(ptIter + count),...
                ptCloudIn.Location(ptIter + zOffset)]*R;

            loc(ptIter) = rotMatOut(1) + t(1);
            loc(ptIter + count) = rotMatOut(2) + t(2);
            loc(ptIter + zOffset) = rotMatOut(3) + t(3);
        end
end

% Apply forward transform to normal vector
nv = cast([], 'like', loc);
if ~isempty(ptCloudIn.Normal)
    if vision.internal.isRigidTransform(T)
        if coder.target('MATLAB')
            if ismatrix(ptCloudIn.Normal)
                nv = ptCloudIn.Normal * R;
            else
                nv = reshape(reshape(ptCloudIn.Normal, [], 3) * R, size(ptCloudIn.Normal));
            end
        else
            count = coder.internal.indexInt(numel(ptCloudIn.Normal)/3);
            nv = coder.nullcopy(ptCloudIn.Normal);
            zOffset = coder.internal.indexInt(2*count);
            for ptIter = 1:count
                rotMatOut = [ptCloudIn.Normal(ptIter), ptCloudIn.Normal(ptIter + count),...
                    ptCloudIn.Normal(ptIter + zOffset)]*R;

                nv(ptIter) = rotMatOut(1) ;
                nv(ptIter + count) = rotMatOut(2) ;
                nv(ptIter + zOffset) = rotMatOut(3) ;
            end
        end
    else
        % Recompute normals for affine/shear transform
        nv = surfaceNormalImpl(pointCloud(loc), 6);
    end
end

ptCloudOut = pointCloud(loc, 'Color', ptCloudIn.Color, 'Normal', nv, ...
    'Intensity', ptCloudIn.Intensity);
end