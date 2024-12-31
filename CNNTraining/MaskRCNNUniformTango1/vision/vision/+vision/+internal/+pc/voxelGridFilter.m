%vision.internal.pc.voxelGridFilter apply voxel grid filter to points
%
%   [means, invars, vars] = voxelGridFilter(xyzPoints, voxelSize, minVoxelPoints, eigRatio)
%   applies a voxel grid filter of size voxelSize to the points in
%   xyzPoints. Voxels are discarded if they have fewer than minVoxelPoints
%   points. Variance of voxels whose largest eigen value is more than
%   eigRatio times the others is inflated.
%
%   [...] = voxelGridFilter(..., spatialLimits) additionally specifies
%   spatialLimits as [xmin, xmax, ymin, ymax, zmin, zmax]. Voxels outside
%   this are discarded.
%
%   [..., counts] = voxelGridFilter(...) additionally returns counts, the
%   number of points in each voxel.
%
%   Notes
%   -----
%   - The eigRatio is used to inflate the variance matrix when it is found
%     to be nearly singular.

%#codegen

% Copyright 2017-2024 The MathWorks, Inc.

function [voxelMeansOut, voxelICovsOut, voxelCovsOut, varargout] = voxelGridFilter(xyzPoints, voxelSize, minVoxelPoints, eigenValueRatio, varargin)

if nargin==4
    rangeLimits = [...
        min(xyzPoints(:,1)),max(xyzPoints(:,1)), ...
        min(xyzPoints(:,2)),max(xyzPoints(:,2)),...
        min(xyzPoints(:,3)),max(xyzPoints(:,3))];
else
    rangeLimits = varargin{1};
end

needCounts = nargout>3;
isSim      = isempty(coder.target);

if isSim
    [voxelMeans, ~, ~, ~, ~, voxelCovs, varargout{1:double(needCounts)}] = visionVoxelGridFilter(...
        xyzPoints, [], [], [], [], voxelSize, rangeLimits, minVoxelPoints);
elseif coder.internal.preferMATLABHostCompiledLibraries()
    [voxelMeans, ~, ~, ~, ~, voxelCovs, varargout{1:double(needCounts)}] = ...
        vision.internal.buildable.voxelGridFilterBuildable.voxelGridFilter(...
        xyzPoints, [], [], [], [], voxelSize, rangeLimits, minVoxelPoints);
else
    [voxelMeans, ~, ~, ~, ~, voxelCovs, varargout{1:double(needCounts)}] = vision.internal.codegen.pc.voxelGridFilter(...
        xyzPoints, [], [], [], [], voxelSize, rangeLimits, minVoxelPoints);
end

voxelICovs = nan(size(voxelCovs), 'like', voxelCovs);

if isempty(voxelCovs)
    voxelMeansOut = voxelMeans;
    voxelICovsOut = voxelICovs;
    voxelCovsOut  = voxelCovs;
    return;
end

% Ignore voxels with non-finite variances
isInvalidVoxel = squeeze(any(~isfinite(voxelCovs), 1:2));

numVoxels =  coder.internal.indexInt(size(voxelMeans, 1));
for n = 1 : numVoxels

    if isInvalidVoxel(n)
        continue;
    end

    % Eigenvectors corresponding to distinct eigenvalues of a symmetric
    % matrix must be orthogonal to each other.
    vcov = voxelCovs(:, :, n);
    [V, D] = eig(vcov,'vector');

    maxEv = max(D);
    flat = (maxEv >= D * eigenValueRatio);

    if any(flat)
        D(flat) = maxEv / eigenValueRatio;

        J = V * V' - eye(3);
        if max(abs(J(:))) < 10*eps(class(J))
            iV = V';
        else
            iV = inv(V);
        end

        voxelCovs(:, :, n)  = real(V * diag(D   ) * iV);
        voxelICovs(:, :, n) = real(V * diag(1./D) * iV);
    else
        voxelICovs(:,:,n) = inv(vcov);
    end
end

% g2800694
numInvalidVoxels = coder.internal.indexInt(nnz(isInvalidVoxel));

if numInvalidVoxels ~= 0
    outSizeVoxelMean = numVoxels - numInvalidVoxels;
    outSizevoxelCovs = coder.internal.indexInt([size(voxelCovs,1), size(voxelCovs,2), outSizeVoxelMean]);
    voxelMeansOut = coder.nullcopy(zeros(outSizeVoxelMean, size(voxelMeans,2),'like', voxelMeans));
    voxelICovsOut = coder.nullcopy(zeros(outSizevoxelCovs,'like', voxelICovs));
    voxelCovsOut  = coder.nullcopy(zeros(outSizevoxelCovs,'like', voxelCovs));

    count = 0;
    for i = 1:numVoxels
        if ~isInvalidVoxel(i)
            count = count + 1;
            voxelMeansOut(count,:) = voxelMeans(i,:);
            voxelICovsOut(:, :, count)= voxelICovs(:, :, i);
            voxelCovsOut(:, :, count)= voxelCovs(:, :, i);
        end
    end
else
    voxelMeansOut = voxelMeans;
    voxelICovsOut = voxelICovs;
    voxelCovsOut  = voxelCovs;
end

if needCounts
    varargout{1}(isInvalidVoxel, :) = [];
end
end
