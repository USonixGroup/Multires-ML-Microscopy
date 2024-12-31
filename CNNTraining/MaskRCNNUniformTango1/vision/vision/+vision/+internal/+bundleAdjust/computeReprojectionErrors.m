function reprojectionErrors = computeReprojectionErrors(visibility, curMeanErr, returnPointType)
%
%  Copyright 2022 The MathWorks, Inc.
%#codegen
reprojectionErrors = zeros(size(visibility, 1), 1, returnPointType);
nViews = sum(full(visibility), 2); % Sum of rows, full matrix is faster
endIndex   = cumsum(nViews);
startIndex = [1; endIndex(1:end-1)+1];
curMeanErrSqrt = sqrt(curMeanErr);
for n = 1:numel(nViews)
    reprojectionErrors(n) = sum(curMeanErrSqrt(startIndex(n): endIndex(n)))/nViews(n);
end
end