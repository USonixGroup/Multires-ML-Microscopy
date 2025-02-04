function boundaryPixel = getInitialBoundaryPixel(undistortedMask)
% getInitialBoundaryPixel Shoot a ray from the center of the image straight
% down to find the initial boundary pixel

% Copyright 2023 The MathWorks, Inc.

%#codegen

    sRow = -1;
    sCol = -1;
    cx = floor(size(undistortedMask, 2) / 2);
    for i = floor(size(undistortedMask, 1)/2):size(undistortedMask, 1)
        if undistortedMask(i, cx) == 0
            sRow = i-1;
            sCol = cx;
            break;
        end
    end
    if sRow == -1
        sRow = size(undistortedMask, 1);
        sCol = cx;
    end
    boundaryPixel = [sRow, sCol];
end