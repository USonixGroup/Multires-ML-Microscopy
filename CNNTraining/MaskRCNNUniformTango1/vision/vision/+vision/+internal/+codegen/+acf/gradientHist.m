function grHist = gradientHist(gMag, gDir, params) %#codegen
% Compute oriented gradient histograms.

% Copyright 2020 The MathWorks, Inc.
%
% References
% ----------
%   Dollar, Piotr, et al. "Fast feature pyramids for object detection."
%   Pattern Analysis and Machine Intelligence, IEEE Transactions on 36.8
%   (2014): 1532-1545.

    cellSize    = params.CellSize;
    numBins     = params.NumBins;
    interpolate = params.Interpolation;
    full        = params.FullOrientation;

    % Check inputs
    if (nargin < 2 || isempty(gMag))
        grHist = gMag;
        return;
    end

    [m, n] = size(gMag);
    m = single(m);
    n = single(n);

    % Shape input to desired size: m and n must be divisible to CellSize
    cr = single(mod([m n],cellSize));
    if (any(cr))
        m   = m - cr(1);
        n   = n - cr(2);
        gMagCrop = gMag(1:m,1:n,:);
        gDirCrop = gDir(1:m,1:n,:);
    else
        gMagCrop = gMag;
        gDirCrop = gDir;
    end

    grHist = visionACFGradHistCG(gMagCrop, gDirCrop, cellSize, numBins, interpolate, full);
end

function gradHist = visionACFGradHistCG(gMag, gDir, cellSize, numBins, interpolate, useSignedOrientation)
    validatestring(interpolate,{'None','Orientation','Spatial','Both'},mfilename);

    h = size(gMag,1);
    w = size(gMag,2);
    numberOfRowCells = h/cellSize;
    numberOfColumnCells = w/cellSize;

    switch(interpolate)
        case 'None'
            gradHist = noInterpolation(gMag, gDir, cellSize, h, w, numberOfRowCells, numberOfColumnCells, numBins, useSignedOrientation);
        case 'Orientation'
            gradHist = orientationInterpolation(gMag, gDir, cellSize, h, w, numberOfRowCells, numberOfColumnCells, numBins, useSignedOrientation);
        case 'Spatial'
            gradHist = spatialInterpolation(gMag, gDir, cellSize, h, w, numberOfRowCells, numberOfColumnCells, numBins, useSignedOrientation);
            gradHist = normalizeBoundaries(gradHist, numberOfRowCells, numberOfColumnCells, numBins);
        case 'Both'
            gradHist = spatialOrientationInterpolation(gMag, gDir, cellSize, h, w, numberOfRowCells, numberOfColumnCells, numBins, useSignedOrientation);
            gradHist = normalizeBoundaries(gradHist, numberOfRowCells, numberOfColumnCells, numBins);
    end
end

function gradHist = noInterpolation(gMag, gDir, cellSize, h, w, numberOfRowCells, numberOfColumnCells, numBins, useSignedOrientation)
    invInvCellSize = 1 / (cellSize*cellSize);

    if (useSignedOrientation ~= 0)
        invBinWidth = numBins / (2*3.14159265);
    else
        invBinWidth = numBins / (3.14159265);
    end

    % initialize containers
    gradHist = zeros([numberOfRowCells numberOfColumnCells numBins],'single');
    iO = zeros([1,h],'int32');
    iM = zeros([1,h]);

    % process column-by-column
    for c = 1:w
        % process row
        for r = 1:h
            % compute the required offset to the indexed orientation channel
            binInd = floor(gDir(r,c) * invBinWidth);
            % wrap
            if binInd >= numBins
                binInd = single(0);
            end
            % store normalized magnitude - corresponds to normalizing the cell values by the sum of the cell values
            iO(r) = binInd + 1;
            iM(r) = gMag(r,c) * invInvCellSize;
        end

        col = floor((c-1)/cellSize)+1;
        % process row
        for r = 1:h
            row = floor((r-1)/cellSize)+1;
            gradHist(row,col,iO(r)) = gradHist(row,col,iO(r)) + iM(r);
        end
    end
end

function gradHist = orientationInterpolation(gMag, gDir, cellSize, h, w, numberOfRowCells, numberOfColumnCells, numBins, useSignedOrientation)
    invInvCellSize = 1 / (cellSize*cellSize);

    if (useSignedOrientation ~= 0)
        invBinWidth = numBins / (2*3.14159265);
    else
        invBinWidth = numBins / (3.14159265);
    end

    gradHist = zeros([numberOfRowCells numberOfColumnCells numBins],'single');

    % process column-by-column
    for c = 1:w
        % copy the appropriate column
        colgDir = gDir(:,c);
        colgMag = gMag(:,c);

        [iO,iO2,iM,iM2] = softBinOrientations(colgDir,colgMag,h,numBins,invBinWidth,invInvCellSize);

        col = floor((c-1)/cellSize)+1;
        % process rows
        for r = 1:h
            row = floor((r-1)/cellSize)+1;
            gradHist(row,col,iO(r)) = gradHist(row,col,iO(r)) + iM(r);
            gradHist(row,col,iO2(r)) = gradHist(row,col,iO2(r)) + iM2(r);
        end
    end
end

function gradHist = spatialInterpolation(gMag, gDir, cellSize, h, w, numberOfRowCells, numberOfColumnCells, numBins, useSignedOrientation)
    columnRef = -1;
    invCellSize = 1/cellSize;
    invInvCellSize = invCellSize/cellSize;

    if (useSignedOrientation ~= 0)
        invBinWidth = numBins / (2*3.14159265);
    else
        invBinWidth = numBins / (3.14159265);
    end

    % initialize containers
    gradHist = zeros([numberOfRowCells numberOfColumnCells numBins],'single');
    iO = zeros([1,h],'int32');
    iM = zeros([1,h]);

    % process column-by-column
    for c = 1:w
        % process row
        for r = 1:h
            % compute the required offset to the to-be indexed orientation channel
            binInd = floor(gDir(r,c) * invBinWidth);
            % wrap
            if binInd >= numBins
                binInd = single(0);
            end
            % store normalized magnitude - corresponds to normalizing the cell values by the sum of the cell values
            iO(r) = binInd + 1;
            iM(r) = gMag(r,c) * invInvCellSize;
        end

        % Bilinear spatial interpolation
        % Key concept: columnRef and rowRef are incremented by 1/cellSize
        % for each column and row increment, respectively. From these
        % two variables the row and column index along with distances
        % to the cell centers corresponding to these indexes are derived.
        % Start with -0.5f for boundaries.
        if c == 1
            columnRef = 0.5 * invCellSize - 0.5;
        end

        % pool left?
        poolLeft = (columnRef >= 0);

        % hold column index
        if poolLeft
            cellColumnIndex = floor(columnRef);
        else
            cellColumnIndex = -1;
        end

        % pool right?
        poolRight = cellColumnIndex < numberOfColumnCells - 1;

        % column distance, use in bilinear interpolation
        columnDistance = columnRef - cellColumnIndex;
        columnRef = columnRef + invCellSize;
        rowRef = 0.5 * invCellSize - 0.5;

        % process rows
        % top rows
        r = 1;
        while r <= cellSize/2
            cellRowIndex = -1;

            % row distance, use in bilinear interpolation
            rowDistance = rowRef - cellRowIndex;
            rowRef = rowRef + invCellSize;

            % compute weights
            weights = computeWeights(columnDistance, rowDistance);

            % spatial binning
            if (poolLeft)
                gradHist(cellRowIndex+2,cellColumnIndex+1,iO(r)) = gradHist(cellRowIndex+2,cellColumnIndex+1,iO(r))+ weights(2) * iM(r);
            end

            if (poolRight)
                gradHist(cellRowIndex+2,cellColumnIndex+2,iO(r)) = gradHist(cellRowIndex+2,cellColumnIndex+2,iO(r)) + weights(4) * iM(r);
            end
            r = r + 1;
        end

        cellRowIndex = floor(rowRef);
        while (cellRowIndex < numberOfRowCells-1)
            rowDistance = rowRef - cellRowIndex;
            rowRef = rowRef + invCellSize;

            % pointer to the appropriate column and row of the first orientation
            weights = computeWeights(columnDistance, rowDistance);

            % spatial binning
            if (poolLeft)
                gradHist(cellRowIndex+1,cellColumnIndex+1,iO(r)) = gradHist(cellRowIndex+1,cellColumnIndex+1,iO(r)) + weights(1)*iM(r);
                gradHist(cellRowIndex+2,cellColumnIndex+1,iO(r)) = gradHist(cellRowIndex+2,cellColumnIndex+1,iO(r)) + weights(2)*iM(r);
            end
            if (poolRight)
                gradHist(cellRowIndex+1,cellColumnIndex+2,iO(r)) = gradHist(cellRowIndex+1,cellColumnIndex+2,iO(r))+ weights(3)*iM(r);
                gradHist(cellRowIndex+2,cellColumnIndex+2,iO(r)) = gradHist(cellRowIndex+2,cellColumnIndex+2,iO(r)) + weights(4)*iM(r);
            end
            r = r+1;
            cellRowIndex = floor(rowRef);
        end

        % bottom rows
        while r <= h
            cellRowIndex = floor(rowRef);

            % compute weights
            rowDistance = rowRef - cellRowIndex;
            rowRef = rowRef + invCellSize;

            % pointer to the appropriate column and row of the first orientation
            weights = computeWeights(columnDistance, rowDistance);

            % spatial binning
            if (poolLeft)
                gradHist(cellRowIndex+1,cellColumnIndex+1,iO(r)) = gradHist(cellRowIndex+1,cellColumnIndex+1,iO(r)) + weights(1)*iM(r);
            end
            if (poolRight)
                gradHist(cellRowIndex+1,cellColumnIndex+2,iO(r)) = gradHist(cellRowIndex+1,cellColumnIndex+2,iO(r))+ weights(3)*iM(r);
            end
            r = r+1;
        end
    end
end

function gradHist = spatialOrientationInterpolation(gMag, gDir, cellSize, h, w, numberOfRowCells, numberOfColumnCells, numBins, useSignedOrientation)
    columnRef = -1;
    invCellSize = 1/cellSize;
    invInvCellSize = invCellSize/cellSize;

    if (useSignedOrientation ~= 0)
        invBinWidth = numBins / (2*3.14159265);
    else
        invBinWidth = numBins / (3.14159265);
    end

    % initialize containers
    gradHist = zeros([numberOfRowCells numberOfColumnCells numBins],'single');

    % process column-by-column
    for c = 1:w
        % copy the appropriate column
        colgDir = gDir(:,c);
        colgMag = gMag(:,c);

        [iO,iO2,iM,iM2] = softBinOrientations(colgDir,colgMag,h,numBins,invBinWidth,invInvCellSize);

        % Bilinear spatial interpolation
        % Key concept: columnRef and rowRef are incremented by 1/cellSize
        % for each column and row increment, respectively. From these
        % two variables the row and column index along with distances
        % to the cell centers corresponding to these indexes are derived.
        % Start with -0.5f for boundaries.
        if c == 1
            columnRef = 0.5 * invCellSize - 0.5;
        end

        % pool left?
        poolLeft = (columnRef >= 0);

        % hold column index
        if poolLeft
            cellColumnIndex = floor(columnRef);
        else
            cellColumnIndex = -1;
        end

        % pool right?
        poolRight = cellColumnIndex < numberOfColumnCells - 1;

        % column distance, use in bilinear interpolation
        columnDistance = columnRef - cellColumnIndex;
        columnRef = columnRef + invCellSize;
        rowRef = 0.5 * invCellSize - 0.5;

        % process rows
        % top rows
        r = 1;
        while r <= cellSize/2
            cellRowIndex = -1;

            % row distance, use in bilinear interpolation
            rowDistance = rowRef - cellRowIndex;
            rowRef = rowRef + invCellSize;

            % compute weights
            weights = computeWeights(columnDistance, rowDistance);

            % spatial binning
            if (poolLeft)
                gradHist(cellRowIndex+2,cellColumnIndex+1,iO(r)) = gradHist(cellRowIndex+2,cellColumnIndex+1,iO(r))+ weights(2) * iM(r);
                gradHist(cellRowIndex+2,cellColumnIndex+1,iO2(r)) = gradHist(cellRowIndex+2,cellColumnIndex+1,iO2(r))+ weights(2) * iM2(r);
            end

            if (poolRight)
                gradHist(cellRowIndex+2,cellColumnIndex+2,iO(r)) = gradHist(cellRowIndex+2,cellColumnIndex+2,iO(r)) + weights(4) * iM(r);
                gradHist(cellRowIndex+2,cellColumnIndex+2,iO2(r)) = gradHist(cellRowIndex+2,cellColumnIndex+2,iO2(r)) + weights(4) * iM2(r);
            end
            r = r + 1;
        end

        cellRowIndex = floor(rowRef);
        while (cellRowIndex < numberOfRowCells-1)
            rowDistance = rowRef - cellRowIndex;
            rowRef = rowRef + invCellSize;

            % pointer to the appropriate column and row of the first orientation
            weights = computeWeights(columnDistance, rowDistance);

            % spatial binning
            if (poolLeft)
                gradHist(cellRowIndex+1,cellColumnIndex+1,iO(r)) = gradHist(cellRowIndex+1,cellColumnIndex+1,iO(r)) + weights(1)*iM(r);
                gradHist(cellRowIndex+2,cellColumnIndex+1,iO(r)) = gradHist(cellRowIndex+2,cellColumnIndex+1,iO(r)) + weights(2)*iM(r);
                gradHist(cellRowIndex+1,cellColumnIndex+1,iO2(r)) = gradHist(cellRowIndex+1,cellColumnIndex+1,iO2(r)) + weights(1)*iM2(r);
                gradHist(cellRowIndex+2,cellColumnIndex+1,iO2(r)) = gradHist(cellRowIndex+2,cellColumnIndex+1,iO2(r)) + weights(2)*iM2(r);
            end
            if (poolRight)
                gradHist(cellRowIndex+1,cellColumnIndex+2,iO(r)) = gradHist(cellRowIndex+1,cellColumnIndex+2,iO(r))+ weights(3)*iM(r);
                gradHist(cellRowIndex+2,cellColumnIndex+2,iO(r)) = gradHist(cellRowIndex+2,cellColumnIndex+2,iO(r)) + weights(4)*iM(r);
                gradHist(cellRowIndex+1,cellColumnIndex+2,iO2(r)) = gradHist(cellRowIndex+1,cellColumnIndex+2,iO2(r))+ weights(3)*iM2(r);
                gradHist(cellRowIndex+2,cellColumnIndex+2,iO2(r)) = gradHist(cellRowIndex+2,cellColumnIndex+2,iO2(r)) + weights(4)*iM2(r);
            end
            r = r+1;
            cellRowIndex = floor(rowRef);
        end

        % bottom rows
        while r <= h
            cellRowIndex = floor(rowRef);

            % compute weights
            rowDistance = rowRef - cellRowIndex;
            rowRef = rowRef + invCellSize;

            % pointer to the appropriate column and row of the first orientation
            weights = computeWeights(columnDistance, rowDistance);

            % spatial binning
            if (poolLeft)
                gradHist(cellRowIndex+1,cellColumnIndex+1,iO(r)) = gradHist(cellRowIndex+1,cellColumnIndex+1,iO(r)) + weights(1)*iM(r);
                gradHist(cellRowIndex+1,cellColumnIndex+1,iO2(r)) = gradHist(cellRowIndex+1,cellColumnIndex+1,iO2(r)) + weights(1)*iM2(r);
            end
            if (poolRight)
                gradHist(cellRowIndex+1,cellColumnIndex+2,iO(r)) = gradHist(cellRowIndex+1,cellColumnIndex+2,iO(r))+ weights(3)*iM(r);
                gradHist(cellRowIndex+1,cellColumnIndex+2,iO2(r)) = gradHist(cellRowIndex+1,cellColumnIndex+2,iO2(r))+ weights(3)*iM2(r);
            end
            r = r+1;
        end
    end
end

function gradHistOut = normalizeBoundaries(gradHist, numberOfRowCells, numberOfColumnCells, numBins)
    % normalize boundaries
    for n = 0:numBins-1
        % left boundary
        c = 0;
        for r = 0:numberOfRowCells-1
            gradHist(r+1,c+1,n+1) = gradHist(r+1,c+1,n+1) * 8/7;
        end
        % top boundary
        r = 0;
        for c = 0:numberOfColumnCells-1
            gradHist(r+1,c+1,n+1) = gradHist(r+1,c+1,n+1) * 8/7;
        end
        % right boundary
        c = numberOfColumnCells - 1;
        for r = 0:numberOfRowCells-1
            gradHist(r+1,c+1,n+1) = gradHist(r+1,c+1,n+1) * 8/7;
        end
        % bottom boundary
        r = numberOfRowCells - 1;
        for c = 0:numberOfColumnCells-1
            gradHist(r+1,c+1,n+1) = gradHist(r+1,c+1,n+1) * 8/7;
        end
    end

    gradHistOut = gradHist;
end

function [iO,iO2,iM,iM2] = softBinOrientations(colgDir,colgMag,h,numBins,invBinWidth,invInvCellSize)
    iO = coder.nullcopy(zeros([1,h],'int32'));
    iO2 = coder.nullcopy(zeros([1,h],'int32'));
    iM = coder.nullcopy(zeros([1,h]));
    iM2 = coder.nullcopy(zeros([1,h]));

    for i = 1:h
        binFraction = colgDir(i) * invBinWidth;
        binIndex1 = floor(binFraction);
        dis = binFraction - binIndex1;

        if binIndex1 >= numBins
            binIndex1 = single(0);
        end
        binIndex2 = binIndex1 + 1;
        if (binIndex2 >= numBins)
            binIndex2 = single(0);
        end

        % Store bin index in output array. Remember that the orientation bins are arranged along the 3rd dimension.
        iO(i) = binIndex1 + 1;
        iO2(i) = binIndex2 + 1;

        % store the normalized and proportioned magnitude values
        val = colgMag(i) * invInvCellSize;
        iM2(i) = dis * val;
        iM(i) = val - iM2(i);
    end
end

function weights = computeWeights(columnDistance, rowDistance)
    weights = zeros([1 4]);
    weights(1) = 1.0 - columnDistance - rowDistance + columnDistance * rowDistance; %bottom-left area
    weights(2) = rowDistance - columnDistance * rowDistance; % bottom-right
    weights(3) = columnDistance - columnDistance * rowDistance; % top-right
    weights(4) = columnDistance * rowDistance; % top-left
end
