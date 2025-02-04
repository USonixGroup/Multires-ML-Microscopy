function [inImg] = convTriR1(inImg) %#codegen
% SIMD-friendly 2D triangle filter convolution (the 1D triangle filter f is
% [1:r r+1 r:-1:1]/(r+1)^2, the 2D version is simply conv2(f,f')).
% This function handles the case of integer filter with radius = 1
% INPUTS
%  inImg    - [m n k] input k channel single image
%
% OUTPUTS
%  outImg      - [m x n x k] smoothed image

% Copyright 2020 The MathWorks, Inc.
%
% References
% ----------
%   Dollar, Piotr, et al. "Fast feature pyramids for object detection."
%   Pattern Analysis and Machine Intelligence, IEEE Transactions on 36.8
%   (2014): 1532-1545.

    coder.inline('always');
    r = 1;
    p = 12/r/(r+2)-2;
    fRow = [1 p 1]/(2+p);
    convMode = true;
    if (convMode)
        fRow = flip(fRow);
    end
    coder.const(fRow);
    fCol = fRow';
    coder.const(fCol);
    inImg = conv3d3x3r1ValidSymm(inImg, fCol, fRow);
end

function inImg = conv3d3x3r1ValidSymm(inImg, hCol, hRow)
    coder.inline('always');
    nRows = size(inImg,1);
    nCols = size(inImg,2);
    nChns = size(inImg,3);

    %Change temp buffer size from (nRows x nCols) to (nRows x 2)
    temp = coder.nullcopy(zeros(nRows,2,'single'));
    for iChn = 1:nChns
        idx1 = int32(1);
        idx2 = int32(2);
        % process hRow conv - 1st column
        temp(:,idx1) = inImg(:,1,iChn)*hRow(1) + inImg(:,1,iChn)*hRow(2) + inImg(:,1+1,iChn)*hRow(3);
        for ix = 2:nCols-1
            % process hRow conv - temp2 using inImg(,ix,)
            temp(:,idx2) = inImg(:,ix-1,iChn)*hRow(1) + inImg(:,ix,iChn)*hRow(2) + inImg(:,ix+1,iChn)*hRow(3);
            % process hCol conv - out(,ix-1,) using temp1
            inImg(1,ix-1,iChn) = temp(1,idx1)*hCol(1) + temp(1,idx1)*hCol(2) + temp(1+1,idx1)*hCol(3);
            inImg(2:nRows-1,ix-1,iChn) = temp(1:nRows-2,idx1)*hCol(1) + temp(2:nRows-1,idx1)*hCol(2) + temp(3:nRows,idx1)*hCol(3);
            inImg(nRows,ix-1,iChn) = temp(nRows-1,idx1)*hCol(1) + temp(nRows,idx1)*hCol(2) + temp(nRows,idx1)*hCol(3);
            % swap indices
            t1 = idx1;
            idx1 = idx2;
            idx2 = t1;
        end
        % process hRow conv - last temp using inImg(,nCols,)
        temp(:,idx2) = inImg(:,nCols-1,iChn)*hRow(1) + inImg(:,nCols,iChn)*hRow(2) + inImg(:,nCols,iChn)*hRow(3);
        % process hCol conv - out(,nCols-1,)
        inImg(1,nCols-1,iChn) = temp(1,idx1)*hCol(1) + temp(1,idx1)*hCol(2) + temp(1+1,idx1)*hCol(3);
        inImg(2:nRows-1,nCols-1,iChn) = temp(1:nRows-2,idx1)*hCol(1) + temp(2:nRows-1,idx1)*hCol(2) + temp(3:nRows,idx1)*hCol(3);
        inImg(nRows,nCols-1,iChn) = temp(nRows-1,idx1)*hCol(1) + temp(nRows,idx1)*hCol(2) + temp(nRows,idx1)*hCol(3);
        % process hCol conv - out(,nCols,)
        inImg(1,nCols,iChn) = temp(1,idx2)*hCol(1) + temp(1,idx2)*hCol(2) + temp(1+1,idx2)*hCol(3);
        inImg(2:nRows-1,nCols,iChn) = temp(1:nRows-2,idx2)*hCol(1) + temp(2:nRows-1,idx2)*hCol(2)+ temp(3:nRows,idx2)*hCol(3);
        inImg(nRows,nCols,iChn) = temp(nRows-1,idx2)*hCol(1) + temp(nRows,idx2)*hCol(2) + temp(nRows,idx2)*hCol(3);
    end
end
