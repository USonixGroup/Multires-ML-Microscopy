function [inImg] = convTriR5(inImg) %#codegen
% SIMD-friendly 2D triangle filter convolution (the 1D triangle filter f is
% [1:r r+1 r:-1:1]/(r+1)^2, the 2D version is simply conv2(f,f')).
% This function handles the case of integer filter with radius = 5
% INPUTS
%  inImg      - [m n k] input k channel single image
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

    r = 5;
    fRow = single([1:r r+1 r:-1:1]/(r+1)^2);
    convMode = true;
    if (convMode)
        fRow = flip(fRow);
    end
    coder.const(fRow);
    fCol = fRow';
    coder.const(fCol);
    [inImg] = conv2d11x11r5ValidSymm(inImg, fRow, fCol);
end

function [inImg] = conv2d11x11r5ValidSymm(inImg, hRow, hCol)
    coder.inline('always');
    nRows = size(inImg,1);
    nCols = size(inImg,2);
    nChns = size(inImg,3);
    temp = coder.nullcopy(zeros(nRows,nCols,'single'));
    rad = 5;
    nFiltSize = numel(hRow);

    % X-direction
    for iChn = 1:nChns
        % process first 5 cols
        temp(:,1) = inImg(:,5,iChn)*hRow(1) + inImg(:,4,iChn)*hRow(2) + inImg(:,3,iChn)*hRow(3)+ inImg(:,2,iChn)*hRow(4) + inImg(:,1,iChn)*hRow(5);
        for i = 6:nFiltSize
            temp(:,1) = temp(:,1) + inImg(:,i-5,iChn).*hRow(i);
        end
        temp(:,2) = inImg(:,4,iChn)*hRow(1) + inImg(:,3,iChn)*hRow(2) + inImg(:,2,iChn)*hRow(3)+ inImg(:,1,iChn)*hRow(4);
        for i = 5:nFiltSize
            temp(:,2) = temp(:,2) + inImg(:,i-4,iChn).*hRow(i);
        end
        temp(:,3) = inImg(:,3,iChn)*hRow(1) + inImg(:,2,iChn)*hRow(2) + inImg(:,1,iChn)*hRow(3);
        for i = 4:nFiltSize
            temp(:,3) = temp(:,3) + inImg(:,i-3,iChn).*hRow(i);
        end
        temp(:,4) = inImg(:,2,iChn)*hRow(1) + inImg(:,1,iChn)*hRow(2);
        for i = 3:nFiltSize
            temp(:,4) = temp(:,4) + inImg(:,i-2,iChn).*hRow(i);
        end
        temp(:,5) = inImg(:,1,iChn)*hRow(1);
        for i = 2:nFiltSize
            temp(:,5) = temp(:,5) + inImg(:,i-1,iChn).*hRow(i);
        end

        % process last 5 cols
        temp(:,nCols-4) = inImg(:,nCols,iChn)*hRow(11);
        for i = 1:nFiltSize-1
            temp(:,nCols-4) = temp(:,nCols-4) + inImg(:,nCols-nFiltSize+i+1,iChn).*hRow(i);
        end
        temp(:,nCols-3) = inImg(:,nCols,iChn)*hRow(10) + inImg(:,nCols-1,iChn)*hRow(11);
        for i = 1:nFiltSize-2
            temp(:,nCols-3) = temp(:,nCols-3) + inImg(:,nCols-nFiltSize+i+2,iChn).*hRow(i);
        end
        temp(:,nCols-2) = inImg(:,nCols,iChn)*hRow(9) + inImg(:,nCols-1,iChn)*hRow(10) + inImg(:,nCols-2,iChn)*hRow(11);
        for i = 1:nFiltSize-3
            temp(:,nCols-2) = temp(:,nCols-2) + inImg(:,nCols-nFiltSize+i+3,iChn).*hRow(i);
        end
        temp(:,nCols-1) = inImg(:,nCols,iChn)*hRow(8) + inImg(:,nCols-1,iChn)*hRow(9) + inImg(:,nCols-2,iChn)*hRow(10) + inImg(:,nCols-3,iChn)*hRow(11);
        for i = 1:nFiltSize-4
            temp(:,nCols-1) = temp(:,nCols-1) + inImg(:,nCols-nFiltSize+i+4,iChn).*hRow(i);
        end
        temp(:,nCols) = inImg(:,nCols,iChn)*hRow(7) + inImg(:,nCols-1,iChn)*hRow(8) + inImg(:,nCols-2,iChn)*hRow(9) + inImg(:,nCols-3,iChn)*hRow(10) + inImg(:,nCols-4,iChn)*hRow(11);
        for i = 1:nFiltSize-5
            temp(:,nCols) = temp(:,nCols) + inImg(:,nCols-nFiltSize+i+5,iChn).*hRow(i);
        end
        
        % process interior cols
        for ix = rad+1:nCols-rad
            temp(:,ix) = inImg(:,ix-rad,iChn).*hRow(1) + inImg(:,ix-rad+1,iChn).*hRow(2) + inImg(:,ix-rad+2,iChn).*hRow(3)...
                       +inImg(:,ix-rad+3,iChn).*hRow(4) + inImg(:,ix-rad+4,iChn).*hRow(5) + inImg(:,ix-rad+5,iChn).*hRow(6)...
                       +inImg(:,ix-rad+6,iChn).*hRow(7) + inImg(:,ix-rad+7,iChn).*hRow(8) + inImg(:,ix-rad+8,iChn).*hRow(9)...
                       +inImg(:,ix-rad+9,iChn).*hRow(10) + inImg(:,ix-rad+10,iChn).*hRow(11);
        end
    end

    % Y-direction
    for iChn = 1:nChns
        % process interior rows
        for ix = 1:nCols
            inImg(rad+1:nRows-rad,ix,iChn) = temp(1:nRows-10,ix)*hCol(1) + temp(2:nRows-9,ix)*hCol(2) ...
                                +temp(3:nRows-8,ix)*hCol(3) + temp(4:nRows-7,ix)*hCol(4) + temp(5:nRows-6,ix)*hCol(5)...
                                +temp(6:nRows-5,ix)*hCol(6) + temp(7:nRows-4,ix)*hCol(7) + temp(8:nRows-3,ix)*hCol(8)...
                                +temp(9:nRows-2,ix)*hCol(9) + temp(10:nRows-1,ix)*hCol(10) + temp(11:nRows,ix)*hCol(11);
        end
        for ix = 1:nCols
            % process first 5 rows
            inImg(1,ix,iChn) = temp(5,ix)*hCol(1) + temp(4,ix)*hCol(2) + temp(3,ix)*hCol(3) + temp(2,ix)*hCol(4) + temp(1,ix)*hCol(5);
            for i = 6:nFiltSize
                inImg(1,ix,iChn) = inImg(1,ix,iChn) + temp(i-5,ix).*hCol(i);
            end
            inImg(2,ix,iChn) = temp(4,ix)*hCol(1) + temp(3,ix)*hCol(2) + temp(2,ix)*hCol(3) + temp(1,ix)*hCol(4);
            for i = 5:nFiltSize
                inImg(2,ix,iChn) = inImg(2,ix,iChn) + temp(i-4,ix).*hCol(i);
            end
            inImg(3,ix,iChn) = temp(3,ix)*hCol(1) + temp(2,ix)*hCol(2) + temp(1,ix)*hCol(3);
            for i = 4:nFiltSize
                inImg(3,ix,iChn) = inImg(3,ix,iChn) + temp(i-3,ix).*hCol(i);
            end
            inImg(4,ix,iChn) = temp(2,ix)*hCol(1) + temp(1,ix)*hCol(2);
            for i = 3:nFiltSize
                inImg(4,ix,iChn) = inImg(4,ix,iChn) + temp(i-2,ix).*hCol(i);
            end
            inImg(5,ix,iChn) = temp(1,ix)*hCol(1);
            for i = 2:nFiltSize
                inImg(5,ix,iChn) = inImg(5,ix,iChn) + temp(i-1,ix).*hCol(i);
            end
            % process last 5 rows
            inImg(nRows-4,ix,iChn) = temp(nRows,ix)*hCol(11);
            for i=1:nFiltSize-1
                inImg(nRows-4,ix,iChn) = inImg(nRows-4,ix,iChn) + temp(nRows-nFiltSize+i+1,ix).*hCol(i);
            end
            inImg(nRows-3,ix,iChn) = temp(nRows,ix)*hCol(10) + temp(nRows-1,ix)*hCol(11);
            for i = 1:nFiltSize-2
                inImg(nRows-3,ix,iChn) = inImg(nRows-3,ix,iChn) + temp(nRows-nFiltSize+i+2,ix).*hCol(i);
            end
            inImg(nRows-2,ix,iChn) = temp(nRows,ix)*hCol(9) + temp(nRows-1,ix)*hCol(10) + temp(nRows-2,ix)*hCol(11);
            for i = 1:nFiltSize-3
                inImg(nRows-2,ix,iChn) = inImg(nRows-2,ix,iChn) + temp(nRows-nFiltSize+i+3,ix).*hCol(i);
            end
            inImg(nRows-1,ix,iChn) = temp(nRows,ix)*hCol(8) + temp(nRows-1,ix)*hCol(9) + temp(nRows-2,ix)*hCol(10) + temp(nRows-3,ix)*hCol(11);
            for i = 1:nFiltSize-4
                inImg(nRows-1,ix,iChn) = inImg(nRows-1,ix,iChn) + temp(nRows-nFiltSize+i+4,ix).*hCol(i);
            end
            inImg(nRows,ix,iChn) = temp(nRows,ix)*hCol(7) + temp(nRows-1,ix)*hCol(8) + temp(nRows-2,ix)*hCol(9) + temp(nRows-3,ix)*hCol(10) + temp(nRows-4,ix)*hCol(11);
            for i = 1:nFiltSize-5
                inImg(nRows,ix,iChn) = inImg(nRows,ix,iChn) + temp(nRows-nFiltSize+i+5,ix).*hCol(i);
            end
        end
    end
end

