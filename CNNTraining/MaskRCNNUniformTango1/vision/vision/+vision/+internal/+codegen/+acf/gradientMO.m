function [grM,grO] = gradientMO(inImg, params) %#codegen
% Compute gradient magnitude and orientation.

% Copyright 2020 The MathWorks, Inc.
%
% References
% ----------
%   Dollar, Piotr, et al. "Fast feature pyramids for object detection."
%   Pattern Analysis and Machine Intelligence, IEEE Transactions on 36.8
%   (2014): 1532-1545.

% The following commented code performs the same task as this function
%----------------------------------------------------------------------
% h = size(inImg,1);
% w = size(inImg,2);
% [Gx3d,Gy3d]=gradient(inImg);
% grM3d=sqrt(Gx3d.^2+Gy3d.^2);
% [grM,maxIdx] = max(grM3d,[],3);
% Gx=coder.nullcopy(zeros(h,w,'like',inImg));
% Gy=coder.nullcopy(zeros(h,w,'like',inImg));
% for i=1:h
%     for j=1:w
%         Gx(i,j)= Gx3d(i,j,maxIdx(i,j));
%         Gy(i,j)= Gy3d(i,j,maxIdx(i,j));
%     end
% end
%
% grO=atan2(Gy,Gx);
%
% normRad   = params.NormalizationRadius;
% normConst = params.NormalizationConstant;
% fullOrient = params.FullOrientation;
%
% if (fullOrient == 0)
%     for  n = 1: h * w
%         if (grO(n) < 0)
%             grO(n) = grO(n) + pi;
%         end
%     end
% else
%     % orientation between [0 2*pi]
%     twoPi = pi * 2;
%     for  n = 1: h * w
%         if (grO(n) < 0)
%             grO(n) = grO(n) + twoPi;
%         end
%     end
% end
%
% if normRad > 0
%     %S = vision.internal.acf.convTri(grM, normRad, 1);
%     S = convTri(grM, normRad, 1);
%     grM = grM ./ (S + normConst);
% end
%----------------------------------------------------------------------

    nRows = size(inImg,1);
    nCols = size(inImg,2);
    fullOrient = params.FullOrientation;
    r = single(0.5);

    offSet = pi;
    if fullOrient ~= 0
        offSet = offSet + pi;
    end

    Gx3d = coder.nullcopy(zeros(nRows,1,'like',inImg));
    Gy3d = coder.nullcopy(zeros(nRows,1,'like',inImg));

    grM = coder.nullcopy(zeros(nRows,nCols,'like',inImg));
    Gx  = coder.nullcopy(zeros(nRows,nCols,'like',inImg));
    grO = coder.nullcopy(zeros(nRows,nCols,'like',inImg));

    % compute gradient in Gx3d, Gy3d
    iChn = 1;
    % process column ix=1
    ix = 1;
    Gx3d(:) = inImg(:,ix+1,iChn) - inImg(:,ix,iChn);
    Gx(:,ix) = Gx3d;
    % process rows for ix=1
    iy = 1;
    Gy3d(iy) = inImg(iy+1,ix,iChn) - inImg(iy,ix,iChn);
    iy = nRows;
    Gy3d(iy) = inImg(iy,ix,iChn) - inImg(iy-1,ix,iChn);
    Gy3d(2:nRows-1) = (inImg(3:nRows,ix,iChn) - inImg(1:nRows-2,ix,iChn)) .* r;
    grO(:,ix) = Gy3d;
    grM(:,ix) = sqrt(Gx3d.*Gx3d + Gy3d.*Gy3d);

    % process column ix=nCols
    ix = nCols;
    Gx3d(:) = inImg(:,ix,iChn)-inImg(:,ix-1,iChn);
    Gx(:,ix) = Gx3d;
    % process rows for ix=nCols
    iy = 1;
    Gy3d(iy) = inImg(iy+1,ix,iChn)-inImg(iy,ix,iChn);
    iy = nRows;
    Gy3d(iy) = inImg(iy,ix,iChn)-inImg(iy-1,ix,iChn);
    Gy3d(2:nRows-1) = (inImg(3:nRows,ix,iChn)-inImg(1:nRows-2,ix,iChn)) .* r;
    grO(:,ix) = Gy3d;
    grM(:,ix) = sqrt(Gx3d.*Gx3d + Gy3d.*Gy3d);

    for ix = 2:nCols-1
        Gx3d(:) = (inImg(:,ix+1,iChn)-inImg(:,ix-1,iChn)) .* r;
        Gx(:,ix) = Gx3d;

        iy = 1;
        Gy3d(iy) = (inImg(iy+1,ix,iChn)-inImg(iy,ix,iChn));
        iy = nRows;
        Gy3d(iy) = (inImg(iy,ix,iChn)-inImg(iy-1,ix,iChn));
        Gy3d(2:nRows-1) = (inImg(3:nRows,ix,iChn)-inImg(1:nRows-2,ix,iChn)) .* r;
        grO(:,ix) = Gy3d;

        grM(:,ix) = sqrt(Gx3d.*Gx3d + Gy3d.*Gy3d);
    end

    iChn = 2;

    % process column ix=1
    ix = 1;
    Gx3d(:) = (inImg(:,ix+1,iChn)-inImg(:,ix,iChn));
    % process rows for ix=1
    iy = 1;
    Gy3d(iy) = (inImg(iy+1,ix,iChn)-inImg(iy,ix,iChn));
    iy = nRows;
    Gy3d(iy) = (inImg(iy,ix,iChn)-inImg(iy-1,ix,iChn));
    Gy3d(2:nRows-1) = (inImg(3:nRows,ix,iChn)-inImg(1:nRows-2,ix,iChn)) .* r;
    tempCol = sqrt(Gx3d.*Gx3d + Gy3d.*Gy3d);
    for iy = 1:nRows
        if (tempCol(iy) > grM(iy,ix))
            grM(iy,ix) = tempCol(iy);
            Gx(iy,ix) = Gx3d(iy);
            grO(iy,ix) = Gy3d(iy);
        end
    end

    % process column ix=nCols
    ix = nCols;
    Gx3d(:) = (inImg(:,ix,iChn)-inImg(:,ix-1,iChn));
    % process rows for ix=nCols
    iy = 1;
    Gy3d(iy) = (inImg(iy+1,ix,iChn)-inImg(iy,ix,iChn));
    iy = nRows;
    Gy3d(iy) = (inImg(iy,ix,iChn)-inImg(iy-1,ix,iChn));
    Gy3d(2:nRows-1) = (inImg(3:nRows,ix,iChn)-inImg(1:nRows-2,ix,iChn)) .* r;
    tempCol = sqrt(Gx3d.*Gx3d + Gy3d.*Gy3d);
    for iy = 1:nRows
        if (tempCol(iy) > grM(iy,ix))
            grM(iy,ix) = tempCol(iy);
            Gx(iy,ix) = Gx3d(iy);
            grO(iy,ix) = Gy3d(iy);
        end
    end

    for ix = 2:nCols-1
        Gx3d(:) = (inImg(:,ix+1,iChn)-inImg(:,ix-1,iChn)) .* r;
        iy = 1;
        Gy3d(iy) = (inImg(iy+1,ix,iChn)-inImg(iy,ix,iChn));
        iy = nRows;
        Gy3d(iy) = (inImg(iy,ix,iChn)-inImg(iy-1,ix,iChn));
        Gy3d(2:nRows-1) = (inImg(3:nRows,ix,iChn)-inImg(1:nRows-2,ix,iChn)) .* r;
        tempCol = sqrt(Gx3d.*Gx3d + Gy3d.*Gy3d);
        for iy = 1:nRows
            if (tempCol(iy) > grM(iy,ix))
                grM(iy,ix) = tempCol(iy);
                Gx(iy,ix) = Gx3d(iy);
                grO(iy,ix) = Gy3d(iy);
            end
        end
    end

    iChn = 3;

    % process column ix=1
    ix = 1;
    Gx3d(:) = (inImg(:,ix+1,iChn)-inImg(:,ix,iChn));
    % process rows for ix=1
    iy = 1;
    Gy3d(iy) = (inImg(iy+1,ix,iChn)-inImg(iy,ix,iChn));
    iy = nRows;
    Gy3d(iy) = (inImg(iy,ix,iChn)-inImg(iy-1,ix,iChn));
    Gy3d(2:nRows-1) = (inImg(3:nRows,ix,iChn)-inImg(1:nRows-2,ix,iChn)) .* r;
    tempCol = sqrt(Gx3d.*Gx3d + Gy3d.*Gy3d);
    for iy = 1:nRows
        if (tempCol(iy) > grM(iy,ix))
            grM(iy,ix) = tempCol(iy);
            Gx(iy,ix) = Gx3d(iy);
            grO(iy,ix) = Gy3d(iy);
        end
        grO(iy,ix) = atan2(grO(iy,ix),Gx(iy,ix));
        if (grO(iy,ix) < 0)
            grO(iy,ix) = grO(iy,ix) + offSet;
        end
    end

    % process column ix=nCols
    ix = nCols;
    Gx3d(:) = (inImg(:,ix,iChn)-inImg(:,ix-1,iChn));
    % process rows for ix=nCols
    iy = 1;
    Gy3d(iy) = (inImg(iy+1,ix,iChn)-inImg(iy,ix,iChn));
    iy = nRows;
    Gy3d(iy) = (inImg(iy,ix,iChn)-inImg(iy-1,ix,iChn));
    Gy3d(2:nRows-1) = (inImg(3:nRows,ix,iChn)-inImg(1:nRows-2,ix,iChn)) .* r;
    tempCol = sqrt(Gx3d.*Gx3d + Gy3d.*Gy3d);
    for iy = 1:nRows
        if (tempCol(iy) > grM(iy,ix))
            grM(iy,ix) = tempCol(iy);
            Gx(iy,ix) = Gx3d(iy);
            grO(iy,ix) = Gy3d(iy);
        end
        grO(iy,ix) = atan2(grO(iy,ix),Gx(iy,ix));
        if (grO(iy,ix) < 0)
            grO(iy,ix) = grO(iy,ix) + offSet;
        end
    end

    for ix = 2:nCols-1
        Gx3d(:) = (inImg(:,ix+1,iChn)-inImg(:,ix-1,iChn)) .* r;
        iy = 1;
        Gy3d(iy) = (inImg(iy+1,ix,iChn)-inImg(iy,ix,iChn));
        iy = nRows;
        Gy3d(iy) = (inImg(iy,ix,iChn)-inImg(iy-1,ix,iChn));
        Gy3d(2:nRows-1) = (inImg(3:nRows,ix,iChn)-inImg(1:nRows-2,ix,iChn)) .* r;
        tempCol = sqrt(Gx3d.*Gx3d + Gy3d.*Gy3d);
        for iy = 1:nRows
            if (tempCol(iy) > grM(iy,ix))
                grM(iy,ix) = tempCol(iy);
                Gx(iy,ix) = Gx3d(iy);
                grO(iy,ix) = Gy3d(iy);
            end
            grO(iy,ix) = atan2(grO(iy,ix),Gx(iy,ix));
            if (grO(iy,ix) < 0)
                grO(iy,ix) = grO(iy,ix) + offSet;
            end
        end
    end

    normRad   = params.NormalizationRadius;
    normConst = params.NormalizationConstant;

    % Normalize gradient magnitude if a normalization radius is specified
    if normRad > 0
        S = vision.internal.codegen.acf.convTri(grM, normRad, 1) + normConst;
        grM = grM ./ S;
    end
end
