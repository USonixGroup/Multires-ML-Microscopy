function outImg = imResample(inImg, nR1, nC1, norm) %#codegen
% Fast image downsampling/upsampling using bilinear interpolation

% Copyright 2020 The MathWorks, Inc.
%
% References
% ----------
%   Dollar, Piotr, et al. "Fast feature pyramids for object detection."
%   Pattern Analysis and Machine Intelligence, IEEE Transactions on 36.8
%   (2014): 1532-1545.

    ha = int32(size(inImg,1));
    wa = int32(size(inImg,2));
    hb = int32(nR1);
    wb = int32(nC1);
    nCh = size(inImg,3);
    outImg = zeros(hb,wb,nCh,'single');
    C = zeros(ha+4,1,'single');
    C(ha+1:ha+4) = single(0);
    [wn, xas, xbs, xwts, xbd] = resampleCoef(wa, wb, 0);
    [hn, yas, ybs, ywts, ybd] = resampleCoef(ha, hb, 4);

    if (wa == 2*wb)
        norm = norm/single(2);
    end
    if (wa == 3*wb)
        norm = norm/single(3);
    end
    if (wa == 4*wb)
        norm = norm/single(4);
    end
    norm = norm/single(1+1e-6);
    for y = 1:hn
        ywts(y) = ywts(y).*norm;
    end
    x1 = int32(-1);

    % resample each channel in turn
    for z = 1:nCh
        for x = 1:wb
            if (x == 1)
                x1 = int32(1);
            end
            xa = xas(x1)+1;
            xb = xbs(x1)+1;
            wt = xwts(x1);
            wt1 = 1-wt;

            if (wa == 2*wb)
                for y = 1:ha
                    C(y) = inImg(y,xa,z) + inImg(y,xa+1,z);
                end
                x1 = x1+2;
            elseif (wa == 3*wb)
                for y = 1:ha
                    C(y) = inImg(y,xa,z) + inImg(y,xa+1,z)+ inImg(y,xa+2,z);
                end
                x1 = x1+3;
            elseif (wa == 4*wb )
                for y = 1:ha
                    C(y) = inImg(y,xa,z) + inImg(y,xa+1,z)+ inImg(y,xa+2,z)+inImg(y,xa+3,z);
                end
                x1 = x1+4;
            elseif (wa > wb)
                m = int32(1);
                while (x1+m < wn && (xb-1) == xbs(x1+m))
                    m = m+1;
                end
                if (m == 1)
                    for y = 1:ha
                        C(y) = inImg(y,xa,z)*xwts(x1+0);
                    end
                end
                if(m == 2)
                    for y = 1:ha
                        C(y) = inImg(y,xa,z)*xwts(x1+0) + inImg(y,xa+1,z)*xwts(x1+1);
                    end
                end
                if (m == 3)
                    for y = 1:ha
                        C(y) = inImg(y,xa,z)*xwts(x1+0) + inImg(y,xa+1,z)*xwts(x1+1) ...
                             + inImg(y,xa+2,z)*xwts(x1+2);
                    end
                end
                if (m >= 4)
                    for y = 1:ha
                        C(y) = inImg(y,xa,z)*xwts(x1+0) + inImg(y,xa+1,z)*xwts(x1+1) ...
                             + inImg(y,xa+2,z)*xwts(x1+2)+ inImg(y,xa+3,z)*xwts(x1+3);
                    end
                end
                for x0 = 4:m-1
                    wt1 = xwts(x1+x0);
                    for y = 1:ha
                        C(y) = C(y)+ inImg(y,xa+x0,z)*wt1;
                    end
                end
                x1 = x1+m;
            else
                xBd = x-1<xbd(1) || x-1>=wb-xbd(2);
                x1 = x1+1;
                if (xBd)
                    C(1:ha,1) = inImg(1:ha,xa,z);
                end
                if (~xBd)
                    for y = 1:ha
                        C(y) = inImg(y,xa,z)*wt + inImg(y,xa+1,z)*wt1;
                    end
                end
            end

            if (ha == hb*2)
                r2 = norm/2;
                for y = 0:hb-1
                    outImg(y+1,xb,z) = (C(2*y+1)+C(2*y+2))*r2;
                end
            elseif (ha == hb*3)
                for y = 0:hb-1
                    outImg(y+1,xb,z) = (C(3*y+1)+C(3*y+2)+C(3*y+3))*(norm/3);
                end
            elseif (ha == hb*4)
                for y = 0:hb-1
                    outImg(y+1,xb,z) = (C(4*y+1)+C(4*y+2)+C(4*y+3)+C(4*y+4))*(norm/4);
                end
            elseif (ha>hb)
                if (ybd(1) == 2)
                    for y = 0:hb-1
                        ya = yas(y*4+1)+1;
                        outImg(y+1,xb,z) = C(ya+0)*ywts(y*4+1+0) + C(ya+1)*ywts(y*4+1+1);
                    end
                end
                if (ybd(1) == 3)
                    for y = 0:hb-1
                        ya = yas(y*4+1)+1;
                        outImg(y+1,xb,z) = C(ya+0)*ywts(y*4+1+0) + C(ya+1)*ywts(y*4+1+1)+ C(ya+2)*ywts(y*4+1+2);
                    end
                end
                if (ybd(1) == 4)
                    for y = 0:hb-1
                        ya = yas(y*4+1)+1;
                        outImg(y+1,xb,z) = C(ya+0)*ywts(y*4+1+0) + C(ya+1)*ywts(y*4+1+1)...
                            + C(ya+2)*ywts(y*4+1+2) + C(ya+3)*ywts(y*4+1+3);
                    end
                end
                if (ybd(1) > 4)
                    for y = 0:hn-1
                        outImg(ybs(y+1)+1,xb,z) = outImg(ybs(y+1)+1,xb,z) + C(yas(y+1)+1) * ywts(y+1);
                    end
                end
            else
                y = 1;
                while (y <= ybd(1))
                    outImg(y,xb,z) = C(yas(y)+1)*ywts(y);
                    y = y+1;
                end
                while (y <= hb-ybd(2))
                    outImg(y,xb,z) = C(yas(y)+1)*ywts(y)+C(yas(y)+2)*(norm-ywts(y));
                    y = y+1;
                end
                while (y <= hb)
                    outImg(y,xb,z) = C(yas(y)+1)*ywts(y);
                    y = y+1;
                end
            end

        end
    end
end %function imResampleCore

function [n, yas, ybs, wts, bd] = resampleCoef(ha, hb, pad)
% Compute interpolation coefficients for resampling a single column.

    s = single(hb)/single(ha);
    sInv = 1/s;
    wt0 = single(1e-3)*s;
    ds = ha>hb;
    bd = zeros(2,1,'int32');

    if (ds)
        n = int32(1);
        if (pad > 2)
            nMax = ha+pad*hb;
        else
            nMax = ha+2*hb;
        end
    else
        n = hb;
        nMax = hb;
    end

    % initialize memory
    wts = zeros(nMax,1,'single');
    yas = zeros(nMax,1,'int32');
    ybs = zeros(nMax,1,'int32');
    if (ds)
        for yb = 0:hb-1
            % create coefficients for downsampling
            ya0f = single(yb).*sInv;
            ya1f = ya0f+sInv;
            W = single(0);
            ya0 = int32(ceil(ya0f));
            ya1 = int32(floor(ya1f));
            n1 = int32(0);
            for ya = ya0-1:ya1+1-1 %upperbound-1
                wt = s;
                if (ya == ya0-int32(1))
                    wt = (single(ya0)-ya0f)*s;
                elseif (ya == ya1)
                    wt = (ya1f-single(ya1))*s;
                end
                if (wt > wt0 && ya >= 0)
                    ybs(n) = yb;
                    yas(n) = ya;
                    wts(n) = wt;
                    n = n+1;
                    n1 = n1+1;
                    W = W+wt;
                end
            end
            if (W > 1)
                for i = 0:n1-1
                    wts(n-n1+i) = wts(n-n1+i)/W;
                end
            end
            if (n1 > bd(1))
                bd(1) = n1;
            end
            while (n1 < pad)
                ybs(n) = yb;
                yas(n) = yas(n-1);
                wts(n) = 0;
                n = n+1;
                n1 = n1+1;
            end
        end
    else
        for yb = 0:hb-1
            yaf = (single(0.5)+single(yb))*sInv - single(0.5);
            ya = int32(floor(yaf));
            wt = single(1);
            if (ya >= 0 && ya < ha-1)
                wt = 1-(yaf-single(ya));
            end
            if (ya < 0)
                ya = int32(0);
                bd(1) = bd(1)+1;
            end
            if (ya >= ha-1)
                ya = ha-1;
                bd(2) = bd(2)+1;
            end
            ybs(yb+1) = yb;
            yas(yb+1) = ya;
            wts(yb+1) = wt;
        end
    end
end %function resampleCoef

