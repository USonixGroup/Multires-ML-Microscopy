function outImg = rgb2luv( RGBImage) %#codegen
% SIMD friendly implementation of rgb2luv

% Copyright 2020 The MathWorks, Inc.
%
% References
% ----------
%   Dollar, Piotr, et al. "Fast feature pyramids for object detection."
%   Pattern Analysis and Machine Intelligence, IEEE Transactions on 36.8
%   (2014): 1532-1545.

    nrm = single(1.0);

    [lTable,mr,mg,mb, minu, minv,un,vn] = initRgb2luv(nrm);
    Cols = size(RGBImage,2);

    outImg = coder.nullcopy(zeros(size(RGBImage),'single'));

    for ix = 1:Cols
        rCol = RGBImage(:,ix,1);
        gCol = RGBImage(:,ix,2);
        bCol = RGBImage(:,ix,3);
        x = mr(1).*rCol + mg(1).*gCol + mb(1).*bCol;
        y = mr(2).*rCol + mg(2).*gCol + mb(2).*bCol;
        z = mr(3).*rCol + mg(3).*gCol + mb(3).*bCol;

        y1024 = floor(y*1024)+1;
        L = lTable(y1024);

        z = 1./(x + 15.*y + 3.*z + 1e-35);

        outImg(:,ix,1) = L;
        outImg(:,ix,2) = L .* ((13*4).*x.*z - 13*un) - minu;
        outImg(:,ix,3) = L .* ((13*9).*y.*z - 13*vn) - minv;
    end
end

function [lTable, mr, mg, mb, minu, minv, un, vn] = initRgb2luv(z)
    y0 = single((6.0/29)*(6.0/29)*(6.0/29));
    a = single((29.0/3)*(29.0/3)*(29.0/3));
    un = single(0.197833);
    vn = single(0.468331);

    mr = zeros(3,1,'single');
    mg = zeros(3,1,'single');
    mb = zeros(3,1,'single');

    mr(1) = 0.430574*z;
    mr(2) = 0.222015*z;
    mr(3) = 0.020183*z;
    mg(1) = 0.341550*z;
    mg(2) = 0.706655*z;
    mg(3) = 0.129553*z;
    mb(1) = 0.178325*z;
    mb(2) = 0.071330*z;
    mb(3) = 0.939180*z;

    maxi = single(1.0/270);
    minu = -88*maxi;
    minv = -134*maxi;
    lTable = coder.nullcopy(zeros(1064,1,'single'));
    for i = 0:1024
        y = (i/single(1024.0));
        if (y > y0)
            l = 116*(y.^(1.0/3.0))-16;
        else
            l = y*a;
        end
        lTable(i+1) = l*maxi;
    end
    for i = (1025+1):1064
        lTable(i) = lTable(i-1);
    end
end
