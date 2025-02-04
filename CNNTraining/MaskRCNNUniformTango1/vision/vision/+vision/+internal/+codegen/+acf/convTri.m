function [J] = convTri(I, r, s) %#codegen
% Convolves an image by a 2D triangle filter (the 1D triangle filter f is
% [1:r r+1 r:-1:1]/(r+1)^2, the 2D version is simply conv2(f,f')).
%
% INPUTS
%  I      - [m n k] input k channel single image
%  r      - [3] integer filter radius (or any value between 0 and 1)
%  s      - [1] integer downsampling amount after convolving
%
% OUTPUTS
%  J      - [m x n x k] smoothed image

% Copyright 2020 The MathWorks, Inc.
%
% References
% ----------
%   Dollar, Piotr, et al. "Fast feature pyramids for object detection."
%   Pattern Analysis and Machine Intelligence, IEEE Transactions on 36.8
%   (2014): 1532-1545.

    % Check inputs
    if (nargin < 2 || isempty(r))
        r = 3;
    end
    if (nargin < 3 || isempty(s))
        s = 1;
    end
    if (isempty(I) || (r == 0 && s == 1))
        J = I;
        return;
    end

    if (r <= 1)
        J = vision.internal.codegen.acf.convTriR1(I);
    else
        J = vision.internal.codegen.acf.convTriR5(I);
    end

    if (s > 1)
        t = floor(s/2)+1;
        J = J(t:s:end-s+t, t:s:end-s+t, :);
    end
end

