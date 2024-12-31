function quaternion = rotationToQuaternion(R)
% rotationToQuaternion Convert orthogonal rotation matrices to unit quaternions
% 
% quaternion is a 4-by-M vector
% R is a 3-by-3-by-M orthogonal matrix of corresponding rotation matrix
%
% Note
% ----
% R is rotation of vectors anti-clockwise in a right-handed system by pre-multiplication

% Copyright 2014-2021 The MathWorks, Inc.

%#codegen

% References
% ----------
% http://en.wikipedia.org/wiki/Rotation_matrix#Quaternion

Qxx = squeeze( R(1, 1, :) );
Qxy = squeeze( R(1, 2, :) );
Qxz = squeeze( R(1, 3, :) );
Qyx = squeeze( R(2, 1, :) );
Qyy = squeeze( R(2, 2, :) );
Qyz = squeeze( R(2, 3, :) );
Qzx = squeeze( R(3, 1, :) );
Qzy = squeeze( R(3, 2, :) );
Qzz = squeeze( R(3, 3, :) );

t = Qxx + Qyy + Qzz;

num = size(R, 3);

r = zeros(num, 1,'like', R);
w = zeros(num, 1,'like', R);
x = zeros(num, 1,'like', R);
y = zeros(num, 1,'like', R);
z = zeros(num, 1,'like', R);

maxv = max(Qxx, max(Qyy,Qzz));

positivet = t>=0;
qxxismax  = ~positivet & maxv==Qxx;
qyyismax  = ~positivet & maxv==Qyy;
qzzismax  = ~positivet & maxv==Qzz;

% Compute r
r(positivet) = sqrt(1 + t(positivet));
r(qxxismax)  = sqrt(1 + Qxx(qxxismax) - Qyy(qxxismax) - Qzz(qxxismax));
r(qyyismax)  = sqrt(1 + Qyy(qyyismax) - Qxx(qyyismax) - Qzz(qyyismax));
r(qzzismax)  = sqrt(1 + Qzz(qzzismax) - Qxx(qzzismax) - Qyy(qzzismax));

% Compute s
s = 0.5 ./ r;

% Compute w
w(positivet) = 0.5 * r(positivet);
w(qxxismax) = (Qzy(qxxismax) - Qyz(qxxismax)) .* s(qxxismax);
w(qyyismax) = (Qxz(qyyismax) - Qzx(qyyismax)) .* s(qyyismax);
w(qzzismax) = (Qyx(qzzismax) - Qxy(qzzismax)) .* s(qzzismax);

% Compute x
x(positivet) = (Qzy(positivet) - Qyz(positivet)) .* s(positivet);
x(qxxismax)  = 0.5 * r(qxxismax);
x(qyyismax)  = (Qyx(qyyismax) + Qxy(qyyismax)) .* s(qyyismax);
x(qzzismax)  = (Qxz(qzzismax) + Qzx(qzzismax)) .* s(qzzismax);

% Compute y
y(positivet) = (Qxz(positivet) - Qzx(positivet)) .* s(positivet);
y(qxxismax)  = (Qyx(qxxismax) + Qxy(qxxismax)) .* s(qxxismax);
y(qyyismax)  = 0.5 * r(qyyismax);
y(qzzismax)  = (Qzy(qzzismax) + Qyz(qzzismax)) .* s(qzzismax);

% Compute z
z(positivet) = (Qyx(positivet) - Qxy(positivet)) .* s(positivet);
z(qxxismax)  = (Qxz(qxxismax) + Qzx(qxxismax)) .* s(qxxismax);
z(qyyismax)  = (Qzy(qyyismax) + Qyz(qyyismax)) .* s(qyyismax);
z(qzzismax)  = 0.5 * r(qzzismax);

quaternion = [w.';x.'; y.'; z.'];
