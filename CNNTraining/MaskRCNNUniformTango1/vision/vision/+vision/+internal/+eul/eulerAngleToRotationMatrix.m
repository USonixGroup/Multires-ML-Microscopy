function R = eulerAngleToRotationMatrix(eul, seq)
%eulerAngleToRotationMatrix Convert Euler angles to rotation matrix
%   R = eulerAngleToRotationMatrix(eul, seq) converts a vector of Euler
%   angles [phix, phiy, phiz] to a 3D rotation matrix, R. Rotation angles
%   are in radians. The rotation sequence is 'XYZ' by default, where the
%   order of rotation angles is X Axis Rotation, Y Axis Rotation, and Z
%   Axis Rotation, i.e., R = Rx * Ry * Rz. 'ZYX' rotation order is also
%   supported.

%   Copyright 2017-2019 The MathWorks, Inc.

%#codegen

% Only supported euler angle sequences are XYZ and ZYX.
isXYZ = nargin<2 || strcmpi(seq, 'XYZ');

R = zeros(3,3,'like',eul);

ct = cos(eul);
st = sin(eul);

if isXYZ
    %     The rotation matrix R can be constructed as follows by
    %     ct = [cx cy cz] and st = [sx sy sz]
    %
    %     R = [            cy*cz,           -cy*sz,     sy]
    %         [ cx*sz + cz*sx*sy, cx*cz - sx*sy*sz, -cy*sx]
    %         [ sx*sz - cx*cz*sy, cz*sx + cx*sy*sz,  cx*cy]
    %       = Rx(tx) * Ry(ty) * Rz(tz)

    R(1,1) = ct(2)*ct(3);
    R(1,2) = -ct(2)*st(3);
    R(1,3) = st(2);
    R(2,1) = ct(1)*st(3) + ct(3)*st(1)*st(2);
    R(2,2) = ct(1)*ct(3) - st(1)*st(2)*st(3);
    R(2,3) = -ct(2)*st(1);
    R(3,1) = st(1)*st(3) - ct(1)*ct(3)*st(2);
    R(3,2) = ct(3)*st(1) + ct(1)*st(2)*st(3);
    R(3,3) = ct(1)*ct(2);
else
    %     The rotation matrix R can be constructed as follows by
    %     ct = [cz cy cx] and st = [sy sy sx]
    %
    %     R = [  cy*cz   sy*sx*cz-sz*cx    sy*cx*cz+sz*sx
    %            cy*sz   sy*sx*sz+cz*cx    sy*cx*sz-cz*sx
    %              -sy            cy*sx             cy*cx]
    %       = Rz(tz) * Ry(ty) * Rx(tx)

    R(1,1) = ct(2).*ct(1);
    R(1,2) = st(3).*st(2).*ct(1) - ct(3).*st(1);
    R(1,3) = ct(3).*st(2).*ct(1) + st(3).*st(1);
    R(2,1) = ct(2).*st(1);
    R(2,2) = st(3).*st(2).*st(1) + ct(3).*ct(1);
    R(2,3) = ct(3).*st(2).*st(1) - st(3).*ct(1);
    R(3,1) = -st(2);
    R(3,2) = st(3).*ct(2);
    R(3,3) = ct(3).*ct(2);
end
