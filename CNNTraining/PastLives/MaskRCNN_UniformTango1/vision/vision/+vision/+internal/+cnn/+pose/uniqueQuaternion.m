function q = uniqueQuaternion(q)
% Constrain quaternions to lie on the positive hemisphere to avoid the sign
% ambiguity of quaternion representations (since quaternions q and -q both
% represent the same rotation).

% Copyright 2023 The MathWorks, Inc.

    if q(1)==0 && q(2)==0 && q(3)==0
        if q(4) < 0
            q = -q;
        end
    elseif q(1)==0 && q(2)==0 && q(3)~=0
        if q(3) < 0
            q = -q;
        end
    elseif q(1)==0 && q(2)~=0
        if q(2) < 0
            q = -q;
        end
    elseif q(1) ~= 0
        if q(1) < 0
            q = -q;
        end
    end
end