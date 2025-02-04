function rigidtform3dObjs = rigid3dConvert(rigid3dObjs)
% rigid3dConvert takes in rigid3d objects and converts them to rigidtform3d
%   objects. rigid3dObjs can be either a single rigid3d object or a table
%   filled with rigid3d objects.

% Copyright 2022 The MathWorks, Inc.

    rigidtform3dObjs = repelem(rigidtform3d, height(rigid3dObjs), 1);
    for i = 1:height(rigid3dObjs)
        rigidtform3dObjs(i, 1).A = rigid3dObjs(i, 1).T';
    end
end