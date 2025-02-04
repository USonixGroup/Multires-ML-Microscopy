function outputType = validatePoseTableRigid3d(poses, funcName, argName)
% validatePoseTableRigid3d Function to validate rigidtform3d or rigid3d pose table.

% Copyright 2019-2022 The MathWorks, Inc.

% Validate the table
validateattributes(poses, {'table'},{'size',[NaN 2],'nonempty'}, funcName, argName);

% Check columns
propertyNames = poses.Properties.VariableNames;
if ~all(ismember({'ViewId', 'AbsolutePose'}, propertyNames))
    error(message('vision:table:missingRequiredColumns', 'cameraPoses', ...
        ['ViewId', ', ', 'AbsolutePose']));
end

% Check the values
validateViewIds(poses.ViewId, funcName);
validateRigid3dPoses(poses.AbsolutePose, funcName);

outputType = class(poses.AbsolutePose(1).T);
end

%--------------------------------------------------------------------------
function validateViewIds(viewIds, funcName)
validateattributes(viewIds, {'uint32'}, {'nonsparse', 'vector', ...
    'integer', 'positive', 'real'}, funcName, 'ViewIds');
end

%--------------------------------------------------------------------------
function validateRigid3dPoses(tform, funcName)
validateattributes(tform, {'rigidtform3d','rigid3d'}, {'vector'},...
    funcName, 'AbsolutePose');
end
