function ptCloudOut = pctransform(ptCloudIn, tform)

% Copyright 2014-2023 The MathWorks, Inc.

%#codegen

% Validate the first argument
validateattributes(ptCloudIn, {'pointCloud'}, {'scalar'}, 'pctransform', 'ptCloudIn');

% Validate the second argument
validateattributes(tform, {'affinetform3d', 'rigidtform3d', 'simtform3d', 'affine3d',...
    'rigid3d', 'single', 'double'}, {'nonempty', 'nonsparse'}, 'pctransform', '', 2);

coder.gpu.kernelfun;
isOrganized = ndims(ptCloudIn.Location) == 3;

% Check if second argument is numeric
if(isnumeric(tform))
    % Validate the second argument for point displacements
    validateattributes(tform, { 'single', 'double' }, {'real', ...
        'size', size(ptCloudIn.Location)}, 'pctransform', 'D');
    
    if ~isGpuCodegen
        % Add point displacements to Location
        loc = ptCloudIn.Location + cast(tform, 'like', ptCloudIn.Location);
        nv = cast([], 'like', ptCloudIn.Location);
        if ~isempty(ptCloudIn.Normal)
            % Recompute normals
            nv = surfaceNormalImpl(pointCloud(loc), 6);
        end
    else
        % GPU implementation to compute normals.
        inpLocations = ptCloudIn.Location;
        isRigidTForm = 0;
        if ~isempty(ptCloudIn.Normal)
            inpNormals = ptCloudIn.Normal;
        else
            if ~isOrganized
                inpNormals = zeros(0,3,'like',inpLocations);
            else
                inpNormals = zeros(0,0,3,'like',inpLocations);
            end
        end
        
        [loc,nv] = vision.internal.codegen.gpu.pctransformGpuImpl(...
            inpLocations,inpNormals,tform,isRigidTForm);
    end
    % Create output point cloud object.
    ptCloudOut = pointCloud(loc, 'Color', ptCloudIn.Color, 'Normal', nv, ...
        'Intensity', ptCloudIn.Intensity);
else
    % Validate the second argument for affinetform3d, rigidtform3d, simtform3d, affine3d, or rigid3d
    validateattributes(tform, {'rigidtform3d', 'affinetform3d', 'simtform3d', 'rigid3d', 'affine3d'},...
        {'scalar'}, 'pctransform', 'tform');
    
    T = cast(tform.T, 'like', ptCloudIn.Location);
    
    if ~isGpuCodegen
        ptCloudOut = vision.internal.pc.transform(ptCloudIn, T);  
    else
        % GPU implementation for pctransform with affinetform3d/rigidtform3d,
        % affine3d/rigid3d, or simtform3d transformation input
        inpLocations = ptCloudIn.Location;
        isRigidTForm = images.internal.coder.gpu.rigid3d.isRigidTransform(T);
        if ~isempty(ptCloudIn.Normal)
            inpNormals = ptCloudIn.Normal;
        else
            if ~isOrganized
                inpNormals = zeros(0,3,'like',inpLocations);
            else
                inpNormals = zeros(0,0,3,'like',inpLocations);
            end
        end
        
        [outLocations,outNormals] = vision.internal.codegen.gpu.pctransformGpuImpl(...
            inpLocations,inpNormals,T,isRigidTForm);
        
        ptCloudOut = pointCloud(outLocations, 'Color', ptCloudIn.Color, 'Normal', outNormals, ...
            'Intensity', ptCloudIn.Intensity);
    end
end

end

function flag = isGpuCodegen()
%#codegen
    flag = coder.gpu.internal.isGpuEnabled;
end
