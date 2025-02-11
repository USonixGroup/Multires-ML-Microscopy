function ptCloud = pcread(filename)
 
% Copyright 2015-2023 The MathWorks, Inc.

% Validate the input
if isstring(filename)
    filename = char(filename);
end

if ~ischar(filename)
    error(message('vision:pointcloud:badFileName'));
end

% Validate the file type
idx = find(filename == '.');
if (~isempty(idx))
    extension = lower(filename(idx(end)+1:end));
else
    extension = '';
end

% Validate the file extension.
if(~(strcmpi(extension,'pcd') || strcmpi(extension,'ply')))
    error(message('vision:pointcloud:unsupportedFileExtension'));
end

% Verify that the file exists.
fid = fopen(filename, 'r');
if (fid == -1)
    if ~isempty(dir(filename))
        error(message('MATLAB:imagesci:imread:fileReadPermission', filename));
    else
        error(message('MATLAB:imagesci:imread:fileDoesNotExist', filename));
    end
else
    % File exists.  Get full filename.
    filename = fopen(fid);
    fclose(fid);
end

if( strcmpi(extension,'ply') )
    % Read properties of 'Vertex'
    elementName = 'vertex';
    requiredProperties = {'x','y','z'};
    % Alternative names are specified in a cell array within the main cell array.
    optionalProperties = {{'r','red','diffuse_red'},{'g','green', 'diffuse_green'},{'b','blue','diffuse_blue'},'nx','ny','nz','intensity'};
    properties = visionPlyRead(filename,elementName,requiredProperties,optionalProperties);

    % Get location property
    x = properties{1};
    y = properties{2};
    z = properties{3};
    if isa(x,'double') || isa(y,'double') || isa(z,'double')
        loc = [double(x), double(y), double(z)];
    else
        loc = [single(x), single(y), single(z)];
    end

    % Get color property
    r = properties{4};
    g = properties{5};
    b = properties{6};
    if isa(r,"uint16") && isa(g,"uint16") && isa(b,"uint16")
        color = [r, g, b];
    else
        color = [im2uint8(r), im2uint8(g), im2uint8(b)];
    end
    

    % Get normal property
    nx = properties{7};
    ny = properties{8};
    nz = properties{9};
    if isa(nx,'double') || isa(ny,'double') || isa(nz,'double')
        normal = [double(nx), double(ny), double(nz)];
    else
        normal = [single(nx), single(ny), single(nz)];
    end
    
    % Get intensity property
    intensity = properties{10};
    
elseif( strcmpi(extension,'pcd') )
    requiredProperties = {'x','y','z'};
    optionalProperties = {'r','g','b','normal_x','normal_y','normal_z','intensity'};
    properties = visionPcdRead(filename,requiredProperties,optionalProperties);
    
    % Get location property
    x = properties{1};
    y = properties{2};
    z = properties{3};
    % Get color property
    r = properties{4};
    g = properties{5};
    b = properties{6}; 
    % Get normal property
    nx = properties{7};
    ny = properties{8};
    nz = properties{9};
    % Get intensity property
    intensity = properties{10};
    
    [~,cols] = size(x);
    
    % Check if it is organized or unorganized point cloud
    if cols == 1 
        dim = 2;
    else
        dim = 3;
    end
    
    if isempty(x) || isempty(y) || isempty(z)
        loc = [];
    else
        if dim > 2
            x = reshapeValuesRowise(x);
            y = reshapeValuesRowise(y);
            z = reshapeValuesRowise(z);
        end
        
        if isa(x,'double') || isa(y,'double') || isa(z,'double')
            loc = cat(dim, double(x), double(y), double(z));
        else
            loc = cat(dim,single(x), single(y), single(z));
        end         
    end
    
    if isempty(r) || isempty(g) || isempty(b)
        color = [];
    else
        if dim > 2
            r = reshapeValuesRowise(r);
            g = reshapeValuesRowise(g);
            b = reshapeValuesRowise(b);
        end        
        color = cat(dim,im2uint8(r), im2uint8(g), im2uint8(b));
    end
    
    if isempty(nx) || isempty(ny) || isempty(nz)
        normal = [];
    else
        if dim > 2
            nx = reshapeValuesRowise(nx);
            ny = reshapeValuesRowise(ny);
            nz = reshapeValuesRowise(nz);
        end             
        if isa(nx,'double') || isa(ny,'double') || isa(nz,'double')
            normal = cat(dim, double(nx), double(ny), double(nz));
        else
            normal = cat(dim,single(nx), single(ny), single(nz));
        end         
    end
    
    if ~isempty(intensity)
        intensity = reshapeValuesRowise(intensity);
    end
end

ptCloud = pointCloud(loc, 'Color', color, 'Normal', normal, ...
    'Intensity', intensity);

end

function out = reshapeValuesRowise(in)
    [numRow,numCol] = size(in);    
    temp = reshape(in,[numCol,numRow]);
    out = temp';    
end
