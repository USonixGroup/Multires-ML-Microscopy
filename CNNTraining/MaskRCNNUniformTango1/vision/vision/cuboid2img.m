function varargout = cuboid2img(cuboid, projectionMatrix, varargin)

    % Copyright 2022-2023 The MathWorks, Inc.
    
    %#codegen

    narginchk(2,4)
    nargoutchk(0,3)  

    % Parse and validate inputs.
    [cuboidpts, projectionMatrix, outputFormat, outputClass] = ...
        parseAndValidateInput(cuboid, projectionMatrix, varargin{:});       
       
    % Computation will be done in double and ouput will be cast in the
    % input format.
    cuboidpts = double(cuboidpts);
    projectionOut = double(projectionMatrix);
    
    % Should face vertices be computed.
    if nargout == 3
        faceVerticesComputed = 1;
    else
        faceVerticesComputed = 0;
    end

    % Compute projections.
    if isempty(coder.target)
        [projectedCuboids, validIdx, faceVertices] = ...
            computeProjections(cuboidpts, projectionOut, outputFormat, faceVerticesComputed);       
    else
        [projectedCuboids, validIdx, faceVertices] = ...
            computeProjectionsCG(cuboidpts, projectionOut, outputFormat, faceVerticesComputed);
    end

    varargout{1} = cast(projectedCuboids, outputClass);

    if nargout == 2
        varargout{2} = validIdx;
    elseif nargout == 3
        varargout{2} = validIdx;
        varargout{3} = cast(faceVertices, outputClass);
    end
  
        
end

%--------------------------------------------------------------------------
function cuboidCorners = cubeCorners(cuboidsIn)    
    % Compute cuboid corners for all input cuboids
    numCuboids = size(cuboidsIn, 1);
    cuboidCorners = ones([8, 4, numCuboids], 'like', cuboidsIn);
    for i=1:numCuboids
        cuboidCorners(:,1:3,i) = getCornerPoints(cuboidsIn(i,:));
    end    
end 

%--------------------------------------------------------------------------
function points = getCornerPoints(in)
        % Returns the corner points of the cuboid.
    
        dimensions = in(4:6);
        center = in(1:3);

        % Compute the Euler rotation matrix.
        rotM = vision.internal.eul.eulerAngleToRotationMatrix(deg2rad(in(9:-1:7)),'ZYX');
        

        cPoints = [ dimensions(1)/2, dimensions(2)/2, dimensions(3)/2;
                    -dimensions(1)/2, dimensions(2)/2,  dimensions(3)/2;
                    -dimensions(1)/2, -dimensions(2)/2, dimensions(3)/2;
                    dimensions(1)/2,  -dimensions(2)/2, dimensions(3)/2;
                    dimensions(1)/2,  dimensions(2)/2,  -dimensions(3)/2;
                    -dimensions(1)/2, dimensions(2)/2,  -dimensions(3)/2;
                    -dimensions(1)/2, -dimensions(2)/2, -dimensions(3)/2;
                    dimensions(1)/2,  -dimensions(2)/2, -dimensions(3)/2];

        points = (rotM*(cPoints)' + repmat(center', 1, size(cPoints, 1)))';
 end

%--------------------------------------------------------------------------
function [projectPoints, isValid, faceVertices, projectPoints2D] = ...
        applyTransform(cornerPoints, projectionMatrix, faceVerticesRequired)

    % Apply the projection matrix and compute face vertices
    %
    % ------Inputs-----
    % cornerPoints - 8-by-3-by-M matrix contining corner vertices
    % projectionMatrix - 4-by-3 projection matrix
    % faceVerticesRequired - indicates whether Face Vertices are required.
    % ------Outputs-----
    % projectPoints - 8-by-3-by-M homogeneous coordinate projection.
    % isValid - M-by-1 array indicating whether the projection lies on the image.
    % faceVertices - M-by-8-by-6 array of all face vertices.
    % projectPoints2D - 8-by-2-by-M 2D coordinates of the vertices.

    
    % Indices corresponding to face.
    sideMatrix = getFaceVertices();

    numCuboids = size(cornerPoints,3);
    projectPoints = pagemtimes(cornerPoints, projectionMatrix); 
    isValid = ones(numCuboids,1);

    % vertices in image coordinates. 
    projectPoints2D = projectPoints(:,[1,2],:)./projectPoints(:,3,:);

    faceVertices = zeros(numCuboids,8,6,'like',cornerPoints);

    for i = 1:numCuboids        
        
        % Ensure that the cuboid is in front of the camera and has at least one
        % vertex in positive image coordinates. 
        if all(projectPoints(:,3,i) < 0, 'all') || all(projectPoints(:,[1,2],i) < 0, 'all')
            isValid(i) = 0;
        end
        
        % Obtain face vertices if required.
        if(faceVerticesRequired)
            for j = 1:6
                faceVertices(i,[1 3 5 7],j) = projectPoints2D(sideMatrix(j,1:4),1,i);
                faceVertices(i,[2 4 6 8],j) = projectPoints2D(sideMatrix(j,1:4),2,i);
            end
        end
    end
    
end

%--------------------------------------------------------------------------
function [projectedCuboids, validIdx, faceVerticesOut] = ...
                computeProjections(cuboidpts, projection, outputFormat, faceVerticesComputed)
    % Computes projected cuboids depending on the format and also computes 
    % the face vertices associated to each face. 

    % Obtain cube corner points.
    cornerPoints = cubeCorners(cuboidpts);

    [projectPoints, valid, faceVertices, projectPoints2D] = ...
        applyTransform(cornerPoints, projection, faceVerticesComputed);
    
    validIdx = logical(valid);
    
    if strcmp(outputFormat, "vertices") 
        % Prune invalid cuboids.    
        projectedCuboids = projectPoints2D(:,:,validIdx);
        faceVerticesOut = faceVertices(validIdx,:,:);

    elseif strcmp(outputFormat,"rectangles")

        % Convert vertices representation to rectangles representation.
        numProjected = size(projectPoints,3);
        rectanglesOut = zeros([numProjected,8], 'like', cuboidpts);
        faceVerticesNew = zeros([numProjected,8,6], 'like', cuboidpts);

        for i=1:numProjected
            [rectanglesOut(i,:),faceVerticesNew(i,:,:)] = ...
                verticesToRectangularFormat(projectPoints(:,:,i)', faceVerticesComputed);
        end
        % Prune invalid cuboids.
        projectedCuboids = rectanglesOut(validIdx,:);
        faceVerticesOut = faceVerticesNew(validIdx,:,:);

    end

end

%--------------------------------------------------------------------------
function [projectedOut, faceVertices] = verticesToRectangularFormat(eightptsIn,...
    faceVerticesComputed)
    % Change vertices representation to rectangles format.    
    
    eightptsTemp = squeeze(eightptsIn);

    % sort the vertices according to z axis distance
    [~,sortIdx] = sort(eightptsTemp(3,:));

    % Homogeneous scaling. 
    eightpts = eightptsTemp';
    eightpts = eightpts(:,[1,2])./eightpts(:,3);

    sideMatrix = getFaceVertices();
    faceVertices = zeros(8,6, 'like', eightptsIn);
    
    
    % Compute number of face vertices corresponding to top 5 nearest.
    % 5 numbers are considered instead of 4 to break ties.
    for i=1:6
        sideMatrix(i,5) = sum(ismember(sideMatrix(i,1:4), sortIdx(1:5)));
    end
    % Compute number of face vertices corresponding to top 5 farthest.
    for i=1:6
        sideMatrix(i,6) = sum(ismember(sideMatrix(i,1:4), sortIdx(4:8)));
    end

    % Identify faces that are nearest(front) and farthest(rear) faces.
    [~,maxIdx] = max(sideMatrix(:,5));
    [~,minIdx] = max(sideMatrix(:,6));    
    front= eightpts(sideMatrix(maxIdx,1:4),:);
    back = eightpts(sideMatrix(minIdx,1:4),:);
       
    eightptsNew = zeros(8,2,'like', eightptsIn);

    % Convert front face to rectangle.
    xmin = min(front(1:4,1));
    xmax = max(front(1:4,1));
    ymin = min(front(1:4,2));
    ymax = max(front(1:4,2));
    frontbox = [xmin,ymin,xmax-xmin,ymax-ymin];
    
    if (faceVerticesComputed)
        % Store vertices of front face.
        idx = sideMatrix(maxIdx,1:4);
        eightptsNew(idx(1),:) = [xmin ymin];
        eightptsNew(idx(2),:) = [xmax ymin];
        eightptsNew(idx(3),:) = [xmax ymax];
        eightptsNew(idx(4),:) = [xmin ymax];
    end

    % Convert rear face to rectangle.
    xmin = min(back(1:4,1));
    xmax = max(back(1:4,1));
    ymin = min(back(1:4,2));
    ymax = max(back(1:4,2));
    backbox = [xmin,ymin,xmax-xmin,ymax-ymin];

    if(faceVerticesComputed)
        % Store vertices of rear face.
        idx = sideMatrix(minIdx,1:4);
        eightptsNew(idx(1),:) = [xmin ymin];
        eightptsNew(idx(2),:) = [xmax ymin];
        eightptsNew(idx(3),:) = [xmax ymax];
        eightptsNew(idx(4),:) = [xmin ymax];
    end

    % Combine front and rear face in rectangles projected cuboid format.
    projectedOut = [frontbox, backbox];
    
    if (faceVerticesComputed)
        % Store face vertices x and y.       
        for i=1:6
            faceVertices([1 3 5 7],i) = eightptsNew(sideMatrix(i,1:4),1);
            faceVertices([2 4 6 8],i) = eightptsNew(sideMatrix(i,1:4),2);
        end    
    end
    
end
%--------------------------------------------------------------------------
function [projectedCuboids, validIdx, faceVerticesOut] = ...
                computeProjectionsCG(cuboidpts, projection, outputFormat, faceVerticesComputed)
    % Computes projected cuboids and also computes 
    % the face vertices associated to each face. <Code Generation path>
    
    % Obtain all face vertex ids.
    sideMatrix = getFaceVertices();
    
    % Preallocations.
    cornerPoints = zeros(8,4,size(cuboidpts,1),'like',cuboidpts);
    cornerPointOut = zeros(8,2,size(cuboidpts,1),'like',cuboidpts);
    rectanglesOut = zeros(size(cuboidpts,1),8,'like',cuboidpts); 
    facesOut = zeros(size(cuboidpts,1),8,6,'like',cuboidpts);
    idx = ones(size(cuboidpts,1),1);

    % Loop over input cuboids and convert each one.
    for i=1:size(cuboidpts,1)

        currentCuboidCorners = getCornerPoints(cuboidpts(i,:));
        cornerPoints(:,1:3,i) = currentCuboidCorners;
        cornerPoints(:,4,:) = 1;
        currentCorner = squeeze(cornerPoints(:,:,i));
        projectedVertices = projection' * permute(currentCorner,[2,1]);

        % Condition to ignore the cuboids.
        if all(projectedVertices(3,:) < 0) || all(projectedVertices([1,2],:) < 0,'all')
            idx(i) = 0;
            continue;        
        end
        
        cornerPts = permute(projectedVertices,[2,1]);

        % Homogeneous rescaling.
        cornerPointOut(:,:,i) = cornerPts(:,[1,2])./cornerPts(:,3);
        
        % Compute output and facevertices depending on OutputFormat.       
        if strcmp(outputFormat,"vertices")                
        
            faceVertices = zeros(8,6, 'like', cuboidpts);    
            
            if(faceVerticesComputed)
                % Store face vertices x and y.
                for j=1:6
                    faceVertices([1 3 5 7],j) = cornerPointOut(sideMatrix(j,1:4),1,i);
                    faceVertices([2 4 6 8],j) = cornerPointOut(sideMatrix(j,1:4),2,i);
                end
                facesOut(i,:,:) = faceVertices;
            end
        
        else          
            [projectedCuboids,faceVertices] = ...
                verticesToRectangularFormat(projectedVertices, faceVerticesComputed);
            rectanglesOut(i,:) = projectedCuboids;
            facesOut(i,:,:) = faceVertices;
        end

    end

    % validIdx will be used to prune invalid cuboids.
    validIdx = logical(idx);

    if strcmp(outputFormat, "vertices")
        projectedCuboids = cornerPointOut(:,:,validIdx);        
    else
        projectedCuboids = rectanglesOut(validIdx,:);
    end
    faceVerticesOut = facesOut(validIdx,:,:);


end

%--------------------------------------------------------------------------
function [cuboid, projectionMatrixOut, outputFormat, outputClass] = ...
        parseAndValidateInput(cuboid, projectionMatrix, varargin)
    % Validate and parse inputs.

    validateattributes(cuboid, {'double', 'single'}, ...
    {'real', 'nonsparse', 'nonempty', 'finite', '2d', 'ncols', 9}, mfilename, 'cuboid');

    validateattributes(projectionMatrix, {'double','single'}, ...
    {'finite', '2d', 'real', 'nonsparse', 'size', [3,4]}, mfilename, 'projectionMatrix');

    projectionMatrixOut = projectionMatrix';
    
    % Validate dimensions of the cuboid. (>=0)
    validateattributes(cuboid(:,4), {'single', 'double'}, ...
                       {'nonnegative'}, mfilename, 'xlen');
    validateattributes(cuboid(:,5), {'single', 'double'}, ...
                       {'nonnegative'}, mfilename, 'ylen');
    validateattributes(cuboid(:,6), {'single', 'double'}, ...
                       {'nonnegative'}, mfilename, 'zlen');

    
    % Store the input format.
    if isa(cuboid, 'double')
        outputClass = 'double';
    else
        outputClass = 'single';
    end

    % Parse and validate the OutputFormat NVP.
    if isempty(coder.target)
        p = inputParser;
        addOptional(p,'OutputFormat', 'vertices');
        parse(p,varargin{:});
        outputFormat = p.Results.OutputFormat;
    else
        defaults = struct('OutputFormat', 'vertices');
        pvPairs = struct('OutputFormat',  uint32(0));
        properties =  struct( ...
            'CaseSensitivity', false, ...
            'StructExpand',    true, ...
            'PartialMatching', true);
        optarg = coder.internal.parseParameterInputs(pvPairs, properties, varargin{:});
        outputFormat = coder.internal.getParameterValue(optarg.OutputFormat, defaults.OutputFormat, varargin{:});
        
    end
    validateattributes(outputFormat, {'char','string'},{'scalartext'});
    validatestring(outputFormat, {'vertices', 'rectangles'}, 'cuboid2img', 'OutputFormat');

end
%--------------------------------------------------------------------------
function sideMatrix = getFaceVertices()
    % Returns matrix containing face vertex ids.
    % Output is 6-by-6, where columns indicate faces and first four column 
    % contain indices of vertices making up the face. Column 5 and 6 are
    % empty and will be used by consumer functions for statistics.
    %
    %                   z
    %                     |
    %                     |
    %                     |_3_____2                   
    %                    /|      /|                   
    %                   / |     / |                   
    %                  /  |7__ / _|6__ __ __ y                  
    %                 /   /   /   /                  
    %                /   /   /   /                  
    %               4__ /__1/   /                  
    %               |  /    |  /                  
    %               | /     | /
    %               |/______|/           
    %               / 8       5
    %              /
    %             x
    %  

    % Vertex mapping of faces.
    frontMap = [1 4 8 5];
    backMap = [3 2 6 7];
    left = [2 1 5 6];
    right = [4 3 7 8];
    top = [2 3 4 1];
    bottom = [7 6 5 8];

    sideMatrix = zeros(6,6);

    sideMatrix(:,1:4)=[frontMap; backMap; left; right; top; bottom];
    
end

