classdef planeModel < vision.internal.EnforceScalarHandle
    
    % Copyright 2015-2023 The MathWorks, Inc.
    
    %#codegen
    
    properties (GetAccess = public, SetAccess = private)
        Parameters;
    end
    
    properties(Dependent)
        Normal;
    end
    
    methods
        %==================================================================
        % Constructor
        %==================================================================
        % model = planeModel(params);
        function this = planeModel(params)
            if nargin == 0
                this.Parameters = [0 0 0 0];
            else
                % Validate the inputs
                validateattributes(params, {'single', 'double'}, ...
                    {'real', 'nonsparse', 'finite', 'vector', 'numel', 4}, mfilename, 'params');
                % Validate normal attributes only when params is not all zeros
                if ~all(params(:) == 0)
                validateattributes(any(params(1:3)), {'logical'}, ...
                    {'nonzero'}, mfilename, 'Normal, params(1:3),');
                end
                
                this.Parameters = params;
            end
        end
        
        %==================================================================
        % Dependent properties
        %==================================================================
        function normal = get.Normal(this)
            normal = this.Parameters(1:3);
        end
        
        %==================================================================
        % Plot the plane
        %==================================================================
        function H = plot(this, varargin)
            [color, currentAxes] = validateAndParseInputs(mfilename, varargin{:});
            
            % Compute the polygon vertices from the intersection of the
            % plane with the axis limits
            [X, Y, Z] = getPatchPointsFromAxisLimits(this.Parameters, ...
                currentAxes.XLim, currentAxes.YLim, currentAxes.ZLim);
            
            if numel(X) < 3
                % There should be at least 3 points to be a valid polygon
                warning(message('vision:pointcloud:outsideLimits'));
            end
            
            % Make sure the patch does not change the limits of axes
            xlim = currentAxes.XLim;
            ylim = currentAxes.YLim;
            zlim = currentAxes.ZLim;
            % Create a patch object
            handle = patch(X, Y, Z, color, 'Parent', currentAxes);
            currentAxes.XLim = xlim;
            currentAxes.YLim = ylim;
            currentAxes.ZLim = zlim;
            
            if nargout > 0
                H = handle;
            end
        end
        
        %==================================================================
        % Find rotation 
        %==================================================================
        function tform = normalRotation(this, referenceVector)  
            
            validateattributes(referenceVector, {'double',...
            'single'}, {'real', 'nonsparse',...
                'finite', 'nonnan', 'vector', 'numel', 3}, 'normalRotation', 'referenceVector');
            
            normalVector = this.Normal;
            
            % Calculate transformation matrix by calculating the axis-angle for
            % rotation
            rotAngle = atan2(norm(cross(normalVector, referenceVector)),...
                dot(normalVector, referenceVector));

            % To calculate the rotation of vector A onto B, the axis of
            % rotation is given by BxA
            rotationAxis = cross(referenceVector, normalVector);

            rotationAxis = rotationAxis ./ norm(rotationAxis);

            rotMatrix = rotationVectorToMatrix(rotationAxis * rotAngle);
            tform = rigidtform3d(rotMatrix, [0 0 0]);
        end
    end
end

%==========================================================================
function [C, ax] = validateAndParseInputs(filename, varargin)
% Parse the PV-pairs

% Setup parser
parser = inputParser;
parser.CaseSensitive = false;
parser.FunctionName  = filename;

% Adding color parameter for edge and face.
% Default color given here is Orange500 which is #f57729.
defaultcolor = [245, 119, 41]./255;
parser.addParameter('Color', defaultcolor);

parser.addParameter('Parent', [], @vision.internal.inputValidation.validateAxesHandle);

parser.parse(varargin{:});
C  = vision.internal.inputValidation.validateColor(parser.Results.Color,'Color');
ax = parser.Results.Parent;

% Plot to the specified axis, or create a new one
if isempty(ax)
    ax = newplot;
end

end

%==========================================================================
% Compute the corner points of the polygon
%==========================================================================
function [X, Y, Z] = getPatchPointsFromAxisLimits(params, xlim, ylim, zlim)
classToUse = class(params);
xyz = zeros(12, 3, classToUse);
xmin = xlim(1);
xmax = xlim(2);
ymin = ylim(1);
ymax = ylim(2);
zmin = zlim(1);
zmax = zlim(2);

count = 0;
if (abs(params(1)) > eps(classToUse))
    % x, ymin, zmin
    x = (-params(2)*ymin-params(3)*zmin-params(4))/params(1);
    if (x >= xmin && x <= xmax)
        count = count + 1;
        xyz(count, :) = [x; ymin; zmin];
    end
    
    % x, ymin, zmax
    x = (-params(2)*ymin-params(3)*zmax-params(4))/params(1);
    if (x >= xmin && x <= xmax)
        count = count + 1;
        xyz(count, :) = [x; ymin; zmax];
    end
    
    % x, ymax, zmin
    x = (-params(2)*ymax-params(3)*zmin-params(4))/params(1);
    if (x >= xmin && x <= xmax)
        count = count + 1;
        xyz(count, :) = [x; ymax; zmin];
    end
    
    % x, ymax, zmax
    x = (-params(2)*ymax-params(3)*zmax-params(4))/params(1);
    if (x >= xmin && x <= xmax)
        count = count + 1;
        xyz(count, :) = [x; ymax; zmax];
    end
end

if (abs(params(2)) > eps(classToUse))
    % xmin, y, zmin
    y = (-params(1)*xmin-params(3)*zmin-params(4))/params(2);
    if (y >= ymin && y <= ymax)
        count = count + 1;
        xyz(count, :) = [xmin; y; zmin];
    end
    
    % xmin, y, zmax
    y = (-params(1)*xmin-params(3)*zmax-params(4))/params(2);
    if (y >= ymin && y <= ymax)
        count = count + 1;
        xyz(count, :) = [xmin; y; zmax];
    end
    
    % xmax, y, zmin
    y = (-params(1)*xmax-params(3)*zmin-params(4))/params(2);
    if (y >= ymin && y <= ymax)
        count = count + 1;
        xyz(count, :) = [xmax; y; zmin];
    end
    
    % xmax, y, zmax
    y = (-params(1)*xmax-params(3)*zmax-params(4))/params(2);
    if (y >= ymin && y <= ymax)
        count = count + 1;
        xyz(count, :) = [xmax; y; zmax];
    end
end

if (abs(params(3)) > eps(classToUse))
    % xmin, ymin, z
    z = (-params(1)*xmin-params(2)*ymin-params(4))/params(3);
    if (z >= zmin && z <= zmax)
        count = count + 1;
        xyz(count, :) = [xmin; ymin; z];
    end
    
    % xmin, ymax, z
    z = (-params(1)*xmin-params(2)*ymax-params(4))/params(3);
    if (z >= zmin && z <= zmax)
        count = count + 1;
        xyz(count, :) = [xmin; ymax; z];
    end
    
    % xmax, ymin, z
    z = (-params(1)*xmax-params(2)*ymin-params(4))/params(3);
    if (z >= zmin && z <= zmax)
        count = count + 1;
        xyz(count, :) = [xmax; ymin; z];
    end
    
    % xmax, ymax, z
    z = (-params(1)*xmax-params(2)*ymax-params(4))/params(3);
    if (z >= zmin && z <= zmax)
        count = count + 1;
        xyz(count, :) = [xmax; ymax; z];
    end
end

xyz = xyz(1:count, :);
% Remove redundant corners
xyz = unique(xyz, 'rows');
if size(xyz, 1) >= 3
    % Reorder the points so they form a proper polygon
    xyz = reorderPoints(xyz);
end

X = xyz(:, 1);
Y = xyz(:, 2);
Z = xyz(:, 3);
end

%==========================================================================
% Reorder the points so they follow the sequential order of a polygon
%==========================================================================
function xyz = reorderPoints(xyz)
% Re-arrange the order of the vertices to make sure the polygon is not
% self-intersecting
angle = zeros(size(xyz,1), 1, 'like', xyz);
p = mean(xyz, 1);
% Take the line from the center to the first vertex as a reference
p0 = xyz(1, :) - p;
ref = [];
% Compute the angle between the reference and the line from the center
% to each vertex
for k = 2:size(xyz, 1)
    pk = xyz(k, :) - p;
    dp = dot(p0, pk);
    cp = cross(p0, pk);
    a = atan2(norm(cp), dp);
    if k == 2
        ref = cp;
    else
        if dot(cp, ref) < 0
            a = 2 * pi - a;
        end
    end
    angle(k) = a;
end

% Re-arrange the vertices according to its angle
[~, IND] = sort(angle);
xyz = xyz(IND, :);
end