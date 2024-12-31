classdef cylinderModel < vision.internal.EnforceScalarHandle

    % Copyright 2015-2023 The MathWorks, Inc.

    %#codegen

    properties (GetAccess = public, SetAccess = private)
        Parameters;
    end

    properties(Dependent)
        Center;
        Orientation;
        Height;
        Radius;
    end

    methods
        %==================================================================
        % Constructor
        %==================================================================
        % model = cylinderModel(params);
        function this = cylinderModel(params)
            if nargin == 0
                this.Parameters = zeros(1,7);
            else
                % Validate the inputs
                validateattributes(params, {'single', 'double'}, ...
                    {'real', 'nonsparse', 'finite', 'vector', 'numel', 7}, mfilename, 'params');
                % Validate radius attributes only when params is not all zeros
                if ~all(params(:) == 0)
                    validateattributes(params(7), {'single', 'double'}, ...
                        {'scalar','positive'}, mfilename, 'Radius, params(7),');
                end
                this.Parameters = params;
            end
        end

        %==================================================================
        % Dependent properties
        %==================================================================
        function center = get.Center(this)
            center = (this.Parameters(1:3)+this.Parameters(4:6))/2;
        end

        function orientation = get.Orientation(this)
            orientation = this.Parameters(4:6)-this.Parameters(1:3);
        end

        function height = get.Height(this)
            p1 = this.Parameters(1:3);
            p2 = this.Parameters(4:6);
            height = norm(p1 - p2);
        end

        function radius = get.Radius(this)
            radius = this.Parameters(7);
        end

        %==================================================================
        % Find points in cylinder
        %==================================================================
        function indices = findPointsInModel(this, ptCloud)
            arguments
                this
                ptCloud(1,1) pointCloud
            end

            R = findCylinderRotation(this);

            % Align the cylinder to the z-axis.
            tform = rigidtform3d(R, [0 0 0]);
            center = transformPointsForward(tform, this.Center);

            % Apply the same transformation to the point cloud.
            ptCloudTformed = pctransform(ptCloud, tform);

            % Get the z-limits of the cylinder.
            minZ = center(3) - this.Height/2;
            maxZ = center(3) + this.Height/2;

            % Find the points in the aligned cylinder.
            if ismatrix(ptCloudTformed.Location)
                logicalIdx = ptCloudTformed.Location(:,3) >= minZ & ...
                    ptCloudTformed.Location(:,3) <= maxZ & ...
                    vecnorm(center(1:2) - ptCloudTformed.Location(:,1:2), 2, 2) <= this.Radius;
            else % Point cloud is organized.
                logicalIdx = ptCloudTformed.Location(:,:,3) >= minZ & ...
                    ptCloudTformed.Location(:,:,3) <= maxZ & ...
                    (ptCloudTformed.Location(:,:,1) - center(1)).^2 + ...
                    (ptCloudTformed.Location(:,:,2) - center(2)).^2 <= this.Radius.^2;
            end

            indices = uint32(find(logicalIdx));
        end

        %==================================================================
        % Plot a cylinder
        %==================================================================
        function H = plot(this, varargin)
            [currentAxes, color] = validateAndParseInputs(mfilename, varargin{:});

            [X, Y, Z] = cylinder(this.Radius, 30);
            [X, Y, Z] = this.transformCylinder(X, Y, Z);

            handle = surf(currentAxes, X, Y, Z, 'FaceColor', color);
            if nargout > 0
                H = handle;
            end
        end
    end

    methods (Access = protected)
        %==================================================================
        % helper function to transform the data
        %==================================================================
        function [X, Y, Z] = transformCylinder(this, X, Y, Z)
            h = this.Height;
            % Rescale the height.
            Z(2, :) = Z(2, :) * h;

            % Get the cylinder rotation.
            R = findCylinderRotation(this);

            % Get the cylinder translation.
            translation = this.Parameters(1:3);
            if iscolumn(translation)
                translation = translation';
            end
            
            % Translate and rotate the points to the desired axis
            % direction.
            XYZ = bsxfun(@plus, [X(:), Y(:), Z(:)] * R, translation);

            X = reshape(XYZ(:, 1), 2, []);
            Y = reshape(XYZ(:, 2), 2, []);
            Z = reshape(XYZ(:, 3), 2, []);
        end

        %==================================================================
        % helper function to find the rotation that aligns the cylinder to
        % the z-axis
        %==================================================================
        function R = findCylinderRotation(this)
            % Define the orientation vector for the z-axis.
            a = cast([0 0 1], 'like', this.Parameters);

            % Get the orientation vector for the cylinder.
            h = this.Height;
            if h == 0
                b = a;
            else
                b = (this.Parameters(4:6) - this.Parameters(1:3)) / h;
            end

            % Get the axis of rotation.
            v = cross(a, b);

            % Find the rotation matrix that aligns the cylinder to the
            % z-axis.
            s = dot(v, v);
            if abs(s) > eps(class(s))
                Vx = [ 0,  -v(3),  v(2); ...
                     v(3),    0,  -v(1); ...
                    -v(2),  v(1),    0];
                R = transpose(eye(3) + Vx + Vx*Vx*(1-dot(a, b))/s);
            else % The cylinder is already aligned to the z-axis.
                R = eye(3, 'like', this.Parameters);
            end
        end
    end
end

%==========================================================================
function [ax, color] = validateAndParseInputs(filename, varargin)
% Parse the PV-pairs

% Setup parser
parser = inputParser;
parser.CaseSensitive = false;
parser.FunctionName  = filename;
parser.addParameter('Parent', [], ...
    @vision.internal.inputValidation.validateAxesHandle);

% Adding color parameter for edge and face.
% Default color given here is Orange500 which is #f57729.
defaultcolor = [245, 119, 41]./255;
parser.addParameter('Color', defaultcolor);

parser.parse(varargin{:});
ax = parser.Results.Parent;
color = vision.internal.inputValidation.validateColor(parser.Results.Color,'Color');

% Plot to the specified axis, or create a new one
if isempty(ax)
    ax = newplot;
end
end
