classdef sphereModel < vision.internal.EnforceScalarHandle
    
    % Copyright 2015-2023 The MathWorks, Inc.
    
    %#codegen
    
    properties (GetAccess = public, SetAccess = private)
        Parameters;
    end
    
    properties(Dependent)
        Center;
        Radius;
    end
    
    methods
        %==================================================================
        % Constructor
        %==================================================================
        % model = sphereModel(params);
        function this = sphereModel(params)
            if nargin == 0
                this.Parameters = [0 0 0 0];
            else
                
                % Validate the inputs
                validateattributes(params, {'single', 'double'}, ...
                    {'real', 'nonsparse', 'finite', 'vector', 'numel', 4}, mfilename, 'params');
                % Validate radius attributes only when params is not all zeros
                if ~all(params(:) == 0)
                    validateattributes(params(4), {'single', 'double'}, ...
                        {'scalar','positive'}, mfilename, 'Radius, params(4),');
                end
                this.Parameters = params;
            end
        end
        
        %==================================================================
        % Dependent properties
        %==================================================================
        function center = get.Center(this)
            center = this.Parameters(1:3);
        end
        
        function radius = get.Radius(this)
            radius = this.Parameters(4);
        end

        %==================================================================
        % Find points in sphere
        %==================================================================
        function indices = findPointsInModel(this, ptCloud)
            arguments
                this
                ptCloud(1,1) pointCloud
            end
            
            indices = findNeighborsInRadius(ptCloud, this.Center, this.Radius);
        end
        
        %==================================================================
        % Plot the sphere
        %==================================================================
        function H = plot(this, varargin)
            [currentAxes, color] = validateAndParseInputs(mfilename, varargin{:});
            
            [X, Y, Z] = sphere(30);
            X = this.Radius*X + this.Parameters(1);
            Y = this.Radius*Y + this.Parameters(2);
            Z = this.Radius*Z + this.Parameters(3);
            handle = surf(currentAxes,X,Y,Z, 'FaceColor', color);

            if nargout > 0
                H = handle;
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