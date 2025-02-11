%   pointTrackImpl Common implementation for pointTrack
%
%   This class is for internal use only.
%
%   This class implements the core functionality of pointTrack.
%   It is common for both simulation and codegen.

% Copyright 2021 The MathWorks, Inc.

%#codegen

classdef pointTrackImpl

    properties
        % ViewIds An M-element vector of view ids
        ViewIds;

        % Points An M-by-2 matrix of [x,y] point coordinates
        Points;

        % FeatureIndices An M-element vector of feature indices
        FeatureIndices;
    end

    methods

        function this = pointTrackImpl(varargin)
            if nargin == 0
                this.ViewIds        = zeros(1, 0, 'uint32');
                this.Points         = zeros(0, 2);
                this.FeatureIndices = zeros(1, 0, 'uint32');
            elseif nargin == 2
                viewIds             = varargin{1};
                points              = varargin{2};
                this.ViewIds        = viewIds;
                this.Points         = points;
                this.FeatureIndices = zeros(1, 0, 'uint32');
            else
                viewIds             = varargin{1};
                points              = varargin{2};
                featureIndices      = varargin{3};
                this.ViewIds        = viewIds;
                this.Points         = points;
                this.FeatureIndices = featureIndices;
            end
        end

        %------------------------------------------------------------------
        function this = set.Points(this, points)
            this.Points = ...
                vision.internal.inputValidation.checkAndConvertPoints(...
                points, 'pointTrack', 'points');
        end

        %------------------------------------------------------------------
        function this = set.ViewIds(this, viewIds)
            validateattributes(viewIds, {'numeric'}, ...
                               {'vector', 'integer', 'nonnegative', 'nonsparse'}, ...
                               'pointTrack', 'viewIds');
            viewIds = viewIds(:)';
            this.ViewIds = uint32(viewIds);
        end

        %------------------------------------------------------------------
        function this = set.FeatureIndices(this, featureIndices)
            validateattributes(featureIndices, {'numeric'}, ...
                               {'vector', 'integer', 'positive', 'nonsparse'}, ...
                               'pointTrack', 'featureIndices');
            this.FeatureIndices = uint32(featureIndices(:)');
        end

    end

end
