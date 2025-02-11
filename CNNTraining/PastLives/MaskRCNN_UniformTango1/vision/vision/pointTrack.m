%pointTrack Object for storing matching points from multiple views
%  You can use this object to store matching 2-D points from multiple
%  views. You can create a pointTrack object using the findTracks method
%  of imageviewset class.
%
%  track = pointTrack(viewIds, points) returns a pointTrack object.
%  viewIds is an M-element vector containing the ids of the views
%  containing the points. points is an M-by-2 matrix containing the [x,y]
%  coordinates of the image points.
%
%  track = pointTrack(viewIds, points, featureIndices) optionally specifies
%  the index of feature associated with each point, featureIndices, as an
%  M-by-1 vector.
%
%  pointTrack properties:
%
%  ViewIds         - an M-element vector of view ids
%  Points          - an M-by-2 matrix of x-y point coordinates
%  FeatureIndices  - an M-by-1 vector of feature indices
%
%  Example: Create a point track across two images
%  -----------------------------------------------
%  % Load images
%  imageDir = fullfile(toolboxdir('vision'), 'visiondata', ...
%      'structureFromMotion');
%  images = imageDatastore(imageDir);
%
%  % Compute features for the first and the second images
%  I1 = rgb2gray(readimage(images, 1));
%  points1 = detectSURFFeatures(I1);
%  [features1, points1] = extractFeatures(I1, points1);
%
%  I2 = rgb2gray(readimage(images, 2));
%  points2 = detectSURFFeatures(I2);
%  [features2, points2] = extractFeatures(I2, points2);
%
%  % Match features between the two images
%  matchedPairs = matchFeatures(features1, features2);
%
%  % Create a point track for the first pair of matched feature points
%  viewIds = [1 2];
%  imagePoints = [points1(matchedPairs(1,1)).Location; ...
%      points2(matchedPairs(1,2)).Location];
%  featureIndices = matchedPairs(1,:);
%  track = pointTrack(viewIds, imagePoints, featureIndices);
%
%  See also imageviewset, bundleAdjustment, bundleAdjustmentStructure,
%    triangulateMultiview, worldpointset, matchFeatures, vision.PointTracker.

%#codegen

% Copyright 2015-2021 The MathWorks, Inc.

classdef pointTrack < vision.internal.pointTrackImpl

    properties (Constant, Access = protected)
        %Version Tag for handling save/load in case of interface changes
        %
        %   1.0  : 16a   first shipping version
        %   1.1  : 20b   add FeatureIndices property
        Version = 1.1;
    end

    methods
        %------------------------------------------------------------------
        function this = pointTrack(varargin)
            this = this@vision.internal.pointTrackImpl(varargin{:});
        end
    end

    methods(Hidden)
        %------------------------------------------------------------------
        function s = toStruct(this)
        % Version number can be used to customize the loading process
            s = struct('ViewIds', this.ViewIds, ...
                       'Points', this.Points, ...
                       'FeatureIndices', this.FeatureIndices, ...
                       'Version', this.Version);
        end

        %------------------------------------------------------------------
        function s = saveobj(this)
            s = toStruct(this);
        end
    end

    methods(Static, Hidden)
        %------------------------------------------------------------------
        function this = loadobj(that)
            if isfield(that, 'FeatureIndices')
                this = pointTrack(that.ViewIds, that.Points, that.FeatureIndices);
            else
                this = pointTrack(that.ViewIds, that.Points);
            end
        end
    end

    methods(Access=public, Static, Hidden)
        %----------------------------------------------------------------------
        function name = matlabCodegenRedirect(~)
            name = 'vision.internal.codegen.pointTrack';
        end
    end
end
