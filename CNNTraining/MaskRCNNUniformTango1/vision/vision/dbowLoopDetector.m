classdef dbowLoopDetector < handle

%  Copyright 2024 MathWorks, Inc.

    properties(Hidden, Access = private)
        detectorObj
    end

    properties(Hidden, Access = public)
        SerializedBag = ''
        IsDefaultBag = false
    end

    methods
        function this = dbowLoopDetector(varargin)
            narginchk(0,1);
            if nargin == 0
                % Creating a database using the default vocabulary

                this.detectorObj = vision.internal.DBoWLoopDetector();
                this.SerializedBag = this.detectorObj.createDetectorFromDefaultBag();
                this.IsDefaultBag = true;
            elseif nargin == 1
                % Creating the database using a bagOfFeaturesDBoW object

                bag = varargin{1};
                validateattributes(bag, {'bagOfFeaturesDBoW'}, {}, mfilename, 'bag');
                this.detectorObj = vision.internal.DBoWLoopDetector();
                this.detectorObj.createDetectorFromSerializedBag(convertStringsToChars(bag.SerializedBag));
                this.SerializedBag = bag.SerializedBag;
                this.IsDefaultBag = false;
            end
        end

        function addVisualFeatures(this, viewID, features)
            % addVisualFeatures Add image features to database.
            %   addVisualFeatures(loopDetector, viewId, features) adds image features to the loopDetector
            %   database, where the image has a specified view id, viewID. viewID is a positive integer. features
            %   must be ORB feature descriptors represented as a binaryFeatures object.
            %
            %   Example 1 : Add image features to loop detector
            %   -----------------------------------------------------------------------------
            %   % Create the loop detector
            %   loopDetector =  dbowLoopDetector;
            %
            %   % Get query image features for 100th view
            %   viewId = 100;
            %   I = imread("cameraman.tif");
            %   points = detectORBFeatures(I);
            %   [features,~] = extractFeatures(I,points);
            %
            %   % Add image features to database
            %   addVisualFeatures(loopDetector, viewId, features);

            narginchk(3,4)
            validateattributes(viewID,{'numeric'},{'nonempty','scalar','nonsparse','real','integer','finite'},mfilename,'viewID');
            validateattributes(features,{'binaryFeatures'},{'nonempty','scalar'},mfilename,'features');
            addFeatures(this.detectorObj, viewID, features.Features);
        end

        function [loopViewIDs, similarityScores] = detectLoop(this,varargin)
            % detectLoop Detect loop closure by visual features.
            %   [loopViewIDs, similarityScores] = detectLoop(loopDetector, features) returns a list of view indices, specified as loopViewIDs, and
            %   their respective similarity scores, specified as similarityScores, when the database is searched for all similar images
            %   to the provided visual features. features must be ORB feature descriptors represented as a binaryFeatures object. similarityScores
            %   are normalized and lie between [0 1] where higher score represents greater similarity to the input visual features. View ids are
            %   returned in the descending order of their similarity scores.
            %
            %   [loopViewIDs, similarityScores] = detectLoop(loopDetector, features, connectedViewIds, relativeThreshold) returns
            %   a list of view indices, specified as loopViewIDs, and their respective similarity scores, specified as similarityScores, when
            %   the database is searched for non-connected similar images to the provided visual features. Connectivity information is not stored
            %   in the detector database, but the input connectedViewIds is used to discard the connected views and only return non-connected views.
            %   relativeThreshold value, lies between 0 and 1, is used to compute a threshold score to filter out the non-connected similar views.
            %   For example, we first compute the minimum similarity score among the connected views and then use relativeThreshold*(min score of connected views)
            %   to get the maximum threshold score such that similarityScores > relativeThreshold*(min score of connected views). 75 percent i.e. 0.75 is a good
            %   value for relativeThreshold.
            %
            %   [...] = detectLoop(..., Name, Value) specifies additional name-value pairs described below:
            %
            %   'NumResults'     - A positive integer specifying the maximum number of results to
            %                      return. Increase this value to return as many matching images as possible.
            %
            %                      Default: 20
            %
            %   Example 1 : Get 10 strong similar views from the given image
            %   -------------------------------------------------------------
            %   % Create the loop detector
            %   loopDetector =  dbowLoopDetector;
            %
            %   % Get query image features for 100th view
            %   viewId = 100;
            %   I = imread("cameraman.tif");
            %   points = detectORBFeatures(I);
            %   [features,~] = extractFeatures(I,points);
            %
            %   % Detect loop closure
            %   loopViewIds = detectLoop(loopDetector, features, NumResults=10);
            %
            %   Example 2 : Detect loop using connected views
            %   -------------------------------------------------------------------------------
            %   % Create the loop detector
            %   loopDetector =  dbowLoopDetector;
            %
            %   % Get query image features for 100th view
            %   viewId = 100;
            %   viewSet = imageviewset;
            %   I = imread("cameraman.tif");
            %   points = detectORBFeatures(I);
            %   [features,~] = extractFeatures(I,points);
            %
            %   Get connected view ids, connected to the current key frame
            %   covisViews          = connectedViews(viewSet, viewId);
            %   covisViewsIds       = covisViews.ViewId;
            %
            %   % Detect loop closure
            %   relativeThreshold = 0.75; % 75 percent of connected view scores
            %   loopViewIds = detectLoop(loopDetector, features, covisViewsIds, relativeThreshold);

            narginchk(2,6)
            parser = inputParser;
            addRequired(parser, 'features', @(x)validateattributes(x,{'binaryFeatures'},{'nonempty','scalar'},mfilename,'features'));
            addOptional(parser, 'connectedViewIDs', [], @(x)validateattributes(x, {'numeric'}, {'vector', 'integer'},mfilename,'connectedViewIDs'));
            addOptional(parser, 'relativeThreshold', [], @(x)validateattributes(x, {'numeric'},{'scalar','nonsparse','real','nonempty','<=',1,'>=',0},mfilename,'relativeThreshold'));
            addParameter(parser, 'NumResults', 20, @(x) validateattributes(x,{'numeric'},{'nonempty','scalar','integer','positive','real','nonsparse'},mfilename,'NumResults'));
            parse(parser,varargin{:})
            features = parser.Results.features;
            connectedViewIDs = parser.Results.connectedViewIDs;
            relativeThreshold = double(parser.Results.relativeThreshold);
            NumResults = double(parser.Results.NumResults);

            % Handling the two different types of function calls.
            if isempty(connectedViewIDs) && isempty(relativeThreshold)
                [loopViewIDs, similarityScores] = detectLoop(this.detectorObj,features.Features,NumResults);
            elseif ~isempty(connectedViewIDs) && ~isempty(relativeThreshold)
                [loopViewIDs, similarityScores] = detectLoopUsingConnectedViews(this.detectorObj,features.Features,double(connectedViewIDs),relativeThreshold,NumResults);
            elseif ~isempty(connectedViewIDs) && isempty(relativeThreshold)
                error(message('vision:dbowLoopDetector:missingRelativeThreshold'));
            else
                error(message('vision:dbowLoopDetector:invalidDetectLoopFunctionCall'));
            end
        end
    end

    methods (Static, Hidden)
        %------------------------------------------------------------------
        % Save object
        %------------------------------------------------------------------
        function that = saveobj(this)
            that.SerializedBag          = this.SerializedBag;
            that.IsDefaultBag             = this.IsDefaultBag;
        end
    end

    methods (Static, Hidden)
        %------------------------------------------------------------------
        % Load object
        %------------------------------------------------------------------
        function this = loadobj(that)
            this = dbowLoopDetector();
            if ~that.IsDefaultBag
                this.detectorObj = vision.internal.DBoWLoopDetector();
                this.detectorObj.createDetectorFromSerializedBag(convertStringsToChars(that.SerializedBag));
                this.SerializedBag = that.SerializedBag;
                this.IsDefaultBag = false;
            end
        end
    end
end