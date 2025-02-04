classdef PointTracker< matlab.System
%PointTracker Track points in video using Kanade-Lucas-Tomasi (KLT) algorithm
% 
%   H = vision.PointTracker returns a System object, H, that
%   tracks a set of points using the Kanade-Lucas-Tomasi feature tracking 
%   algorithm.  To initialize the tracking process, you must use
%   the initialize method to specify the initial locations of the points
%   and the initial video frame. Then, use the step method to track the 
%   points in subsequent video frames.  You can also reset the points
%   at any time by using the setPoints method.
% 
%   H = vision.PointTracker(Name, Value, ...)
%   configures the tracker object properties, specified as one or more 
%   name-value pair arguments. Unspecified properties have default values.
% 
%   initialize method syntax:
% 
%   initialize(H, POINTS, I) initializes points to track and sets the
%   initial video frame. The initial locations POINTS, must be an M-by-2
%   array of [x y] coordinates. The initial video frame I, must be a 2-D
%   grayscale or RGB image, and must be the same size and data type as the
%   video frames passed to the step method. 
%
%   step method syntax:
% 
%   [POINTS, POINT_VALIDITY] = step(H, I) tracks the points in the input
%   frame, I. The output POINTS contain an M-by-2 array of [x y]
%   coordinates that correspond to the new locations of the points in the
%   input frame, I. The output, POINT_VALIDITY provides an M-by-1 logical
%   array, indicating whether or not each point has been reliably tracked.
%   The input frame, I must be the same size and data type as the video
%   frames passed to the initialize method. 
% 
%   [POINTS, POINT_VALIDITY, SCORES] = step(H, I) additionally returns the
%   confidence score for each point. The M-by-1 output array, SCORE,
%   contains values between 0 and 1. These values correspond to the degree
%   of similarity between the neighborhood around the previous location and
%   new location of each point. These values are computed as a function of
%   the sum of squared differences between the previous and new
%   neighborhoods. The greatest tracking confidence corresponds to a
%   perfect match score of 1. 
% 
%   setPoints method syntax:
%
%   setPoints(H, POINTS) sets the points for tracking. The method sets the
%   M-by-2 POINTS array of [x y] coordinates with the points to track. This
%   method can be used if the points need to be re-detected because too
%   many of them have been lost during tracking. 
%
%   setPoints(H, POINTS, POINT_VALIDITY) additionally lets you mark points
%   as either valid or invalid. The input logical vector POINT_VALIDITY of
%   length M, contains the true or false value corresponding to the
%   validity of the point to be tracked. The length M corresponds to the
%   number of points. A false value indicates an invalid point that should
%   not be tracked. For example, you can use this method with the
%   estimateGeometricTransform2D function to determine the transformation
%   between the point locations in the previous and current frames, and
%   then mark the outliers as invalid.
% 
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(obj, x) and y = obj(x) are
%   equivalent.
%
%   PointTracker methods:
% 
%   step         - See above description for use of this method
%   initialize   - See above description for use of this method
%   setPoints    - See above description for use of this method
%   release      - Allow property value and input characteristics changes
%   clone        - Create a tracker object with the same property values
%   isLocked     - Locked status (logical)
% 
%   PointTracker properties:
% 
%   BlockSize              - The size of the integration window around each point.
%   NumPyramidLevels       - The number of pyramid levels.
%   MaxIterations          - The maximum number of search iterations.
%   MaxBidirectionalError  - Maximum threshold on the forward-backward error. 
%   
%
%   Example: Tracking a face
% 
%   % Create System objects for reading and displaying video, and for
%   % drawing a bounding box of the object.
%   videoReader = VideoReader('visionface.avi');
%   videoPlayer = vision.VideoPlayer('Position', [100, 100, 680, 520]);
%
%   % Read the first video frame which contains the object and then show
%   % the object region
%   objectFrame = readFrame(videoReader);  % read the first video frame
% 
%   objectRegion = [264, 122, 93, 93];  % define the object region
%   % You can also use the following commands to select the object region
%   % using a mouse. The object must occupy majority of the region.
%   % figure; imshow(objectFrame); objectRegion=round(getPosition(imrect))
%
%   objectImage = insertShape(objectFrame, 'Rectangle', objectRegion, ...
%                             'ShapeColor', 'red');
%   figure; imshow(objectImage); title('Red box shows object region');
% 
%   % Detect interest points in the object region
%   points = detectMinEigenFeatures(rgb2gray(objectFrame), 'ROI', objectRegion);
%   
%   % Display the detected points
%   pointImage = insertMarker(objectFrame, points.Location, '+', ...
%                             'MarkerColor', 'white');
%   figure, imshow(pointImage), title('Detected interest points');
% 
%   % Create a tracker object.
%   tracker = vision.PointTracker('MaxBidirectionalError', 1);
% 
%   % Initialize the tracker
%   initialize(tracker, points.Location, objectFrame);
% 
%   % Track and display the points in each video frame
%   while hasFrame(videoReader)
%     frame = readFrame(videoReader);             % Read next image frame
%     [points, validity] = step(tracker, frame);  % Track the points
%     out = insertMarker(frame, points(validity, :), '+'); % Display points
%     step(videoPlayer, out);                    % Show results
%     pause(0.1);
%   end
% 
%   release(videoPlayer);
%
%   See also vision.MarkerInserter, detectHarrisFeatures,
%   detectMinEigenFeatures, detectFASTFeatures, imcrop, imrect,
%   vision.HistogramBasedTracker.

  
%   Copyright 2011-2023 The MathWorks, Inc.

    methods
        function out=PointTracker
        end

        function out=getKLTParams(~) %#ok<STOUT>
        end

        function out=getNumOutputsImpl(~) %#ok<STOUT>
        end

        function out=getNumPyramidLevels(~) %#ok<STOUT>
            % Set NumPyramidLevels so that the top level of the pyramid
            % has the size of at least 3x3, which is the minimum permissible
            % size for the KLT algorithm.
        end

        function out=initialize(~) %#ok<STOUT>
            % initialize Sets the points to track and the initial video frame
            %   initialize(H, POINTS, I) sets the points to track in an image I.
            %   POINTS must be an N-by-2 array of [x y] coordinates, and I must 
            %   be a grayscale or RGB image representing the first video frame.
        end

        function out=isInputSizeMutableImpl(~) %#ok<STOUT>
        end

        function out=loadObjectImpl(~) %#ok<STOUT>
            % public properties
        end

        function out=normalizeScores(~) %#ok<STOUT>
        end

        function out=pointsOutsideImage(~) %#ok<STOUT>
        end

        function out=releaseImpl(~) %#ok<STOUT>
        end

        function out=saveObjectImpl(~) %#ok<STOUT>
            % public properties
        end

        function out=setPoints(~) %#ok<STOUT>
            % setPoints Sets the points to track.
            %   setPoints(H, POINTS, PointValididty) sets the points to be 
            %   tracked. POINTS must be an N-by-2 array of [x y] coordinates. 
            %   PointValididty must be an N-element logical vector, in which the 
            %   value of true indicates that the corresponding point is currently 
            %   visible and should be tracked. This method can be used to 
            %   re-initialize the points when too many points are lost during 
            %   tracking. It can also be used to modify the locations of 
            %   previously tracked points, or to mark certain points as invalid.
        end

        function out=stepImpl(~) %#ok<STOUT>
            % step() can only be called after initialize()
        end

        function out=validateImage(~) %#ok<STOUT>
        end

        function out=validateInputsImpl(~) %#ok<STOUT>
        end

        function out=validatePoints(~) %#ok<STOUT>
        end

    end
    properties
        % BlockSize Size of integration window 
        %   Specify a two-element vector, [height, width] to represent the
        %   neighborhood around each point being tracked. The height and width
        %   must be odd integers. This neighborhood defines the area for the
        %   spatial gradient matrix computation. The minimum value for 
        %   BlockSize is [5 5]. Increasing the size of the neighborhood, 
        %   increases the computation time. 
        %
        %   Default: [31 31]
        BlockSize;

        % MaxBidirectionalError Forward-backward error threshold 
        %   Specify a numeric scalar for the maximum bidirectional error. If 
        %   the value is less than inf, the object tracks each point from the
        %   previous to the current frame, and then tracks the same points back
        %   to the previous frame. The object calculates the bidirectional 
        %   error, which is the distance in pixels from the original location 
        %   of the points to the final location after the backward tracking. 
        %   The corresponding points are considered invalid when the error is 
        %   greater than the value set for this property. Recommended values 
        %   are between 0 and 3 pixels. 
        %
        %   Using the bidirectional error is an effective way to eliminate 
        %   points that could not be reliably tracked. However, the 
        %   bidirectional error requires additional computation. When you set 
        %   the MaxBidirectionalError property to inf, the object will not 
        %   compute the bidirectional error. 
        %
        %   Default: inf
        MaxBidirectionalError;

        % MaxIterations Maximum number of search iterations
        %   Specify a positive integer scalar for the maximum number of search
        %   iterations for each point. The KLT algorithm performs an iterative
        %   search for the new location of each point until convergence.
        %   Typically, the algorithm converges within 10 iterations. This
        %   property sets the limit on the number of search iterations.
        %   Recommended values are between 10 and 50. 
        %
        %   Default: 30
        MaxIterations;

        % NumPyramidLevels Number of pyramid levels
        %   Specify an integer scalar number of pyramid levels. The point 
        %   tracker implementation of the KLT algorithm uses image pyramids. 
        %   The object generates an image pyramid, where each level is reduced 
        %   in resolution by a factor of two compared to the previous level. 
        %   Selecting a pyramid level greater than one, enables the algorithm 
        %   to track the points at multiple levels of resolution, starting at 
        %   the lowest level. Increasing the number of pyramid levels allows 
        %   the algorithm to handle larger displacements of points between 
        %   frames, but also increases computation. Recommended values are 
        %   between 1 and 4. 
        %
        %   Default: 3
        NumPyramidLevels;

    end
end
