classdef HistogramBasedTracker< matlab.System
%HistogramBasedTracker Track object in video based on histogram
% 
%   H = vision.HistogramBasedTracker returns a System object, H, that
%   tracks an object by using the Continuously Adaptive Mean Shift
%   (CAMShift) algorithm. It uses the histogram of pixel values to identify
%   the tracked object. To initialize the tracking process, you must use
%   the initializeObject method to specify an exemplar image of the object.
%   Then, use the step method to track the object in consecutive video
%   frames.
% 
%   H = vision.HistogramBasedTracker('PropertyName', PropertyValue, ...)
%   returns a tracker System object, H, with each specified property set to
%   the specified value.
% 
%   initializeObject method syntax:
% 
%   initializeObject(H, I, R) sets the object to track by extracting it
%   from the [x y width height] region R, in an M-by-N image I. I can be
%   any 2-D feature map that distinguishes the object from the background.
%   For example, I can be a hue channel of the HSV color space. R also
%   represents the initial search window for the next call to the step
%   method. Typically, I is the first frame of a video in which the object
%   appears. For best results, the object must occupy the majority of R.
% 
%   initializeObject(H, I, R, N) additionally, lets you specify N, the
%   number of histogram bins. By default, N is set to 16. Increasing N
%   enhances the ability of the tracker to discriminate the object.
%   However, it also narrows the range of changes to the object's visual
%   characteristics that the tracker can accommodate, making it more
%   susceptible to losing track.
%
%   step method syntax:
% 
%   BBOX = step(H, I) returns the [x y width height] bounding box, BBOX, of
%   the tracked object. Before calling the step method, use the
%   initializeObject method to identify the object to track, and to set the
%   initial search window.
% 
%   [BBOX, ORIENTATION] = step(H, I) additionally returns the angle between
%   the x-axis and the major axis of the ellipse, which has the same second
%   order moments as the object. The returned angle ranges from -pi/2 to
%   pi/2.
% 
%   [BBOX, ORIENTATION, SCORE] = step(H, I) additionally returns the
%   confidence score indicating whether the returned bounding box, BBOX,
%   contains the tracked object. SCORE is between 0 and 1, with the
%   greatest confidence equal to 1.
% 
%   initializeSearchWindow method syntax:
% 
%   initializeSearchWindow(H, R) sets the initial search  window, R,
%   specified as [x y width height]. The next call to the step method will
%   use R as the initial window to search for the object. This method is
%   useful when you lose track of the object. You can use it to
%   re-initialize object's initial location and size.
% 
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(obj, x) and y = obj(x) are
%   equivalent.
%
%   HistogramBasedTracker methods:
% 
%   step                   - See above description for use of this method
%   initializeObject       - See above description for use of this method
%   initializeSearchWindow - See above description for use of this method
%   release                - Allow property value and input characteristics changes
%   clone                  - Create a tracker object with the same property values
%   isLocked               - Locked status (logical)
% 
%   HistogramBasedTracker properties:
% 
%   ObjectHistogram - Normalized pixel value histogram
% 
%   Notes:
% 
%   - The HistogramBasedTracker is most suitable for tracking a single
%     object. 
%   - You can improve the computational speed of the HistogramBasedTracker
%     by setting the class of the image, I, to uint8.
% 
%   Example: Tracking a face
% 
%   % Create System objects for reading and displaying video, and for
%   % drawing bounding box of the object.
%   videoReader = VideoReader('vipcolorsegmentation.avi');
%   videoPlayer = vision.VideoPlayer();
% 
%   % Read the first video frame which contains the object and then show
%   % the object region
%   objectFrame = readFrame(videoReader);  % read the first video frame
%   objectHSV = rgb2hsv(objectFrame); % convert to HSV color space
%   objectRegion = [40, 45, 25, 25];  % define the object region
%   objectImage = insertShape(objectFrame, 'Rectangle', objectRegion, ...
%                            'ShapeColor', 'red');
%   figure; imshow(objectImage); title('Red box shows object region');
%   % You can also use the following commands to select the object region
%   % using a mouse. The object must occupy majority of the region.
%   % figure; imshow(objectFrame); objectRegion=round(getPosition(imrect))
% 
%   % Set the object based on the hue channel of the first video frame
%   tracker = vision.HistogramBasedTracker;
%   initializeObject(tracker, objectHSV(:,:,1) , objectRegion);
% 
%   % Track and display the object in each video frame
%   while hasFrame(videoReader)
%     frame = readFrame(videoReader);         % Read next image frame
%     hsv = rgb2hsv(frame);                   % Convert to HSV color space
%     bbox = step(tracker, hsv(:,:,1));       % Track object in hue channel
%                                             % where it's distinct from
%                                             % the background
%     out = insertShape(frame, 'Rectangle', ...
%                   bbox, 'ShapeColor', 'red');    % Draw a box around the object
%     step(videoPlayer, out);                      % Show results
%     pause(0.1);
%   end
% 
%   release(videoPlayer);
%
%   See also insertShape, imrect, rgb2hsv.

 
%   Copyright 2011-2022 The MathWorks, Inc.

    methods
        function out=HistogramBasedTracker
        end

        function out=backProject(~) %#ok<STOUT>
        end

        function out=clipROI(~) %#ok<STOUT>
        end

        function out=computeBoundingBox(~) %#ok<STOUT>
        end

        function out=cropImage(~) %#ok<STOUT>
        end

        function out=fitEllipse(~) %#ok<STOUT>
            % Compute the moments of the probability map in the extended ROI.
        end

        function out=getNumOutputsImpl(~) %#ok<STOUT>
        end

        function out=hist2D(~) %#ok<STOUT>
        end

        function out=initializeObject(~) %#ok<STOUT>
            % initializeObject Sets the object to track
            %   initializeObject(H, I, R) sets the object to track by extracting
            %   it from the [x y width height] region R, in an M-by-N image I. I
            %   can be any 2-D feature map that distinguishes the object from the
            %   background. For example, I can be a hue channel of the HSV color
            %   space. R also represents the initial search window for the next
            %   call to the step method. Typically, I is the first frame of a
            %   video in which the object appears. For best results, the object
            %   must occupy the majority of R.
            % 
            %   initializeObject(H, I, R, N) additionally, lets you specify the
            %   number of histogram bins N. By default, N is set to 16. N must be
            %   chosen according to the image data. A larger N does not always
            %   produce better result.
        end

        function out=initializeSearchWindow(~) %#ok<STOUT>
            % initializeSearchWindow Sets the initial search window
            %   initializeSearchWindow(H, R) sets the initial search  window, R,
            %   specified as [x y width height]. The next call to the step method
            %   will use R as the initial window to search for the object. This
            %   method is useful when you lose track of the object. You can use
            %   it to re-initialize object's initial location and size.
        end

        function out=isInputSizeMutableImpl(~) %#ok<STOUT>
        end

        function out=loadObjectImpl(~) %#ok<STOUT>
        end

        function out=meanShift(~) %#ok<STOUT>
        end

        function out=moment2D(~) %#ok<STOUT>
        end

        function out=saveObjectImpl(~) %#ok<STOUT>
        end

        function out=stepImpl(~) %#ok<STOUT>
        end

        function out=validateImage(~) %#ok<STOUT>
        end

        function out=validateInputsImpl(~) %#ok<STOUT>
        end

    end
    properties
        %ObjectHistogram Normalized pixel value histogram
        %   Set this property to an N-element vector which is the normalized
        %   histogram of the object's pixel values. Histogram values must be
        %   normalized to between 0 and 1. You can use the initializeObject
        %   method to set this property. This property is tunable.
        %
        %   Default: []
        ObjectHistogram;

    end
end
