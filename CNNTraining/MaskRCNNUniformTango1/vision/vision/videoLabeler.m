function videoLabeler(varargin)
%videoLabeler Label ground truth data in video, image sequence or custom
%data source.
%
%   videoLabeler invokes the Video Labeler app for labeling ground truth
%   data in a video, image sequence or custom data source. This app is used
%   to interactively specify rectangular, polyline and pixel level Regions
%   of Interest (ROI) labels, and Scene labels, which together define
%   ground truth data that can be used for comparison against an algorithm
%   or training a classifier.
%
%   videoLabeler(videoFileName) opens the app and loads the video
%   videoFileName.
%
%   videoLabeler(imageSeqFolder) opens the app and loads the image
%   sequence in folder imageSeqFolder. imageSeqFolder is a scalar string or
%   character vector specifying a folder containing image files. Image
%   files must have extensions supported by imformats and are loaded in the
%   order returned by DIR.
%
%   videoLabeler(imageSeqFolder, timeStamps) opens the app and loads each
%   image in folder imageSeqFolder associated with a timestamp from
%   timeStamps. timeStamps is a duration vector of the same length as the
%   number of images in the sequence. If timeStamps is not specified, a
%   default of seconds(0 : numImages-1) is used, where numImages is the
%   number of images in the folder.
%
%   videoLabeler(gtSource) opens the app and loads data from the
%   data source gtSource. gtSource must be a groundTruthDataSource object
%   created from a video file, image sequence or custom reader.
%
%   videoLabeler(sessionFile) opens the app and loads a saved Video
%   Labeling session. sessionFile is the path to the MAT file containing
%   the saved session.
%
%   videoLabeler(gTruth) opens the app and loads ground truth data
%   from a scalar groundTruth object, gTruth. gTruth must contain ground
%   truth data for a video, an image sequence, or a custom data source.
%
%   Example 1: Open Video Labeler with a video
%   ------------------------------------------
%   % This example shows how to open the Video Labeler app with a video
%   videoLabeler('vipmen.avi')
%
%   Example 2: Open Video Labeler with an image sequence
%   ----------------------------------------------------
%   % This example shows how to open the Video Labeler app with a
%   % sequence of images located in a folder.
%
%   % Specify path for the image sequence directory.
%   imageDir = fullfile(toolboxdir('vision'), 'visiondata', 'NewTsukuba');
%
%   % Load time stamps corresponding to the image sequence
%   timeStamps = duration(0,0,1:150);
%
%   % Open Video Labeler with image sequence
%   videoLabeler(imageDir, timeStamps)
%
%   Example 3: Open Video Labeler with a custom reader
%   --------------------------------------------------
%   % This example shows how to open the Video Labeler app with a
%   % custom reader. A custom reader can be used to read image sequence
%   % data in a custom format that cannot be opened using imread or
%   % VideoReader. In this example, a custom reader is created using an
%   % image data store.
%
%   % Specify path for the image sequence directory.
%   imageDir = fullfile(toolboxdir('vision'), 'visiondata', ...
%       'calibration', 'mono');
%
%   % Load time stamps corresponding to the image sequence
%   timeStamps = duration(0,0,1:10);
%
%   % Use an image data store as a custom data source
%   imds = imageDatastore(imageDir);
%
%   % Write a reader function to read images from the data source. The
%   % first input argument to readerFcn, sourceName is not used. The
%   % 2nd input, currentTimeStamp is converted from a duration scalar
%   % to a 1-based index suitable for the data source.
%   frameInterval = seconds(timeStamps(2)-timeStamps(1));
%   readerFcn = @(~,idx)readimage(imds,floor(seconds(idx)/frameInterval));
%
%   % Create data source for images in imageDir using readerFcn
%   gtSource = groundTruthDataSource(imageDir, readerFcn, timeStamps);
%
%   % Open Video Labeler with created data source
%   videoLabeler(gtSource)
%
%   See also imageLabeler, groundTruth, groundTruthDataSource,
%   objectDetectorTrainingData, pixelLabelTrainingData,
%   vision.labeler.AutomationAlgorithm, labelDefinitionCreator.

%   Copyright 2018-2024 The MathWorks, Inc.

narginchk(0,4);

vision.internal.videoLabeler.videoLabelerInternal(varargin{:});

end
