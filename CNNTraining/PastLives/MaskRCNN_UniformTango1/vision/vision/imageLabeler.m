function imageLabeler(varargin)
%imageLabeler Label ground truth in a collection of images.
%
%  imageLabeler launches the Image Labeler app for interactive image
%  labeling. You can use the app to label:
%
%     * Rectangular Regions of Interest (ROI) for object detection
%     * Pixel labels for semantic segmentation
%     * Scene labels for image classification
%
%  imageLabeler(imageFolder) opens the app and loads all images from
%  imageFolder.
%
%  imageLabeler(imgDatastore) opens the app and loads all images from an
%  imageDatastore object.
%
%  imageLabeler(sessionFile) opens the app and loads a saved session.
%  sessionFile is the path to a MAT file containing the saved session.
%
%  imageLabeler(projectFile) opens the app and loads a saved project.
%  projectFile is the path to a prj file containing the saved project.
%
%  imageLabeler(gTruth) opens the app and loads ground truth labels from
%  the scalar groundTruth object, gTruth. gTruth must contain ground truth
%  for a collection of images.
%
%  Notes
%  -----
%  - Define and execute custom label automation algorithms with the Image
%    Labeler app by creating an <a href="matlab:help('vision.labeler.AutomationAlgorithm')">AutomationAlgorithm</a> class.
%
%  - Only ground truth for a collection of images
%
%  Example: Open Image Labeler with an image collection.
%  ------------------------------------------------------
%  stopSignsDir = fullfile(toolboxdir('vision'), 'visiondata', 'stopSignImages')
%  imds = imageDatastore(stopSignsDir)
%  imageLabeler(imds)
%
%  See also videoLabeler, groundTruth, groundTruthDataSource,
%           objectDetectorTrainingData, pixelLabelTrainingData, 
%           vision.labeler.AutomationAlgorithm, labelDefinitionCreator.

% Copyright 2017-2023 The MathWorks, Inc.

% Following syntaxes are for internal use only and may change in the future.
%
%  imageLabeler(imageFolder, isDataBlockedImage) launch the app and
%  immediately load all images from imageFolder. If isDataBlockedImage flag
%  is true, all the images are loaded as blockedImages and the app is
%  directly launched in blockedImage labeling mode. If false, the app
%  launches in regular mode
%
%  imageLabeler(bimArray) launch the app and immediately load all the
%  blockedImages in bimArray. bimArray is an array of BlockedImage objects
%  from the workspace. The app is launched in BlockedImage labeling mode.
%  NOTES:
%    - Saving and loading session object is supported for this syntax.
%    - Saving and loading groundTruth objects is not supported for this
%      syntax. Changes made to BlockedImage objects will not be preserved.


vision.internal.imageLabeler.imageLabelerInternal(varargin{:});
