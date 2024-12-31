function [metrics, varargout] = evaluateSemanticSegmentation(varargin)
%evaluateSemanticSegmentation Evaluate semantic segmentation data set against ground truth
%
%   metrics = evaluateSemanticSegmentation(dsResults,dsTruth) computes
%   various metrics to evaluate the quality of the semantic segmentation
%   results. dsResults represents the results of semantic segmentation and
%   holds the predicted pixel labels. dsTruth represents the ground truth
%   semantic segmentation and holds the true pixel labels.
%
%   dsResults and dsTruth can be any Datastore that returns categorical
%   images or volumes, such as a pixelLabelDatastore. The output of
%   read(dsResults) or read(dsTruth) must be a categorical array, a cell
%   array, or table. When the output of read is a multi-column cell array
%   or table, the second column of must be categorical arrays.
%
%   dsResults and dsTruth can be a cell arrays of Datastore objects, which
%   encapsulate label image files on disk.
%
%   metrics = evaluateSemanticSegmentation(datasetConfMat, classNames)
%   computes various metrics to evaluate the quality of the semantic
%   segmentation results from the segmentation confusion matrices.
%   datasetConfMat must be a F-by-1 table or F-by-1 cell array where F is
%   the number of images in the dataset. If it is a table, it must contain
%   a variable named 'ConfusionMatrix'. Each row in the table contains a
%   cell array with the confusion matrix for each image. If the input is a
%   cell array, each element must contain the confusion matrix for each
%   image.
%
%   [metrics, blockMetrics] = evaluateSemanticSegmentation(datasetConfMat, classNames) 
%   computes various metrics to evaluate the quality of the block-based
%   semantic segmentation results from the segmentation confusion matrices.
%   datasetConfMat must be a B-by-3 table where B is the total number of
%   blocks in all the images in the dataset. The table must contain the
%   following variable names, 'ImageNumber', 'ConfusionMatrix' and
%   'BlockInfo'. Each row in the table contains the details corresponding
%   to each block in the image. classNames is a set of class names
%   specified as a string vector or cell array of character vectors.
%
%   The output metrics is a semanticSegmentationMetrics object with the
%   following properties:
%
%    * metrics.ConfusionMatrix is the confusion matrix for the classes in
%      the data set. It is a square table where element (i,j) is the count
%      of pixels known to belong to class i but predicted to belong to
%      class j.
%
%    * metrics.NormalizedConfusionMatrix is the confusion matrix normalized
%      by the number of pixels known to belong to each class. Element (i,j)
%      represents the fraction, in [0,1], of pixels known to belong to
%      class i but predicted to belong to class j.
%
%    * metrics.DataSetMetrics contains the metrics computed over the data
%      set. When you use the dsResults and dsTruth input arguments, all
%      five metrics below are included by default in a 1-by-5 element
%      table. When you use the datasetConfMat and classNames input
%      arguments, the first four metrics are included by default in a
%      1-by-4 element table (MeanBFScore is not supported).
%
%       - GlobalAccuracy: Fraction of correctly classified pixels
%                         regardless of class.
%       - MeanAccuracy:   Fraction of correctly classified pixels
%                         averaged over the classes.
%       - MeanIoU:        Intersection over union (IoU) coefficient
%                         averaged over the classes. The IoU is also
%                         known as the Jaccard index.
%       - WeightedIoU:    IoU average weighted by the number of pixels
%                         (cardinal) in each class.
%       - MeanBFScore:    Mean over all the images of the mean BF score
%                         for each image. The BF score is a contour
%                         matching metric based on the F1-measure. It
%                         assesses how well predicted boundaries of objects
%                         match ground truth boundaries.
%
%    * metrics.ClassMetrics contains the metrics computed for each class.
%      When you use the dsResults and dsTruth input arguments, all three
%      metrics below are include by default in a C-by-3 element table,
%      where C is the number of classes. When you use the datasetConfMat
%      and classNames input arguments, the first two metrics are included
%      by default in a C-by-2 element table (MeanBFScore is not supported).
%
%       - Accuracy:    Fraction of correctly classified pixels
%                      in each class.
%       - IoU:         Intersection over union (IoU) coefficient for each
%                      class.
%       - MeanBFScore: Average over all the images of the BF score
%                      for each class.
%
%    * metrics.ImageMetrics contains the metrics computed for each image.
%      When you use the dsResults and dsTruth input arguments, all five
%      metrics supported by metrics.DataSetMetrics are include by default
%      in a F-by-5 table, where F is the number of images in the data
%      set. If pxdsResults and pxdsTruth are cell arrays of
%      pixelLabelDatastore objects, then metrics.ImageMetrics is a cell
%      array of tables. When you use the datasetConfMat and classNames
%      input arguments, the first four metrics supported by
%      metrics.DataSetMetrics are included by default in a F-by-4 element
%      table (MeanBFScore is not supported).
%      
%   blockMetrics is a F-by-1 cell array, where F is the number of
%      images in the data set. Each element in the cell array contains a
%      K-by-5 table by default, where K is the number of blocks in an image
%      in the data set, listing the same metrics as in
%      metrics.DataSetMetrics computed for each block individually
%      including BlockInfo. MeanBFScore is not supported. Each row in the
%      BlockInfo column contains a struct listing the spatial information
%      about each block with the following fields BlockStartWorld,
%      BlockEndWorld, DataStartWorld, DataEndWorld. For more details on
%      BlockInfo, read the 'IncludeBlockInfo' Name-Value pair of <a href="matlab:help('blockedImage/apply')">blockedImage/apply</a> method.
%
%   [...] = evaluateSemanticSegmentation(___,Name,Value,...) computes
%   semantic segmentation metrics using name-value pairs to control the
%   evaluation. Parameters include:
%
%      "Metrics"  -  Metric or list of metrics to compute, specified as
%                    a vector of strings. This parameter changes which
%                    variables in the DataSetMetrics, ClassMetrics, and
%                    ImageMetrics tables are computed. ConfusionMatrix and
%                    NormalizedConfusionMatrix are computed no matter what
%                    the value of this parameter is. Valid values are:
%                    "accuracy", "all", "bfscore", "global-accuracy",
%                    "iou", and "weighted-iou". When the input to the
%                    function is datasetConfMat, "bfscore" is not
%                    supported.
%
%                    Default: "all"
%
%      "Verbose"  -  Set true to display progress information.
%
%                    Default: true
%
%   Notes
%   -----
%   1) Control which metrics are computed using the "Metrics" parameter.
%      evaluateSemanticSegmentation always computes the classification
%      matrix.
%
%   2) evaluateSemanticSegmentation supports parallel computing using
%      multiple MATLAB workers when processing data from a
%      pixelLabelDatastore. Enable parallel computing using the
%      <a href="matlab:preferences('Computer Vision Toolbox')">preferences dialog</a>.
%
%   3) If dsResults and dsTruth are cell arrays of datastore objects, then
%      metrics.ImageMetrics is a cell array of tables listing the metrics
%      for each image with each table corresponding to a distinct
%      datastore object.
%
%   4) The "bfscore" metric is not supported when the input is specified as
%      confusion matrices, datasetConfMat.
%   
%   5) In block-based workflows, the input datasetConfMat table is
%      constructed from the output of the blockedImage/apply() method using
%      segmentationConfusionMatrix.
%
%   Reference
%   ---------
%   - Csurka, Gabriela, et al. "What is a good evaluation measure for
%     semantic segmentation?." BMVC. Vol. 27. 2013.
%
%   Example: Evaluate the results of semantic segmentation
%   ------------------------------------------------------
%
%     % The triangleImages data set has 100 test images with
%     % ground truth labels. Define the location of the data set.
%     dataSetDir = fullfile(toolboxdir('vision'),'visiondata','triangleImages');
%
%     % Define the location of the test images.
%     testImagesDir = fullfile(dataSetDir,'testImages');
%
%     % Define the location of the ground truth labels.
%     testLabelsDir = fullfile(dataSetDir,'testLabels');
%
%     % Create an imageDatastore holding the test images.
%     imds = imageDatastore(testImagesDir);
%
%     % Define the class names and their associated label IDs.
%     classNames = ["triangle", "background"];
%     labelIDs   = [255 0];
%
%     % Create a pixelLabelDatastore holding the
%     % ground truth pixel labels for the test images.
%     pxdsTruth = pixelLabelDatastore(testLabelsDir, classNames, labelIDs);
%
%     % Load a semantic segmentation network that has
%     % been trained on the training images of noisyShapes.
%     net = load('triangleSegmentationNetwork');
%     net = net.net;
%
%     % Run the network on the test images. Predicted labels are written to
%     % disk in a temporary directory and returned as a pixelLabelDatastore.
%     pxdsResults = semanticseg(imds, net, "WriteLocation", tempdir);
%
%     % Evaluate the prediction results against the ground truth.
%     metrics = evaluateSemanticSegmentation(pxdsResults, pxdsTruth);
%
%     % Display the classification accuracy, the intersection
%     % over union, and the boundary F-1 score for each class.
%     metrics.ClassMetrics
%
%   See also semanticseg, bfscore, jaccard, pixelLabelDatastore,
%            semanticSegmentationMetrics, pixelLabelTrainingData,
%            segmentationConfusionMatrix.

%   Copyright 2017-2023 The MathWorks, Inc.

[metrics, blockMetrics, canComputeBlockMetrics] = semanticSegmentationMetrics.compute(varargin{:});

if nargout>1
    if canComputeBlockMetrics
        varargout{1} = blockMetrics;
    else
       error(message('vision:semanticseg:cannotOutputBlockMetrics'));
    end
end

