% objectMetrics - object based AP style quality metrics.
%  
%   Object encapsulating AP style quality metrics for a dataset. This
%   serves as the base class for application specific metrics evaluation
%   functions.
%
%   objectMetrics properties:
%
%      ConfusionMatrix  -  Confusion matrix summarizing the classification
%                          results for all objects in the data set.
%
%      NormalizedConfusionMatrix  -  Confusion matrix where the counts of
%                                    predicted objects for each class are
%                                    divided by the number of objects known
%                                    to belong to that class.
%
%      DataSetMetrics  -  Table of metrics aggregated over the data
%                         set.
%
%      ClassMetrics  -  Table of metrics computed for each class.
%
%      ImageMetrics  -  Table where each row lists the metrics for each
%                       image in the data set.
%

%   Copyright 2022-2023 The MathWorks, Inc.

classdef objectMetrics

    properties (SetAccess = protected)
        % ImageMetrics contains the metrics computed for each
        % image in the dataset. This is returned as a numImages-by-numMetrics
        % table. By default, the following metrics are returned each row:
        %      
        % - mAP : Mean average precision is average precision values
        %         averaged over all thresholds specified in the threshold
        %         argument for one image.
        % - AP  : Average precision computed at all the overlap thresholds
        %         specified in threshold argument as a numThresh-by-1 array. 
        ImageMetrics
        
        % ClassMetrics  contains the metrics computed for each class.
        % The output here is a numClasses-by-numMetrics table. By default, 
        % the following metrics are returned per row:
        %     
        % - mAP       : Mean average precision is average precision values
        %               averaged over all thresholds specified in the 
        %               threshold argument for one class.
        % - AP        : Average precision computed at all the overlap 
        %               thresholds specified in threshold argument as a 
        %               numThresh-by-1 array.
        % - Precision : The precision values as a 
        %               numThresh-by-(numObjects+1) numeric array.
        % - Recall    : The recall values as a 
        %               numThresh-by-(numObjects+1) numeric array.
        ClassMetrics
        
        % ClassNames contains a vector of string holding the class names of
        % the objects.
        ClassNames

        % OverlapThreshold
        OverlapThreshold
        
    end

    properties (SetAccess = protected, Hidden = true)
        % Undocumented features, in favor of methods confusionMatrix(obj)
        % and summarize(obj) from 24b onwards.

        % DataSetMetrics contains the metrics averaged over the entire
        % dataset. This is returned as a 1-by-numMetrics table. 
        % By default, the following metrics are returned:
        %      
        % - mAP : Mean average precision is average precision values 
        %         averaged over all thresholds specified in the threshold 
        %         argument.
        % - AP  : Average precision computed at all the overlap thresholds
        %         specified in threshold argument as a numThresh-by-1 array.
        DatasetMetrics

        % ConfusionMatrix Confusion matrix
        % A square table where element (i,j) is the count
        % of objects known to belong to class i but predicted to belong to
        % class j.
        ConfusionMatrix
        
        %NormalizedConfusionMatrix Normalized confusion matrix
        %   A square table where element (i,j) is the share (in [0,1]) of
        %   objects known to belong to class i but predicted to belong to
        %   class j.
        NormalizedConfusionMatrix
    end

    % Configuration properties
    properties (Access = protected)

        PredDS        % Datastore holding the prediction results
        TruthDS       % Datastore holding the groundTruth 
        Verbose       % Flag to enable verbose display
        UseParallel   % Flag to enable parallel processing
        SimilarityFcn % Fcn to compute similarity ratios between all pred gt pairs (like, bboxoverlapratio)
        ReturnLAMR    % Flag to enable returning miss rate
        ReturnAOS     % Flag to enable returning AOS
        TaskName      % Name of the task being evaluated
    end

    % Cached intermediate representation from the DS to compute P-R metrics
    properties (Access = protected)
        PredTable
        TruthTable
        PredTableAllClass
        TruthTableAllClass
        NumImages
        NumClasses
        NumThreshold
        Printer
    end

    methods(Access = protected, Abstract)
        sample = getObjFromList (this, objList, idx);
        area = objArea(~, objList);
    end


    methods
        function obj =  objectMetrics(predDS, gtDS, similarityFcn, threshold, useParallel, verbose, taskName, varargin)
            
            obj.PredDS        = predDS;
            obj.TruthDS       = gtDS;
            obj.OverlapThreshold= threshold;
            obj.Verbose       = verbose;
            obj.UseParallel   = useParallel;
            obj.SimilarityFcn = similarityFcn;            
            obj.TaskName      = taskName;
            
            obj.ReturnLAMR = false;
            obj.ReturnAOS = false;
            if ~isempty(varargin)
                metricsNames = varargin{1};
                obj.ReturnLAMR = ismember("LAMR", metricsNames);
                obj.ReturnAOS = ismember("AOS", metricsNames);
            end


            % Handle verbose display
            obj.Printer = vision.internal.MessagePrinter.configure(verbose);
            
            sampleGT = preview(gtDS);
            samplePred = preview(predDS);
            
            if(~isequal(sort(categories(sampleGT{2})), sort(categories(samplePred{2}))))
                error(message('vision:objectMetrics:mismatchCats'));
            end

            obj.ClassNames = categories(samplePred{2});
            if isempty(obj.ClassNames)
                error(message('vision:objectMetrics:emptyCats'))
            end

            obj.NumClasses = numel(obj.ClassNames);
            obj.NumThreshold = numel(obj.OverlapThreshold);
            
            obj = calculateDataForPrecisionRecall(obj);
            
            iPrintFinalize(obj.Printer);

            obj.ConfusionMatrix = obj.aggregateConfusionMetric();
            obj.NormalizedConfusionMatrix = obj.computeNormalizedConfusionMatrix();
            obj.ClassMetrics = obj.computeClassMetrics();
            obj.ImageMetrics = obj.computeImageMetrics();
            obj.DatasetMetrics = obj.computeDatasetMetrics();          

        end

        function areaMetrics = metricsByArea(metrics, objectAreaRanges, options)
            arguments
                metrics (1,1) {mustBeA(metrics, ["objectDetectionMetrics" "instanceSegmentationMetrics"])}
                objectAreaRanges (:,:) {mustBeNumeric, mustBeReal, mustBeNonnegative, mustBeNonempty, iValidateAreaRanges}
                options.ClassNames (:,1) string {mustBeNonempty,iValidateClassNameInput(options.ClassNames, metrics)} = metrics.ClassNames
            end

            % Convert area edges format to ranges format
            if isvector(objectAreaRanges)
                numBins = numel(objectAreaRanges) - 1;
                objectAreaRanges = objectAreaRanges([1:numBins;(1:numBins)+1]');
            end

            areaMetrics = [];

            for areaIdx = 1:size(objectAreaRanges,1)
                metricsCopy = metrics;
                areaRange = objectAreaRanges(areaIdx,:);

                % Determine ground truths to be removed
                removedTruthFlag = metricsCopy.TruthTable.Area >= areaRange(2) ...
                    | metricsCopy.TruthTable.Area < areaRange(1);

                % Determine detections matched to removed ground truths
                removedTruthIdx = find(removedTruthFlag);
                predMatchingRemovedGT = ismember(metricsCopy.PredTable.AssignedTruthIdx,removedTruthIdx);

                % Determine detections outside size range
                outOfRangePred = metricsCopy.PredTable.Area >= areaRange(2) ...
                    | metricsCopy.PredTable.Area < areaRange(1);

                % Determine detections that are not matched to anything
                unmatchedPred = metricsCopy.PredTable.AssignedTruthIdx == 0;
                
                % Determine detections to be removed
                removedPredFlag = predMatchingRemovedGT | (outOfRangePred & unmatchedPred);

                % Remove GT & detections
                metricsCopy.TruthTable(removedTruthFlag,:) = [];
                metricsCopy.PredTable(removedPredFlag,:) = [];

                % Compute class metrics using filtered data
                metricsCopy.ClassMetrics = metricsCopy.computeClassMetrics();
                    
                areaRow = table(areaRange, VariableNames="AreaRange");
                % If one class is specified, return metrics for that class
                if isscalar(options.ClassNames)
                    areaRow = [areaRow metricsCopy.ClassMetrics(options.ClassNames,:)];
                    areaRow.Properties.RowNames = {};
                % Otherwise, return metrics averaged over specified classes
                else
                    metricsCopy.ClassMetrics = metricsCopy.ClassMetrics(options.ClassNames,:);
                    metricsCopy.DatasetMetrics = metricsCopy.computeDatasetMetrics();
                    areaRow = [areaRow metricsCopy.DatasetMetrics];
                end
                areaMetrics = [areaMetrics; areaRow];
            end
        end

        function ap = averagePrecision(metrics, options)
            arguments
                metrics (1,1) {mustBeA(metrics, ["objectDetectionMetrics" "instanceSegmentationMetrics"])}
                options.ClassName (:,1) string {mustBeNonempty,iValidateClassNameInput(options.ClassName, metrics)} = metrics.ClassNames
                options.OverlapThreshold (1,:) {mustBeNumeric, mustBeReal, mustBeInRange(options.OverlapThreshold,0,1),... 
                    mustBeNonempty,iValidateOverlapThresholdInput(options.OverlapThreshold, metrics)} = metrics.OverlapThreshold
            end
            ap = zeros(length(options.ClassName), length(options.OverlapThreshold));

            % Find indices of input overlap thresholds (stable ordering)
            overlapIdx = arrayfun(@(x)find(metrics.OverlapThreshold==x,1), options.OverlapThreshold);

            for c = 1:length(options.ClassName)
                clsName = options.ClassName(c);
                % AP for c-th class and t-th threshold: {c,t}
                for t = 1:length(overlapIdx)
                    idx = overlapIdx(t);
                    ap(c,idx) = metrics.ClassMetrics{clsName,"AP"}{1}(idx);
                end
            end
        end


        function [precision, recall, scores] = precisionRecall(metrics, options)
            arguments
                metrics (1,1) {mustBeA(metrics, ["objectDetectionMetrics" "instanceSegmentationMetrics"])}
                options.ClassName (:,1) string {mustBeNonempty,iValidateClassNameInput(options.ClassName, metrics)} = metrics.ClassNames
                options.OverlapThreshold (1,:) {mustBeNumeric, mustBeReal, mustBeInRange(options.OverlapThreshold,0,1),... 
                    mustBeNonempty,iValidateOverlapThresholdInput(options.OverlapThreshold, metrics)} = metrics.OverlapThreshold
            end
            precision = cell(length(options.ClassName), length(options.OverlapThreshold));
            recall = cell(length(options.ClassName), length(options.OverlapThreshold));
            scores = cell(length(options.ClassName), 1);
            
            % Find indices of input overlap thresholds (stable ordering)
            overlapIdx = arrayfun(@(x)find(metrics.OverlapThreshold==x,1), options.OverlapThreshold);

            for c = 1:length(options.ClassName)
                clsName = options.ClassName(c);
                for t = 1:length(overlapIdx)
                    idx = overlapIdx(t);
                    recall{c,idx} = metrics.ClassMetrics{clsName,"Recall"}{1}(idx,:);
                    precision{c,idx} = metrics.ClassMetrics{clsName,"Precision"}{1}(idx,:);
                end

                % scores for c-th class (do not vary with IoU threshold)
                scores{c} = [1; sort(...
                    metrics.PredTable(metrics.PredTable.PredLabel == clsName,:).Score,...
                    "descend")]';
            end

            % if isscalar(options.ClassName) && isscalar(options.OverlapThreshold)
            %     % Flatten cell arrays if only a single class is given
            %     recall = recall{1};
            %     precision = precision{1};
            %     scores = scores{1};
            % end

        end


        function [summaryDataset, summaryClass] = summarize(metrics, options)
            arguments
                metrics (1,1) {mustBeA(metrics, ["objectDetectionMetrics" "instanceSegmentationMetrics"])}
                options.MetricName (1,1) string {mustBeNonempty,...
                    mustBeMember(options.MetricName,["all","AP","LAMR","AOS"])} = "all"
            end
            if options.MetricName=="LAMR"
                if ~metrics.ReturnLAMR
                    error(message('vision:objectMetrics:unavailableSummaryLAMR'));
                end
            end
            if options.MetricName=="AOS"
                if ~metrics.ReturnAOS
                    error(message('vision:objectMetrics:unavailableSummaryAOS'));
                end
            end

            % -------------------------------------------------------------
            % Summary class metrics:
            % - number of objects in each class
            % - AP for each class averaged over all overlap thresholds
            % - AP of each class at each overlap threshold as a separate column
            summaryClass = table('Size', [metrics.NumClasses, 1],...
                'VariableNames', {'NumObjects'},...
                'VariableTypes',{'double'},...
                'RowNames', metrics.ClassNames);
            summaryClass.NumObjects = metrics.ClassMetrics.NumObjects;

            if ismember(options.MetricName, ["AP","all"])
                APOverlapAvg = metrics.ClassMetrics.APOverlapAvg;
                summaryClass = addvars(summaryClass, APOverlapAvg);

                namesAPOverlaps = "AP" + string(metrics.OverlapThreshold);
                allOverlapsAPs = array2table(...
                    cat(2,metrics.ClassMetrics.AP{:,1})',...
                    VariableNames=namesAPOverlaps);
                summaryClass = [summaryClass, allOverlapsAPs];
            end

            if metrics.ReturnLAMR && ismember(options.MetricName, ["LAMR","all"])
                % average LAMR over all overlaps
                LAMROverlapAvg = metrics.ClassMetrics.LAMROverlapAvg;
                summaryClass = addvars(summaryClass, LAMROverlapAvg);

                % average LAMR over all classes, at each individual overlap
                allOverlapLAMRs = cat(2,metrics.ClassMetrics.LAMR{:})';
                namesLAMROverlaps = "LAMR" + string(metrics.OverlapThreshold);
                summaryClass = [summaryClass,...
                    array2table(allOverlapLAMRs, VariableNames=namesLAMROverlaps)];
            end

            if metrics.ReturnAOS && ismember(options.MetricName, ["AOS","all"])
                % average AOS over all overlaps, all classes
                AOSOverlapAvg = metrics.ClassMetrics.AOSOverlapAvg;
                summaryClass = addvars(summaryClass, AOSOverlapAvg);

                % average AOS over all classes, at each individual overlap
                allOverlapAOS = cat(2,metrics.ClassMetrics.AOS{:})';
                namesAOSOverlaps = "AOS" + string(metrics.OverlapThreshold);
                summaryClass = [summaryClass,...
                    array2table(allOverlapAOS, VariableNames=namesAOSOverlaps)];
            end

            % -------------------------------------------------------------
            % Summary dataset metrics:
            % - mAPOverlapAvg: over all classes and all overlaps thresholds
            summaryDataset = table('Size', [1, 1],...
                'VariableNames', {'NumObjects'},...
                'VariableTypes',{'double'});
            summaryDataset.NumObjects = metrics.DatasetMetrics.NumObjects;

            if ismember(options.MetricName, ["AP","all"])
                mAPOverlapAvg = metrics.DatasetMetrics.mAPOverlapAvg;
                summaryDataset = addvars(summaryDataset, mAPOverlapAvg);
                % - average over all classes (mAP), at each individual overlap
                namesMAPOverlaps = "mAP" + string(metrics.OverlapThreshold);
                allOverlapMAPs = cat(2,metrics.DatasetMetrics.mAP{:})';
                summaryDataset = [summaryDataset,...
                    array2table(allOverlapMAPs, VariableNames=namesMAPOverlaps)];
            end

            if metrics.ReturnLAMR && ismember(options.MetricName, ["LAMR","all"])
                % average LAMR over all overlaps, all classes
                mLAMROverlapAvg = metrics.DatasetMetrics.mLAMROverlapAvg;
                summaryDataset = addvars(summaryDataset, mLAMROverlapAvg);

                % average LAMR over all classes, at each individual overlap
                allOverlapLAMRs = cat(2,metrics.DatasetMetrics.mLAMR{:})';
                namesLAMROverlaps = "mLAMR" + string(metrics.OverlapThreshold);
                summaryDataset = [summaryDataset,...
                array2table(allOverlapLAMRs, VariableNames=namesLAMROverlaps)];
            end

            if metrics.ReturnAOS && ismember(options.MetricName, ["AOS","all"])
                % average AOS over all overlaps, all classes
                mAOSOverlapAvg = metrics.DatasetMetrics.mAOSOverlapAvg;
                summaryDataset = addvars(summaryDataset, mAOSOverlapAvg);

                % average AOS over all classes, at each individual overlap
                allOverlapAOS = cat(2,metrics.DatasetMetrics.mAOS{:})';
                namesAOSOverlaps = "mAOS" + string(metrics.OverlapThreshold);
                summaryDataset = [summaryDataset,...
                array2table(allOverlapAOS, VariableNames=namesAOSOverlaps)];
            end

        end


        function [confusionMatrix, confusionClassNames] = confusionMatrix(metrics, options)

            arguments
                metrics (1,1) {mustBeA(metrics, ["objectDetectionMetrics" "instanceSegmentationMetrics"])}
                options.ScoreThreshold (1,:) {mustBeNumeric, mustBeReal,...
                    mustBeFinite, mustBeNonempty,... % NOTE: scores can be -ve (e.g. SVMs)
                    mustBeInRange(options.ScoreThreshold,0,1),... 
                    iValidateUniqueScoreThresholdInput(options.ScoreThreshold)} = 0
                options.OverlapThreshold (1,:) {mustBeNumeric, mustBeReal,...
                    mustBePositive, mustBeFinite, mustBeNonempty,...
                    mustBeInRange(options.OverlapThreshold,0,1),...
                    iValidateOverlapThresholdInput(options.OverlapThreshold, metrics)} = metrics.OverlapThreshold
                options.Normalize (1,1) {mustBeNumericOrLogical, mustBeMember(options.Normalize,[0,1])} = false
            end

            numClasses = metrics.NumClasses;
            classes = categorical(1:numClasses, 1:numClasses, metrics.ClassNames);

            numOverlapThresh = numel(options.OverlapThreshold);
            numScoreThresh = numel(options.ScoreThreshold);

            confusionMatrix = cell(numScoreThresh, numOverlapThresh);

            for sIdx = 1:numScoreThresh
                for tIdx = 1:numOverlapThresh

                    % The extra class represents the implicit "background" class,
                    % accounting for false positives and false negatives.
                    confusionMatrix{sIdx,tIdx} = zeros(numClasses+1, numClasses+1);

                    % All predictions that are above the minimum confidence
                    % score threshold and the overlap score threshold
                    trueSamples = (metrics.PredTableAllClass.OverlapScore >= options.OverlapThreshold(tIdx)) ...
                        & (metrics.PredTableAllClass.Score >  options.ScoreThreshold(sIdx)); 
    
                    % All predictions that are unmatched with a ground-truth
                    % after applying the minimum confidence score and overlap
                    % score thresholds
                    falsePositive_rows = (ismissing(metrics.PredTableAllClass.AssignedLabel)...
                        | (metrics.PredTableAllClass.OverlapScore < options.OverlapThreshold(tIdx)))...
                        & (metrics.PredTableAllClass.Score >  options.ScoreThreshold(sIdx));
    
                    for i = 1:numClasses % true class
    
                        if isempty(metrics.PredTableAllClass)
                            continue
                        end
    
                        if isempty(metrics.TruthTableAllClass)
                            continue
                        end
    
                        % Select rows corresponding to ground truth class i.
                        truthClassI_rows = (metrics.PredTableAllClass.AssignedLabel == classes(i));
                        trueClassSamples = trueSamples(truthClassI_rows ,:);
    
                        % -----------------------------------------------------
                        % This accounts for BACKGROUND confusions only
                        % (does not consider inter-class confusions):
                        % -----------------------------------------------------
    
                        % False negatives for this truth class
    
                        % Case 1: truth class i, no matched prediction 
                        % (unassigned ground truths), without any
                        % thresholds being applied to overlaps or confidence
                        numUnassignedClassI_truths = sum(...
                            (metrics.TruthTableAllClass.IsAssigned == 0)...
                            & (metrics.TruthTableAllClass.TruthLabel == classes(i)), 'all');
    
                        %   Case 2: a prediction is assigned to truth class i
                        %   but falls below the overlap score threshold or the
                        %   confidence score threshold as well, resulting in 
                        %   more ground truths becoming unmatched.
                        selectPredsBelowThreshold_ClassI = ...
                            ((metrics.PredTableAllClass.OverlapScore < options.OverlapThreshold(tIdx))...
                            | (metrics.PredTableAllClass.Score <  options.ScoreThreshold(sIdx)))...
                            & (metrics.PredTableAllClass.AssignedLabel == classes(i));
                        %   How many newly un-assigned ground-truths does this
                        %   cause?
                        numUnassignedClassIAtThreshold_truths = numel(...
                            unique( metrics.PredTableAllClass.AssignedTruthIdx(selectPredsBelowThreshold_ClassI) ) );
                        
                        % False negatives
                        %   true class i
                        %   predicted class is background (numClasses+1)
                        confusionMatrix{sIdx,tIdx}(i,numClasses+1) = numUnassignedClassI_truths + numUnassignedClassIAtThreshold_truths;
                        
                        % -----------------------------------------------------
                        % This accounts for the inter-class confusions:
                        % -----------------------------------------------------
                        for j = 1:numClasses % predicted class
                            
                            % Select columns corresponding to predicted class j, sum up
                            % all the matches, and aggregate the results into the confusion
                            % matrix.
                            predEqualsGTClass = trueClassSamples( ...
                                ismember(metrics.PredTableAllClass.PredLabel(truthClassI_rows), classes(j)));
                            numPredEqualsGTClass = sum(predEqualsGTClass, 'all');
                            confusionMatrix{sIdx,tIdx}(i,j) = confusionMatrix{sIdx,tIdx}(i,j) + numPredEqualsGTClass;
    
                            % False positive predictions
                            %   predicted class j
                            %   truth class is background (numClasses+1)
                            unmatchedPredClassJ = (metrics.PredTableAllClass.PredLabel == classes(j)) & falsePositive_rows;
                            confusionMatrix{sIdx,tIdx}(numClasses+1,j) = sum(unmatchedPredClassJ, 'all');
                        end
                    end                
                end
                if options.Normalize
                    rowSum = sum(confusionMatrix{sIdx,tIdx}, 1);
                    confusionMatrix{sIdx,tIdx} = confusionMatrix{sIdx,tIdx} ./ rowSum;
                end
            end
            confusionClassNames = cat(1, metrics.ClassNames, {'unmatched'});
        end

    end

    
    % Helper methods
    methods (Access = private)
        function obj = calculateDataForPrecisionRecall(obj)

            predImageIdx       = [];
            predictionScore    = [];
            objSimilarityScore = [];
            assignedTruthIdx   = [];
            objThetaDiff       = [];
            predArea           = [];
            truthArea          = [];
            truthLabel         = [];
            truthImageIdx      = [];
            assignedLabel      = categorical([]);
            predLabel          = categorical([]);
            isGroundTruthAssigned = [];

            % Hold information on "all classes matching"
            objSimilarityScoreAllClass = [];
            assignedLabelAllClass = [];
            assignedTruthIdxAll = [];
            isGroundTruthAssignedAll = [];

            obj.Printer.linebreak();
            iPrintHeader(obj.Printer,obj.TaskName);
            msg = iPrintInitProgress(obj.Printer,'', 1);

            if(height(obj.PredDS)~=height(obj.TruthDS))
                error(message('vision:objectMetrics:dsLengthMismatch'));
            end

            imIdx = 0;
            %Loop through all images
            while(obj.PredDS.hasdata()||obj.TruthDS.hasdata())
                
                % Handle datastores of unequal length
                if((obj.PredDS.hasdata()&&~obj.TruthDS.hasdata())||...
                    (~obj.PredDS.hasdata()&&obj.TruthDS.hasdata()))

                    error(message('vision:objectMetrics:dsLengthMismatch'));
                end
            
                imIdx = imIdx+1;
            
                % Read predictions and truth
                pred  = read(obj.PredDS);
                truth = read(obj.TruthDS);
            
                predObjList = pred{1};
                predLabelInImg = pred{2};
                predScore = pred{3};
                
                % Sort the predictions based on score
                [predScore, sortIdx] = sort(predScore,'descend');
                predLabelInImg = predLabelInImg(sortIdx);
                predObjList = obj.getObjFromList(predObjList,sortIdx);

                numPred = length(predLabelInImg);

                truthObjList = truth{1};
                truthLabelInImg = truth{2};

                existingGTCount = numel(truthArea);
                truthArea = [truthArea; objArea(obj,truthObjList)];
                truthLabel = [truthLabel; truthLabelInImg];
                truthImageIdx = [truthImageIdx; repmat(imIdx, [length(truthLabelInImg) 1])];
                
                if(~isempty(predObjList))
                    % Match predictions and ground-truth using overlap
                    % criterion specified in obj.SimilarityFcn
                    % - per-class matching following MS-COCO mAP protocol
                    [assignedLabelInImg,assignedOverlapInImg,assignedGTIdxInImg,isGTAssignedInImg] = ...
                        computePerClassMatchingInImage(obj,...
                        predLabelInImg,predObjList,truthLabelInImg,truthObjList);

                    % Compute matchings with all classes together.
                    % This is required to compute inter-class
                    % mis-classifications for the confusion matrix.
                    [assignedLabelInImgAll, assignedOverlapInImgAll, assignedGTIdxInImgAll, isGTAssignedInImgAll] = ...
                        computeAllClassMatchingInImage(obj,...
                        predLabelInImg,predObjList,truthLabelInImg,truthObjList);

                else
                    % Print number of processed images
                    msg = iPrintProgress(obj.Printer, msg, imIdx);

                    % With no predictions in this image, all ground truth
                    % in that image remain un-assigned
                    % - per-class matching, COCO-style
                    isGroundTruthAssigned = [isGroundTruthAssigned; false(length(truthLabelInImg), 1)];
                    % - all-class matching for confusion matrices
                    isGroundTruthAssignedAll = [isGroundTruthAssignedAll; false(length(truthLabelInImg), 1)];

                    continue;
                end

                if obj.ReturnAOS
                    predOrientation = predObjList(:,5);
                    matchedOrientation = zeros(size(predOrientation));
                    matchedOrientation(assignedGTIdxInImg>0) = truthObjList(assignedGTIdxInImg(assignedGTIdxInImg>0),5);
                    imageThetaDiff = predOrientation - matchedOrientation;
                    objThetaDiff = [objThetaDiff; imageThetaDiff];
                end

                % concatenate information for each prediction to the global list
                % - step 1: prediction matching information
                objSimilarityScore = [objSimilarityScore; assignedOverlapInImg];
                predLabel = [predLabel; predLabelInImg];
                assignedLabel = [assignedLabel; assignedLabelInImg];
                predictionScore = [predictionScore; predScore];
                predImageIdx = [predImageIdx; repmat(imIdx, [length(predLabelInImg) 1])];
                predArea = [predArea; objArea(obj,predObjList)];
                % - step 2: ground-truth matching information
                assignedGTIdxGlobal = assignedGTIdxInImg;
                assignedGTIdxGlobal(assignedGTIdxInImg>0) = assignedGTIdxGlobal(assignedGTIdxInImg>0) + existingGTCount;
                assignedTruthIdx = [assignedTruthIdx; assignedGTIdxGlobal];
                isGroundTruthAssigned = [isGroundTruthAssigned; isGTAssignedInImg];

                % for all classes matchings
                % - step 1: prediction matching information
                objSimilarityScoreAllClass = [objSimilarityScoreAllClass; assignedOverlapInImgAll];
                assignedLabelAllClass = [assignedLabelAllClass; assignedLabelInImgAll];
                % - step 2: ground-truth matching information
                assignedGTIdxAllGlobal = assignedGTIdxInImgAll;
                assignedGTIdxAllGlobal(assignedGTIdxInImgAll>0) = ...
                    assignedGTIdxAllGlobal(assignedGTIdxInImgAll>0) + existingGTCount;
                assignedTruthIdxAll = [assignedTruthIdxAll; assignedGTIdxAllGlobal];
                isGroundTruthAssignedAll = [isGroundTruthAssignedAll; isGTAssignedInImgAll];

                % Print number of processed images
                msg = iPrintProgress(obj.Printer, msg, imIdx);
            end

            % Assemble *per-class* prediction and ground truth vectors into tables
            obj.PredTable = table(predImageIdx, predLabel, predictionScore, objSimilarityScore, assignedLabel, assignedTruthIdx, predArea, ...
                VariableNames=["ImgIdx" "PredLabel" "Score" "OverlapScore" "AssignedLabel" "AssignedTruthIdx" "Area"]);
            if obj.ReturnAOS
                obj.PredTable = addvars(obj.PredTable, objThetaDiff, NewVariableNames = "ThetaDiff");
            end

            if isempty(isGroundTruthAssigned) && ~isempty(truthLabel)
                isGroundTruthAssigned = false(size(truthLabel));
            end

            obj.TruthTable = table(truthImageIdx, truthLabel, isGroundTruthAssigned, truthArea,...
                VariableNames=["ImgIdx" "TruthLabel" "IsAssigned" "Area"]);
            
            obj.NumImages = imIdx;
            
            % Assemble *all-classes* prediction and ground truth vectors
            obj.PredTableAllClass = table(predImageIdx, predLabel, predictionScore, ...
                objSimilarityScoreAllClass, assignedLabelAllClass, assignedTruthIdxAll, predArea, ...
                VariableNames=["ImgIdx" "PredLabel" "Score" "OverlapScore" "AssignedLabel" "AssignedTruthIdx" "Area"]);

            if isempty(isGroundTruthAssignedAll) && ~isempty(truthLabel)
                isGroundTruthAssignedAll = false(size(truthLabel));
            end

            obj.TruthTableAllClass = table(truthImageIdx, truthLabel, isGroundTruthAssignedAll, truthArea,...
                VariableNames=["ImgIdx" "TruthLabel" "IsAssigned" "Area"]);

        end


        function [assignedLabelInImg,assignedOverlapInImg,assignedGTIdxInImg,isGTAssignedInImg] = ...
                computeAllClassMatchingInImage(obj,predLabelInImg,predObjList,...
                    truthLabelInImg,truthObjList)
            % Match predictions to groundtruths considering all classes together

            numPred = length(predLabelInImg);
            
            % Generate object overlap ratio table for each pred-truth pair
            overlapRatio = obj.SimilarityFcn(predObjList, truthObjList);
            
            % Assign predictions to groundtruth and get corresponding overlaps/Similarities.
            % For duplicate predictions, assign prediction with highest IOU
            if(~isempty(truthLabelInImg))
                [assignedLabelInImg, assignedOverlapInImg, assignedGTIdxInImg, isGTAssignedInImg] ...
                    = obj.assignGroundTruth(overlapRatio, truthLabelInImg);
            else
                assignedLabelInImg = repmat(categorical(missing), [numPred 1]);
                assignedOverlapInImg = zeros(numPred,1);
                assignedGTIdxInImg = zeros(numPred,1);
                isGTAssignedInImg = false(size(overlapRatio,2),1);
            end
        end
        

        function [assignedLabelInImg,assignedOverlapInImg,assignedGTIdxInImg,isGTAssignedInImg] = ...
                computePerClassMatchingInImage(obj,predLabelInImg,predObjList,...
                    truthLabelInImg,truthObjList)
            % Match predictions to groundtruths, one class at a time, 
            % following the matching logic used in the MS-COCO Evaluation API.
            % (https://github.com/cocodataset/cocoapi/tree/master)

            % Pre-allocate space for pred-truth matchings in the image
            numPred = length(predLabelInImg);
            assignedLabelInImg = repmat(categorical(missing), [numPred 1]);
            assignedOverlapInImg = zeros(numPred,1);
            assignedGTIdxInImg = zeros(numPred,1);
            isGTAssignedInImg = false(length(truthLabelInImg),1);
            
            for clsIdx = 1:obj.NumClasses
                clsName = obj.ClassNames(clsIdx);

                % Select predictions and ground-truths belonging to
                % a specific class
                selectPredInImgPerClass = predLabelInImg==clsName;
                selectTruthInImgPerClass = truthLabelInImg==clsName;
                predObjListInClass = obj.getObjFromList(predObjList,...
                    selectPredInImgPerClass);
                truthObjListInClass = obj.getObjFromList(truthObjList,...
                    selectTruthInImgPerClass);

                % Used for mapping per-class groundtruth sub-array to the 
                % full groundtruth labels array
                mapPerClassToFullGTIndex = dictionary();
                mapPerClassToFullGTIndex(0) = 0;
                if nnz(selectTruthInImgPerClass) > 0
                    keys = 1:nnz(selectTruthInImgPerClass);
                    values =  find(selectTruthInImgPerClass);
                    values = reshape(values, size(keys));
                    mapPerClassToFullGTIndex(keys) = values;
                end

                % Select predictions and truths for this class and generate
                % object overlap ratio matrix for each pred-truth pair.
                % overlapRatio here is the similarity score, computed
                % per-class.
                overlapRatio = obj.SimilarityFcn(predObjListInClass,...
                    truthObjListInClass); 

                % Assign predictions to groundtruth per-class 
                % and get corresponding overlaps/Similarities.
                % For duplicate predictions, assign prediction 
                % with higest IOU.
                truthLabelInImgPerClass = truthLabelInImg(selectTruthInImgPerClass);
                if(~isempty(truthLabelInImg) && ~isempty(truthLabelInImgPerClass)...
                        && nnz(selectPredInImgPerClass) > 0)
                    [assignedLabelInImgPerClass, assignedOverlapInImgPerClass, assignedGTIdxInImgPerClass, isGTAssignedInImgPerClass] = ...
                        obj.assignGroundTruth(overlapRatio, truthLabelInImgPerClass);
                    
                    % Map the per-class pred-gt assignments to
                    % full-length predictions array positions
                    assignedLabelInImg(selectPredInImgPerClass) = assignedLabelInImgPerClass;
                    assignedOverlapInImg(selectPredInImgPerClass) = assignedOverlapInImgPerClass;
                    assignedGTIdxInImg(selectPredInImgPerClass) = mapPerClassToFullGTIndex(assignedGTIdxInImgPerClass);
                    isGTAssignedInImg(selectTruthInImgPerClass) = isGTAssignedInImgPerClass;
                end
            end

        end


        function [assignedLabels, assignedOverlap, assignedObjIdx,isGTAssigned] = assignGroundTruth(~, overlapRatio, truthLabels)
        % assignGroundTruth assigns groundtruth to the predictions based on the
        % similarity score defined in the overlapRatio matrix. The overlapRatio
        % matrix holds the similarity scores for every prediction-gtruth pair. With
        % predictions along the rows and gTruth along the colums.
        % Outputs:
        % assignedLabels - A numPrediction-by-1 vector holding the label of 
        %                  gTruth Object assigned to the prediction.
        % assignedOverlap- A numPrediction-by-1 vector holding the overlap/ 
        %                  similarity score between each prediction and its
        %                  assigned gTruth.
        % assignedObjIdx - A numPrediction-by-1 vector holding the index of 
        %                  gTruth Object assigned to the prediction.
        %
        % isGTAssigned   - A numTruth-by-1 boolean vector that is true when
        %                  that truth object is matched to a prediction,
        %                  and false otherwise.
            
            numPred = size(overlapRatio,1);
            assignedOverlap = zeros(numPred,1);
            assignedObjIdx = zeros(numPred,1);
            
            % track which gts were matched with at least one detection
            isGTAssigned = false(size(overlapRatio,2),1);
            
            % greedily assign detections to ground truth
            for i = 1:numPred        
                
                [maxOverLap, gtIdx] = max( overlapRatio(i,:) );

                if(maxOverLap <= 0)
                    continue;
                end

                assignedObjIdx(i) = gtIdx;
                assignedOverlap(i) = maxOverLap;
                isGTAssigned(gtIdx) = true;
                % remove gt from future consideration. This penalizes multiple
                % detections that overlap one ground truth box, i.e. it is considered
                % a false positive.
                overlapRatio(:,gtIdx) = -inf;
            end
            
            assignedLabels  = arrayfun(@(idx)catLabelFromTruthIdx(idx), assignedObjIdx);

            function label = catLabelFromTruthIdx(idx)
                if (idx==0)
                    label = categorical(missing);
                else
                    label = truthLabels(idx);
                end
            end
        end

        function confusionMatrix = aggregateConfusionMetric(obj)
            numClasses = obj.NumClasses;
            classes = categorical(1:numClasses, 1:numClasses, obj.ClassNames);

            numThresh = obj.NumThreshold;
            
            % The extra class represents the implicit "background" class,
            % accounting for false positives and false negatives.
            confusionMatrix = zeros(numClasses+1, numClasses+1, numThresh);

            for tIdx = 1:numThresh
                trueSamples = obj.PredTableAllClass.OverlapScore >= obj.OverlapThreshold(tIdx); 

                % All predictions that are unmatched with a ground-truth at
                % this threshold
                falsePositive_rows = ismissing(obj.PredTableAllClass.AssignedLabel)...
                    | (obj.PredTableAllClass.OverlapScore < obj.OverlapThreshold(tIdx));

                for i = 1:numClasses % true class

                    if isempty(obj.PredTableAllClass)
                        continue
                    end

                    if isempty(obj.TruthTableAllClass)
                        continue
                    end

                    % Select rows corresponding to ground truth class i.
                    truthClassI_rows = (obj.PredTableAllClass.AssignedLabel == classes(i));
                    trueClassSamples = trueSamples(truthClassI_rows ,:);

                    % False negatives for this truth class
                    %   Case 1: truth class i, no matched prediction (unassigned ground truths)
                    numUnassignedClassI_truths = sum(...
                        (obj.TruthTableAllClass.IsAssigned == 0)...
                        & (obj.TruthTableAllClass.TruthLabel == classes(i)), 'all');

                    %   Case 2: a prediction is assigned to truth class i
                    %   but falls below the overlap score threshold
                    selectPredsBelowThreshold_ClassI = (obj.PredTableAllClass.OverlapScore < obj.OverlapThreshold(tIdx))...
                        & (obj.PredTableAllClass.AssignedLabel == classes(i));
                    %   How many newly un-assigned ground-truths does this
                    %   cause?
                    numUnassignedClassIAtThreshold_truths = numel(...
                        unique( obj.PredTableAllClass.AssignedTruthIdx(selectPredsBelowThreshold_ClassI) ) );
                    confusionMatrix(i,numClasses+1, tIdx) = numUnassignedClassI_truths + numUnassignedClassIAtThreshold_truths;
                    
                    for j = 1:numClasses % predicted class
                        
                        % Select columns corresponding to predicted class j, sum up
                        % all the matches, and aggregate the results into the confusion
                        % matrix.
                        predEqualsGTClass = trueClassSamples( ismember(obj.PredTableAllClass.PredLabel(truthClassI_rows), classes(j)));
                        numPredEqualsGTClass = sum(predEqualsGTClass, 'all');
                        confusionMatrix(i,j, tIdx) = confusionMatrix(i,j, tIdx) + numPredEqualsGTClass;

                        % False positive predictions
                        %   predicted class j
                        %   truth class is background (numClasses+1)
                        unmatchedPredClassJ = (obj.PredTableAllClass.PredLabel == classes(j)) & falsePositive_rows;
                        confusionMatrix(numClasses+1,j, tIdx) = sum(unmatchedPredClassJ, 'all');
                    end
                end                
            end
        end

        function normConfMat = computeNormalizedConfusionMatrix(obj)
            objSum = sum(obj.ConfusionMatrix, 1);
            normConfMat = obj.ConfusionMatrix./objSum;
        end

        function classMetrics = computeClassMetrics(obj)
            
            numClass = obj.NumClasses;
            numThresh = obj.NumThreshold;
            % Initialize table
            classMetrics = table('Size', [numClass, 5],...
                                     'RowNames', obj.ClassNames,...
                                     'VariableNames', {'NumObjects', 'APOverlapAvg', 'AP', 'Precision', 'Recall'},...
                                     'VariableTypes',{'double' 'double', 'cell', 'cell', 'cell'});
            
            if obj.ReturnLAMR
                LAMROverlapAvg = zeros(numClass, 1);
                LAMR = cell(numClass, 1);
                MR = cell(numClass, 1);
                FPPI = cell(numClass, 1);
                classMetrics = addvars(classMetrics, LAMROverlapAvg, LAMR, MR, FPPI);
            end

            if obj.ReturnAOS
                AOSOverlapAvg = zeros(numClass, 1);
                AOS = cell(numClass, 1);
                OrientationSimilarity = cell(numClass, 1);
                classMetrics = addvars(classMetrics, AOSOverlapAvg, AOS, OrientationSimilarity);
            end

            % Compute precision-recall for each class
            for idx = 1:numClass
            
                % While computing mAP for each class we filter the
                % predictions based on the predicted label (vs ground
                % truth), because the model is being evaluated based on
                % predicted values.
                predClassIdx = obj.PredTable.PredLabel==obj.ClassNames(idx);
                predsInClass = obj.PredTable(predClassIdx,:);

                truthClassIdx = obj.TruthTable.TruthLabel==obj.ClassNames(idx);
                truthsInClass = obj.TruthTable(truthClassIdx,:);
            
                % numImages is number of images containing the class
                numImages = numel(unique(truthsInClass.ImgIdx));
                numTruth = height(truthsInClass);
                numPreds = height(predsInClass);

                AP = zeros(numThresh,1);
                LAMR = zeros(numThresh,1);
                precision = zeros(numThresh,numPreds+1);
                recall = zeros(numThresh,numPreds+1);
                MR = zeros(numThresh,numPreds);
                FPPI = zeros(numThresh,numPreds);
                AOS = zeros(numThresh,1);
                OrientationSimilarity = zeros(numThresh,numPreds+1);

                for threshIdx = 1:numThresh

                    truePredictions = predsInClass.OverlapScore>=obj.OverlapThreshold(threshIdx) & predsInClass.AssignedLabel==obj.ClassNames(idx);

                    [AP(threshIdx), precision(threshIdx,:), recall(threshIdx,:)] =...
                        vision.internal.detector.detectorPrecisionRecall(truePredictions, numTruth, predsInClass.Score, true);

                    if obj.ReturnLAMR
                        [LAMR(threshIdx), MR(threshIdx,:), FPPI(threshIdx,:)] = ...
                            vision.internal.detector.detectorMissRateFalsePositivesPerImage(truePredictions, predsInClass.Score, numTruth, numImages, true);
                    end

                    if obj.ReturnAOS
                        [AOS(threshIdx), ~, OrientationSimilarity(threshIdx,:), ~, ~] = ...
                             vision.internal.detector.detectorAOS(truePredictions, predsInClass.ThetaDiff, numTruth, predsInClass.Score, true);
                    end
                    
                end
                classMetrics(idx, 1:5) = {numTruth, mean(AP,'omitnan'), {AP}, {precision}, {recall}};
                if obj.ReturnLAMR
                    classMetrics(idx, ["LAMROverlapAvg" "LAMR" "MR" "FPPI"]) ={mean(LAMR,'omitnan'), {LAMR}, {MR}, {FPPI}};
                end

                if obj.ReturnAOS
                    classMetrics(idx, ["AOSOverlapAvg" "AOS" "OrientationSimilarity"]) ={mean(AOS,'omitnan'), {AOS}, {OrientationSimilarity}};
                end
            end
        end

        function imageMetrics = computeImageMetrics(obj)
            
            numImages = obj.NumImages;
            numThresh = obj.NumThreshold;
            numClass = obj.NumClasses;

            % Initialize table
            imageIdx = num2str((1:numImages)');
            rowNames = "" + imageIdx;
            imageMetrics = table('Size', [numImages, 3],...
                                     'RowNames', rowNames,...
                                     'VariableNames', {'NumObjects', 'APOverlapAvg', 'AP'},...
                                     'VariableTypes',{'double', 'double', 'cell'});

            if obj.ReturnLAMR
                LAMROverlapAvg = zeros(numImages, 1);
                LAMR = cell(numImages, 1);
                imageMetrics = addvars(imageMetrics, LAMROverlapAvg, LAMR);
            end

            if obj.ReturnAOS
                AOSOverlapAvg = zeros(numImages, 1);
                AOS = cell(numImages, 1);
                imageMetrics = addvars(imageMetrics, AOSOverlapAvg, AOS);
            end
            
            for imgIdx = 1:numImages
                % Extract stats for curr Image
                currImgIdx = obj.PredTable.ImgIdx==imgIdx;
                predsInImg = obj.PredTable(currImgIdx,:);
                
                imageAP = zeros(numThresh,1);
                imageLAMR = zeros(numThresh,1);
                imageAOS = zeros(numThresh,1);

                for threshIdx = 1:numThresh

                    AP = zeros(numClass,1);
                    LAMR = zeros(numClass,1);
                    AOS = zeros(numClass,1);
                    numTruthInImg = 0;
                    % Compute precision-recall for each class
                    for c = 1:numClass
                    
                        classIdx = predsInImg.PredLabel==obj.ClassNames(c);
                        predsInClassInImg = predsInImg(classIdx,:);
                    
                        numTruth = nnz(obj.TruthTable.TruthLabel == obj.ClassNames(c) & obj.TruthTable.ImgIdx == imgIdx);
                        numTruthInImg = numTruthInImg + numTruth;

                        truePredictions = predsInClassInImg.OverlapScore >= obj.OverlapThreshold(threshIdx)&...
                                          obj.ClassNames(c) == predsInClassInImg.AssignedLabel;
        
                        AP(c) = vision.internal.detector.detectorPrecisionRecall(...
                            truePredictions, numTruth, predsInClassInImg.Score, true);

                        if obj.ReturnLAMR
                            LAMR(c) = vision.internal.detector.detectorMissRateFalsePositivesPerImage( ...
                                truePredictions, predsInClassInImg.Score, numTruth, 1, true);
                        end
                        if obj.ReturnAOS
                            AOS(c) = vision.internal.detector.detectorAOS( ...
                                truePredictions, predsInClassInImg.ThetaDiff, numTruth, predsInClassInImg.Score, true);
                        end
                    end
                    
                    imageAP(threshIdx) = mean(AP,'omitnan');
                    imageLAMR(threshIdx) = mean(LAMR,'omitnan');
                    imageAOS(threshIdx) = mean(AOS,'omitnan');

                end

                imageMetrics(imgIdx, 1:3) = {numTruthInImg, mean(imageAP,'omitnan'), {imageAP}};
                if obj.ReturnLAMR
                    imageMetrics(imgIdx, ["LAMROverlapAvg" "LAMR"]) = {mean(imageLAMR,'omitnan'), {imageLAMR}};
                end
                if obj.ReturnAOS
                    imageMetrics(imgIdx, ["AOSOverlapAvg" "AOS"]) = {mean(imageAOS,'omitnan'), {imageAOS}};
                end
            end

        end

        function dataSetMetrics = computeDatasetMetrics(obj)

            % Initialize table
            % - mAP: mean over all classes
            % - mAPOverlapAvg: mean over all classes, and over all Overlaps
            dataSetMetrics = table('Size', [1, 3],...
                                     'VariableNames', {'NumObjects', 'mAPOverlapAvg', 'mAP'},...
                                     'VariableTypes',{'double', 'double', 'cell'});
            
            mAPOverlapAvg = mean(obj.ClassMetrics.APOverlapAvg, 'all','omitnan');
            mAP  = {mean([obj.ClassMetrics.AP{:}], 2,'omitnan')};
            
            dataSetMetrics(1, 1:3) = {sum(obj.ClassMetrics.NumObjects), mAPOverlapAvg, mAP};

            if obj.ReturnLAMR
                mLAMROverlapAvg = mean(obj.ClassMetrics.LAMROverlapAvg, 'all','omitnan');
                mLAMR = {mean([obj.ClassMetrics.LAMR{:}], 2,'omitnan')};
                dataSetMetrics = addvars(dataSetMetrics, mLAMROverlapAvg, mLAMR);
            end

            if obj.ReturnAOS
                mAOSOverlapAvg = mean(obj.ClassMetrics.AOSOverlapAvg, 'all','omitnan');
                mAOS = {mean([obj.ClassMetrics.AOS{:}], 2,'omitnan')};
                dataSetMetrics = addvars(dataSetMetrics, mAOSOverlapAvg, mAOS);
            end
        end
    end
end

%--------------------------------------------------------------------------
function iPrintHeader(printer, taskName)
    printer.printMessage('vision:objectMetrics:verboseHeader',taskName);
    printer.print('----------------------------------------');
    printer.linebreak();
end

%--------------------------------------------------------------------------
function iPrintFinalize(printer)
    printer.linebreak();    
    printer.printMessage('vision:objectMetrics:verboseFinalizeTxt');
    printer.linebreak();
end

%--------------------------------------------------------------------------
function updateMessage(printer, prevMessage, nextMessage)
    backspace = sprintf(repmat('\b',1,numel(prevMessage))); % figure how much to delete
    printer.print([backspace nextMessage]);
end

%--------------------------------------------------------------------------
function nextMessage = iPrintInitProgress(printer, prevMessage, k)
    nextMessage = getString(message('vision:objectMetrics:verboseProgressTxt',k));
    updateMessage(printer, prevMessage(1:end-1), nextMessage);
end

%--------------------------------------------------------------------------
function nextMessage = iPrintProgress(printer, prevMessage, k)
    nextMessage = getString(message('vision:objectMetrics:verboseProgressTxt',k));
    updateMessage(printer, prevMessage, nextMessage);
end

function iValidateAreaRanges(in)
    if isvector(in) && numel(in)>1
        validateattributes(in, {'double'},{'increasing'});
    elseif size(in,2)==2
        rangeWidth = diff(in');
        if ~all(rangeWidth>0)
            error(message('vision:objectMetrics:areaRangeNonIncreasing'))
        end
    else
        error(message('vision:objectMetrics:invalidAreaRangeSize'))
    end    
end

function iValidateClassNameInput(in,obj)
    if numel(in) ~= numel(unique(in))
        error(message('vision:objectMetrics:invalidDuplicateClassNames'))
    end
    if ~all(ismember(in,obj.ClassNames))
        error(message('vision:objectMetrics:nonPresentClassNames'))
    end
end

function iValidateUniqueScoreThresholdInput(in)
    if numel(in) ~= numel(unique(in))
        error(message('vision:objectMetrics:invalidDuplicateScoreThresholds'))
    end
end

function iValidateOverlapThresholdInput(in,obj)
    if numel(in) ~= numel(unique(in))
        error(message('vision:objectMetrics:invalidDuplicateOverlapThresholds'))
    end
    if ~all(ismember(in,obj.OverlapThreshold))
        error(message('vision:objectMetrics:nonPresentOverlapThresholds'))
    end
end