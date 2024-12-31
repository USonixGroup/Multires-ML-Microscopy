classdef reidentificationMetrics
%

%   Copyright 2023 The MathWorks, Inc.

    properties (SetAccess = protected)
        % ClassMetrics contains the metrics computed for each class.
        % The output here is a numClasses-by-numMetrics table. By default, 
        % the following metrics are returned per row:
        %
        % - mAP          : Mean average precision are the average precision
        %                  values averaged over all queries and precision
        %                  ranks for a class.
        % - AP           : Average precision computed over all the queries
        %                  for a class. This is a 1-by-numQueries array.
        % - Recall       : Recall computed over all the queries for a class.
        %                  This is a 1-by-numQueries array.
        % - CMC (Rank 1) : Cumulative matching characteristic computed over
        %                  all queries for a class. By default, only rank 1
        %                  CMC is returned. Additional ranks are returned
        %                  when specified.
        ClassMetrics

        % DataSetMetrics contains the metrics averaged over the entire
        % dataset. This is returned as a 1-by-numMetrics table. 
        % By default, the following metrics are returned:
        %
        % - mAP          : Mean average precision are the average precision
        %                  values averaged over all queries and precision
        %                  ranks for all classes.
        % - AP           : Average precision computed over all the queries
        %                  for all classes. This is a 1-by-numQueries array.
        % - Recall       : Recall computed over all the queries for all
        %                  classes. This is a 1-by-numQueries array.
        % - CMC (Rank 1) : Cumulative matching characteristic computed over
        %                  all queries for a class. By default, only rank 1
        %                  CMC is returned. Additional ranks are returned
        %                  when specified.
        DatasetMetrics

        % ClassNames contains a vector of string holding the class names of
        % the objects.
        ClassNames
    end

    % Configuration properties.
    properties (Access = protected)
        ImageFeatures           % All features extracted from ReID network.
        ImageLabels             % All labels of the given features.
        Queries                 % Index of all queries made during evaluation.
        MultiInstanceGalleries  % Galleries per query, includes all images except query.
        SingleInstanceGalleries % Galleries per query, includes one instance per class.
        DistanceMetric          % Metric used to calculate distance between features.
        RandomQueryOrder        % Flag to control query order.
        Verbose                 % Flag to enable verbose display.
    end

    % Cached properties to compute and assign CMC and precision-recall metrics.
    properties (Access = protected)
        CMC
        ClassAP
        ClassCMC
        ClassIndices
        ClassMeanAveragePrecision
        ClassPrecision
        ClassRecall
        FillPrecision
        MeanAveragePrecision
        NumQueries
        NumImages
        NumClasses
        Printer
        Rank
    end

    methods (Hidden = true, Access = protected)
        function obj = reidentificationMetrics(features, labels, ...
                distanceMetric, ranks, numQueries, randomizeQueries, verbose)

            % Configuration setup.
            obj.ImageFeatures    = features;
            obj.ImageLabels      = labels;
            obj.DistanceMetric   = distanceMetric;
            obj.NumQueries       = numQueries;
            obj.RandomQueryOrder = randomizeQueries;
            obj.FillPrecision    = true;
            obj.ClassNames       = unique(labels,'stable');
            obj.NumClasses       = numel(obj.ClassNames);
            obj.NumImages        = size(obj.ImageLabels,2);
            obj.Verbose          = verbose;

            obj.Printer = vision.internal.MessagePrinter.configure(verbose);
            iPrintHeader(obj.Printer);

            % Determine the number of CMC ranks to display.
            if strcmpi(ranks, "All")
                obj.Rank = 1:obj.NumClasses;
            else
                obj.Rank = ranks;
            end

            % Find which indices correspond to each class.
            obj.ClassIndices = findClassIndices(obj);

            % Set queries and build all galleries for each query.
            obj = setQueries(obj);
            obj = buildGalleries(obj);

            % Calculate the CMC for all ranks.
            obj = calculateCumulativeMatchingCharacteristic(obj);

            % Calculate precision and recall for all queries.
            obj = calculatePrecisionAndRecall(obj);

            iPrintFinalize(obj.Printer);

            % Aggregate all metrics.
            obj.ClassMetrics = aggregateClassMetrics(obj);
            obj.DatasetMetrics = aggregateDatasetMetrics(obj);

        end
    end

    methods
        function H = plot(metrics, plotType, options)
            % plot Plot the cumulative matching characteristic (CMC) or precision-recall curve.
            %
            % plot(metrics,"cmc") plots the CMC curve over the data set for the given
            % reidentificationMetrics object. metrics is a reidentificationMetrics object
            % returned by evaluateReidentificationNetwork.
            %
            % plot(metrics,"cmc-per-object") alternatively plots the CMC curve per
            % object class for the given reidentificationMetrics object.
            %
            % plot(metrics,"precision-recall") alternatively plots the
            % precision-recall curve for the complete dataset of the given
            % reidentificationMetrics object.
            %
            % Additional input arguments
            % ----------------------------
            % plot(..., Name=Value) specifies additional name-value
            % pair arguments in line with the plot function as well as the following:
            %
            %    Parent              Output axes, specified as an Axes Properties
            %                        graphics object.
            %
            %                        Default: gca
            %
            %    ShowLegend          A boolean that specifies if the plot is to show
            %                        the legend.
            %
            %                        Default: "auto"
            %
            %    LineWidth           Line width in pixels, specified as a positive
            %                        integer. All lines in the CMC plot are set to the
            %                        value specified.
            %
            %                        Default: 2
            %
            %    LineColor           CMC plot line color specified as a short color name,
            %                        color name, vector of color names, three-column
            %                        matrix of RGB triplets. When using the syntax
            %                        plot(metrics,"cmc-per-object"), you can specify a
            %                        different color for each line or one color for all
            %                        lines. To specify one color for all markers, specify
            %                        LineColor as a color name or an [R G B] vector.
            %
            %                        Default: "auto"

            arguments
                metrics {mustBeNonempty, mustBeScalarOrEmpty,mustBeA(metrics,"reidentificationMetrics")}
                plotType {mustBeMember(plotType,["cmc", "cmc-per-object", "precision-recall"])}
                options.ShowLegend {mustBeNumericOrLogical, mustBeFinite, mustBeScalarOrEmpty} = logical.empty
                options.LineWidth {mustBeNumeric, mustBeFinite, mustBePositive, mustBeScalarOrEmpty, mustBeNonempty} = 2
                options.LineColor = []
                options.Parent {mustBeA(options.Parent,"matlab.graphics.axis.Axes")}
            end

            nargoutchk(0,1)

            % Validate color inputs.
            lineColor = iValidateLineColor(options.LineColor,metrics,plotType);

            % Default the plot legend to be shown if the plot type is CMC
            % per object and if there are at most 10 classes.
            if isempty(options.ShowLegend)
                showLegend = strcmp(plotType, "cmc-per-object") && metrics.NumClasses <= 10;
            else
                showLegend = options.ShowLegend;
            end

            if ~isfield(options, 'Parent')
                options.Parent = newplot(figure);
            end

            % Set plot parameters for the given plot type.
            if strcmp(plotType, "precision-recall")
                xData = metrics.DatasetMetrics.Recall{:};
                yData = metrics.DatasetMetrics.AP{:};
                xLimits = [0 1];
                yLimits = [0 1];
                markerStyle = "none";
            else
                xData = 1:metrics.NumClasses;
                xLimits = [0 metrics.NumClasses];
                markerStyle = ".";

                if strcmp(plotType, "cmc")
                    yData = metrics.CMC.*100;
                    yLimits = [max(0,min(yData(:,1))-10) 100];
                else
                    yData = metrics.ClassCMC'.*100;
                    yLimits = [max(0,min(yData(1,:))-10) 100];
                end
            end

            % Plot the CMC or precision-recall curve.
            handle = plot(options.Parent,xData,yData,...
                LineWidth=options.LineWidth,Marker=markerStyle,MarkerSize=20);

            % Set the line colors based off the LineColor NV argument.
            if ~isempty(lineColor)
                % Assign the input colors to the parent color order.
                lineColorMap = colormap(lineColor);
                options.Parent.ColorOrder = lineColorMap;
            end

            % Annotate the plot.
            if strcmp(plotType, "precision-recall")
                xlabel("Recall");
                ylabel("Precision");
                title("Precision-Recall Curve");
            else
                xlabel("Rank{\it k}");
                ylabel("Reidentification Rate (%)");
                title("Cumulative Match Characteristic (CMC) Curve");
            end

            xlim(xLimits);
            ylim(yLimits);

            % Display the plot legend if set.
            if showLegend
                switch plotType
                    case "cmc"
                        legend("CMC Per Rank",Location="southwest",Interpreter="none");
                    case "cmc-per-object"
                        legend(metrics.ClassNames,Location="southeast",Interpreter="none");
                    case "precision-recall"
                        legend("Precision-Recall",Location="southwest",Interpreter="none");
                end
            end

            if nargout > 0
                H = handle;
            end
        end

    end

    methods (Access = private)
        function obj = calculateCumulativeMatchingCharacteristic(obj)
            iPrintEvaluationStage(obj.Printer,"cumulative matching characteristic",0);

            features = obj.ImageFeatures;

            % Normalize the features.
            normFeatures = features./vecnorm(features,2,1);

            % Compute the CMC for each class.
            classCMC = arrayfun(@(idx) calculateCMCPerClass(obj, ...
                obj.ClassNames(idx), normFeatures),1:obj.NumClasses, ...
                UniformOutput=false)';

            % Store the CMC per class and averaged out.
            obj.ClassCMC = vertcat(classCMC{:});
            obj.CMC = mean(obj.ClassCMC,1);
        end

        function obj = calculatePrecisionAndRecall(obj)
            iPrintEvaluationStage(obj.Printer,"precision and recall",0);

            features = obj.ImageFeatures;

            % Normalize the features.
            normFeatures = features./vecnorm(features,2,1);

            % Compute the precision and recall for each class.
            classPrecisionAndRecallMetrics = arrayfun(@(idx) ...
                calculatePrecisionAndRecallPerClass(obj, obj.ClassNames(idx), ...
                normFeatures),1:obj.NumClasses, UniformOutput=false)';

            % Gather class precision and recall metrics.
            classPrecisionAndRecallMetrics = vertcat(classPrecisionAndRecallMetrics{:});
            classMeanAveragePrecision = classPrecisionAndRecallMetrics(:,1);
            classAvgPrecision = classPrecisionAndRecallMetrics(:,2);
            classRecall = classPrecisionAndRecallMetrics(:,3);

            % Store the precision and recall per class and averaged out.
            obj.ClassMeanAveragePrecision = vertcat(classMeanAveragePrecision{:});
            obj.ClassAP = vertcat(classAvgPrecision{:});
            obj.ClassRecall = vertcat(classRecall{:});
            obj.MeanAveragePrecision = mean(obj.ClassMeanAveragePrecision,1);
        end

        function cmc = calculateCMCPerClass(obj, class, features)
            % Compute the CMC for all queries in a given class.
            iPrintEvaluationStage(obj.Printer,"cumulative matching characteristic",class);

            % Initialize CMC arrays.
            classIndices = find(obj.ImageLabels(obj.Queries) == class);
            classQueries = obj.Queries(classIndices);
            classGalleries = obj.SingleInstanceGalleries(:,classIndices);

            % Calculate and sort the galleries for the class based on the
            % given distance metric.
            galleryLabels = sortGalleries(obj,features,classQueries,classGalleries,"cmc");

            % Find where the relevant image is located (what rank) in
            % the sorted gallery.
            [~,matchIdx] = max(galleryLabels == class,[],1);

            % Accumulate the CMC.
            cmcCount = accumarray(matchIdx',1,[],[],0)';
            cmc = cumsum(cmcCount);

            % Average the CMC and since the CMC is a cumulative monotically
            % increasing vector, pad the vector to equal the total number
            % of classes.
            cmc = [cmc./length(classQueries) ones(1,obj.NumClasses-length(cmcCount))];
        end

        function precisionAndRecallMetrics = calculatePrecisionAndRecallPerClass(obj, class, features)
            % Compute the precision and recall for all queries in a given class.
            iPrintEvaluationStage(obj.Printer,"precision and recall",class);

            % Initialize precision and recall arrays.
            classIndices = find(obj.ImageLabels(obj.Queries) == class);
            classQueries = obj.Queries(classIndices);
            classGalleries = obj.MultiInstanceGalleries(:,classIndices);
            numClassInstances = length(classQueries);
            numQueries = obj.NumQueries;
            matchesPerQuery = zeros(numQueries,numClassInstances);
            precisionPerQuery = zeros(numQueries,numClassInstances);

            % Calculate and sort the galleries for the class based on the
            % given distance metric.
            galleryLabels = sortGalleries(obj,features,classQueries,classGalleries,"precision-recall");

            % Find where the relevant images are in the sorted gallery.
            [matchIdx,~] = find(galleryLabels == class);

            % Reshape the matches to a per query format and convert it to
            % linear indexing format.
            matchIdx = reshape(matchIdx,numClassInstances-1,numClassInstances);
            matchLinearIdx = sub2ind(size(precisionPerQuery),matchIdx,repmat(1:size(precisionPerQuery,2),size(matchIdx,1),1));

            % Accumulate number of matches thoughout queries for recall.
            matchesPerQuery(matchLinearIdx) = 1;
            matchIdxCount = sum(matchesPerQuery,2)';

            % Calculate the precision per query.
            precisionPerQuery(matchLinearIdx) = cumsum(ones(size(matchLinearIdx,1),numClassInstances),1);

            % Since precision has a linear scaling based off rank, scale
            % the precision by rank.
            precisionScale = (1:numQueries)';
            precisionPerQuery = precisionPerQuery./precisionScale;

            % Sum the precisions to get the total precision over the class,
            % and average it below.
            p = sum(precisionPerQuery,2)';

            % Average the metrics.
            p = p./numClassInstances;
            ap = sum(p,2)/(numClassInstances-1);
            r = matchIdxCount/(numClassInstances-1);
            r = cumsum(r)/numClassInstances;

            % Append a 1 to the start of precision and a 0 to the start of
            % recall to define the (0,1) point on the precision/recall
            % curve.
            p = [1 p];
            r = [0 r];

            if obj.FillPrecision
                % Convert all zeros with NaNs and replace all NaNs with
                % the most previous valid value.
                p(p == 0) = nan;
                p = fillmissing(p,"previous");
            end

            precisionAndRecallMetrics = {ap, p, r};

        end

        function galleryLabelsSorted = sortGalleries(obj,features,classQueries,classGalleries,metric)
            % Gather all queries and galleries for the current class.
            featureQuery = features(:,classQueries);
            featureGallery = features(:,classGalleries);

            % Reshape the queries and galleries for feature distance calculations.
            featureQuery = reshape(featureQuery,size(features,1),1,length(classQueries));
            if strcmp(metric,"cmc")
                % Galleries for CMC only have one instance of each class.
                featureGallery = reshape(featureGallery,size(features,1),obj.NumClasses,length(classQueries));
            else
                % Galleries for precision/recall have all instances of a
                % class except the query image.
                featureGallery = reshape(featureGallery,size(features,1),obj.NumQueries-1,length(classQueries));
            end

            % Since the queries and galleries are N-D, use pagemtimes to
            % calculate the distance for each query with its respective
            % gallery.
            if strcmp(obj.DistanceMetric,"cosine")
                % Use the cosine distance for comparing feature distance.
                featDist = 1-pagemtimes(featureGallery,"transpose",featureQuery,"none");
            else
                % Use euclidean squared distance for comparing feature
                % distance.
                featDist = sum((featureGallery-featureQuery).^2, 1);
                featDist = permute(featDist,[2 1 3]);
            end

            % Sort the galleries from closest to furthest distance.
            [~,sortedIdx]=sort(featDist);
            sortedIdx = squeeze(sortedIdx);
            galleryLabels = obj.ImageLabels(classGalleries);
            sortedLinearIdx = sub2ind(size(galleryLabels),sortedIdx,repmat(1:size(galleryLabels,2),size(sortedIdx,1),1));
            galleryLabelsSorted = galleryLabels(sortedLinearIdx);
        end

        function obj = setQueries(obj)
            iPrintEvaluationStage(obj.Printer,"query",0);

            % Set all queries for the given input features.
            if obj.RandomQueryOrder
                queries = randperm(obj.NumImages);
            else
                queries = 1:obj.NumImages;
            end
            obj.Queries = queries(1:obj.NumQueries);
        end

        function obj = buildGalleries(obj)
            iPrintEvaluationStage(obj.Printer,"gallery",0);

            % Build all galleries for every query.
            numImages = obj.NumImages;
            numQueries = obj.NumQueries;
            numClasses = obj.NumClasses;

            % Create numImages multi-instance galleries that exclude one
            % query image per gallery.
            allIndices = repmat(1:numImages,numImages-1,1);
            galleryIdxList = reshape(allIndices,numImages,numImages-1);
            galleryIdxList = sortrows(galleryIdxList,"descend")';

            % Arrange galleries by query order.
            if obj.RandomQueryOrder
                galleryIdxList = galleryIdxList(:,obj.Queries);
            end

            % Reduce the gallery set to the total number requested.
            galleryIdxList = galleryIdxList(:,1:numQueries);

            % Create lists of the corresponding gallery labels.
            labelSets = arrayfun(@(idx) obj.ImageLabels(1,galleryIdxList(:,idx))',1:numQueries,UniformOutput=false);
            labels = [labelSets{:}];

            obj.MultiInstanceGalleries = galleryIdxList;

            % Creat random linear indexer.
            randIdx = cell2mat(arrayfun(@(x) randperm(numImages-1),1:numQueries,UniformOutput=false));
            randIdx = reshape(randIdx,numImages-1,numQueries);
            randLinearIdx = sub2ind(size(galleryIdxList),randIdx,repmat(1:numQueries,numImages-1,1));

            % Apply random indexing to gallery index and label lists.
            galleryIdxList = galleryIdxList(randLinearIdx);
            labels = labels(randLinearIdx);

            % Find the first occurence of each class to build out a
            % single-instance set of galleries.
            [~,firstIndices] = (arrayfun(@(idx) max(labels == obj.ClassNames(idx),[],1),1:numClasses,UniformOutput=false));
            firstIndices = firstIndices';
            firstIndices = vertcat(firstIndices{:});

            % Convert the first occurrence indices to a linear index.
            firstLinearIndices = sub2ind(size(galleryIdxList),firstIndices,repmat(1:numQueries,numClasses,1));

            % Convert found first instances to gallery indices.
            obj.SingleInstanceGalleries = galleryIdxList(firstLinearIndices);

        end

        function classMetrics = aggregateClassMetrics(obj)
            % Create class metrics table.
            classMetrics = table(Size=[obj.NumClasses, 4],...
                                     RowNames=obj.ClassNames,...
                                     VariableNames={'NumInstances', 'mAP', 'AP', 'Recall'},...
                                     VariableTypes={'double' 'double', 'cell', 'cell'});

            % Expand the table to include each rank requested.
            for rank = obj.Rank
                classMetrics = addvars(classMetrics, obj.ClassCMC(:,rank), NewVariableNames=strcat("CMC (Rank ",string(rank),")"));
            end

            % Set the number of instances per class.
            numInstances = cellfun(@(x) size(x,2),obj.ClassIndices,'UniformOutput',false);
            classMetrics{:,1} = [numInstances{:}]';

            % Set class mAP, AP, precision, and recall.
            classMetrics{:,2} = obj.ClassMeanAveragePrecision;
            classMetrics{:,3} = num2cell(obj.ClassAP,2);
            classMetrics{:,4} = num2cell(obj.ClassRecall,2);

        end

        function datasetMetrics = aggregateDatasetMetrics(obj)
            % Create dataset metrics table.
            datasetMetrics = table(Size=[1, 4],...
                                   VariableNames={'NumInstances', 'mAP', 'AP', 'Recall'},...
                                   VariableTypes={'double', 'double', 'cell', 'cell'});

            % Expand the table to include each rank requested.
            for rank = obj.Rank
                datasetMetrics = addvars(datasetMetrics, obj.CMC(:,rank), NewVariableNames=strcat("CMC (Rank ",string(rank),")"));
            end

            % Set the number of instances per class.
            datasetMetrics{:,1} = sum(obj.ClassMetrics.NumInstances);

            % Calculate the average precision and dataset recall.
            averagePrecision = sum(obj.ClassAP,1)/obj.NumClasses;
            recall = sum(obj.ClassRecall,1)/obj.NumClasses;

            % Set the mAP, AP, and recall for the dataset.
            datasetMetrics(:,2:4) = {obj.MeanAveragePrecision, {averagePrecision}, {recall}};
        end

        function classIndices = findClassIndices(obj)
            labels = obj.ImageLabels;
            numClasses = obj.NumClasses;
            classIndices = cell(numClasses,1);

            for i = 1:numClasses
                % Find every index that an object occurs in.
                indices = find(labels == obj.ClassNames{i});

                % Verify that at least two instances of every object exists
                % in the evaluation inputs. Every gallery must have one
                % instance of each object and when querying an object,
                % there must be another instance to place in the gallery.
                if numel(indices) < 2
                    error(message('vision:reidentification:invalidNuminstances'));
                else
                    classIndices{i} = indices;
                end
            end
        end

    end

    methods (Hidden = true, Static = true)
        function obj = compute(features, labels, distanceMetric, ranks,...
                numQueries, randomizeQueries, verbose)
            obj = reidentificationMetrics(features,labels,distanceMetric,...
                ranks, numQueries, randomizeQueries, verbose);
        end
    end

    methods
        function queries =  gatherQueries(obj)
            queries = obj.Queries;
        end

        function gallerySets =  gatherGallerySets(obj)
            gallerySets = obj.SingleInstanceGalleries;
        end
    end
end

%--------------------------------------------------------------------------
% Validation Functions
%--------------------------------------------------------------------------
function lineColor = iValidateLineColor(color, metrics, plotType)
    if ~isempty(color)
        if isnumeric(color)
            validateattributes(color,{'numeric'},{'nonempty','nonsparse',...
                'real','finite','nonnegative','ncols',3,'nonnan'},"plot","LineColor");

            % Convert color to the RGB double.
            color = im2double(color);

            % Confirm double is in the range of [0,1]. This case would
            % occur if the input color is specified as a single or double
            % and was not within the range of [0,1].
            if any([any(color > 1) any(color < 0)])
                error(message('vision:reidentification:invalidLineColorRange'));
            end
        else
            % Convert strings to chars for RGB conversion.
            if isstring(color)
                color = convertStringsToChars(color);
            end

            % Verify color is not hexadecimal.
            if iscell(color)
                colorStart = color{1}(1);
            else
                colorStart = color(1);
            end

            if strcmp(colorStart, "#")
                error(message('vision:reidentification:invalidLineColorHex'));
            end
        end

        % Convert colors to an M-by-3 RGB triplet.
        colorRGB = vision.internal.convertColorSpecToRGB(color);

        % Build out M-by-3 RGB triplet for scalar color inputs.
        if strcmp(plotType,"cmc-per-object")
            if size(colorRGB, 1) == 1
                lineColor = repmat(colorRGB, [metrics.NumClasses 1]);
            elseif size(colorRGB, 1) ~= metrics.NumClasses
                error(message('vision:reidentification:invalidNumLineColors'));
            else
                lineColor = colorRGB;
            end
        else
            if size(colorRGB, 1) ~= 1
                error(message('vision:reidentification:invalidLineColorNotScalar'));
            else
                lineColor = colorRGB;
            end
        end
    else
        lineColor = color;
    end
end

%--------------------------------------------------------------------------
% Verbose Functions
%--------------------------------------------------------------------------
function iPrintHeader(printer)
    printer.printMessage('vision:reidentification:verboseHeader');
    printer.print('-------------------------------------');
    printer.linebreak();
end

%--------------------------------------------------------------------------
function iPrintEvaluationStage(printer,stage,class)
    switch stage
        case "query"
            printer.printMessage('vision:reidentification:verboseSetQueries');
        case "gallery"
            printer.printMessage('vision:reidentification:verboseBuildGallerySets');
        otherwise
            % Calculating CMC or precision and recall.
            if isstring(class)
                % Calculating each class.
                printer.printMessage('vision:reidentification:verboseClassMetricCalculation',stage,class);
            else
                % Entering metric calculation stages.
                printer.linebreak();
                printer.printMessage('vision:reidentification:verboseMetricCalculation',stage);
                printer.print(repelem('-',strlength(stage)+20));
                printer.linebreak();
            end
    end
end

%--------------------------------------------------------------------------
function iPrintFinalize(printer)
    printer.linebreak();
    printer.printMessage('vision:reidentification:verboseFinalizeTxt');
    printer.linebreak();
end