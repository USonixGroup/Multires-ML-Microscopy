classdef ocrMetrics

% Copyright 2022-2023 The MathWorks, Inc.

    properties (SetAccess = protected)
        
        DataSetMetrics
        
        ImageMetrics
    end

    properties(Access = protected)
        
        Version = 1.0;
    end

    properties (Access = private)
        Verbose   % Whether to print to the Command Window
        Printer   % Message printer
        WaitBar   % Progress bar

        WantCharacterErrorRate  % Whether to compute the character error rate.
        WantWordErrorRate  % Whether to compute the word error rate.
        NumImages
    end

    methods (Hidden = true, Static = true)
        %------------------------------------------------------------------
        function this = compute(resultTxt, groundTruthTxt, options)
            this = ocrMetrics();
            this = this.populateProperties(resultTxt, groundTruthTxt, options);
        end

        %--------------------------------------------------------------------------
        function error = computeErrorMetric(truthStr, ocrStr, unit)
        
            if strcmpi(unit, "character")
                truthStr = char(truthStr);
                ocrStr   = char(ocrStr);
            elseif strcmpi(unit, "word")
                truthStr = string(strsplit(truthStr, " "));
                ocrStr   = string(strsplit(ocrStr, " "));
            else
                disp("Incorrect unit");
                error = [];
                return;
            end
        
            error = normLev(ocrStr, truthStr);
        end
    end

    methods (Hidden = true, Access = protected)
        %------------------------------------------------------------------
        function this = ocrMetrics(varargin)
            narginchk(0,1)

            if nargin == 1
                that = varargin{1};

                this.DataSetMetrics = that.DataSetMetrics;
                this.ImageMetrics = that.ImageMetrics;
                this.NumImages = that.NumImages;
                this.Version = that.Version;
            else
                this.DataSetMetrics = table;
                this.ImageMetrics = table;
                this.NumImages = 0;
            end
        end

        %------------------------------------------------------------------
        function this = populateProperties(this, resultTxt, groundTruthTxt, options)

            groundTruthTxtDs = transform(groundTruthTxt, @vision.internal.ocr.validateOCRDataStoreContents);

            if iscell(resultTxt)
                this.NumImages = length(resultTxt);
                resultTxtData = cellfun(@dealOCRText, resultTxt, UniformOutput=false);
                resultTxtDs = vision.internal.cnn.datastore.InMemoryDatastore(resultTxtData);
            else
                validResultTxtDs = transform(resultTxt, @vision.internal.ocr.validateOCRDataStoreContents);
                resultTxtDs = transform(validResultTxtDs, @addConfidenceScoresToDs);
            end

            % Create a MessagePrinter.
            this.Verbose = options.Verbose;
            this.Printer = vision.internal.MessagePrinter.configure(this.Verbose);

            % Parse metrics.
            this = parseMetrics(this, options.Metrics);

            % Compute metrics.
            this = computeMetrics(this, resultTxtDs, groundTruthTxtDs);
        end

        
        %------------------------------------------------------------------
        function this = computeMetrics(this, resultds, gtds)

            this = printHeader(this);

            [this, metrics] = computeMetricsImpl(this, resultds, gtds);

            this = printFooter(this);
            this = finalizeMetrics(this, metrics);
            this = printDone(this);
        end

        %------------------------------------------------------------------
        function this = finalizeMetrics(this, metrics)
            % Finalize metrics based on the intermediate variables

            this.DataSetMetrics = table;
            this.ImageMetrics = table;
    
            if this.WantCharacterErrorRate
                charError = [metrics.CharacterErrorRate]';
                this.ImageMetrics.CharacterErrorRate = charError;
                this.DataSetMetrics.CharacterErrorRate = mean(charError);
            end

            if this.WantWordErrorRate
                wordError = [metrics.WordErrorRate]';
                this.ImageMetrics.WordErrorRate = wordError;
                this.DataSetMetrics.WordErrorRate = mean(wordError);
            end
        end

        %------------------------------------------------------------------
        function [this, metrics] = computeMetricsImpl(this, resultds, gtds)
            cds = combine(resultds, gtds);

            tds = transform(cds, @(data)iForEach(@(x) nAssignDetectionsToGroundTruth(this, x),data));

            msg = iPrintProgress(this.Printer,'',0);
            k = 0;
            metrics = {};
            while hasdata(tds)
                metrics{end+1} = read(tds); %#ok
                k = k + 1;
                msg = iPrintProgress(this.Printer,msg,k);
            end
            this.Printer.linebreak();

            metrics = [metrics{:}];
        end

        %------------------------------------------------------------------
        function [this, metrics] = computeMetricsImplBarProgress(this, resultds, gtds)
            cds = combine(resultds, gtds);

            tds = transform(cds, @(data)iForEach(@nAssignDetectionsToGroundTruth,data));

            this.Printer.printMessage('vision:ocrMetrics:processingNImages', ...
                num2str(this.NumImages));

            % Create a Console Window wait bar.
            this.WaitBar = vision.internal.ConsoleWaitBar( ...
                    this.NumImages, ...     % total number of iterations
                    "Verbose",this.Verbose, ... % whether to print or not
                    "DisplayPeriod",2, ...     % refresh every 2 seconds
                    "PrintElapsedTime",1, ...  % print elapsed time
                    "PrintRemainingTime",1);   % print estimated time remaining

            % Force finish displaying wait bar when we exit this function
            % or if the user sends an interrupt signal.
            cleanUpObj = onCleanup(@() stop(this.WaitBar));

            % Start displaying wait bar
            start(this.WaitBar);

            while hasdata(tds)
                update(this.WaitBar)
                metrics = read(tds);
            end
        end

        %------------------------------------------------------------------
        function metric = nAssignDetectionsToGroundTruth(this, data)
            
            if length(data) == 7
                [~, bbox, scores, txt, ~, gtBbox, gtTxt] = deal(data{:});
            else
                [predictions, ~, gtBbox, gtTxt] = deal(data{:});
                [~, bbox, scores, txt] = deal(predictions{:});
            end
            
            % Remove NaN bounding boxes.
            validDetections = ~isnan(sum(bbox,2));
            numNonDetections = sum(~validDetections);
            bbox = bbox(validDetections,:);
            txt = txt(validDetections);
            scores = scores(validDetections);

            % Match the detections to ground truth.
            overlapThreshold = 0.5;
            [~, ~, assignments] = ...
                vision.internal.detector.assignDetectionsToGroundTruth(bbox, ...
                gtBbox, overlapThreshold, scores);

            % Remove unmatched assigments.
            numNonDetections = numNonDetections + sum(assignments==0);
            txt = txt(assignments~=0);
            assignments = assignments(assignments~=0);
            gtTxt = gtTxt(assignments);

            % Compute the metrics.
            if this.WantCharacterErrorRate
                metric.CharacterErrorRate = evaluateOCRString(txt, gtTxt, MetricUnit = "character");
                
                % Penalize for non-detections.
                metric.CharacterErrorRate = ...
                    (metric.CharacterErrorRate*numel(txt) + 1*numNonDetections) /...
                    (                 numel(txt)          +   numNonDetections  );
            end

            if this.WantWordErrorRate
                metric.WordErrorRate = evaluateOCRString(txt, gtTxt, MetricUnit = "word");

                % Penalize for non-detections.
                metric.WordErrorRate = ...
                    (metric.WordErrorRate*numel(txt) + 1*numNonDetections) /...
                    (            numel(txt)          +   numNonDetections  );
            end
        end
    end

    methods (Hidden = true, Access = protected)
        %------------------------------------------------------------------
        function this = printHeader(this)
            this.Printer.printMessage('vision:ocrMetrics:evaluationHeader');
            N = length(getString(message('vision:ocrMetrics:evaluationHeader')));
            this.Printer.print(repmat('-',1,N));
            this.Printer.linebreak();
            this.Printer.printMessageNoReturn('vision:ocrMetrics:selectedMetrics');
            this.Printer.print(' ');
            printComma = false;

            if this.WantCharacterErrorRate
                this.Printer.printMessageNoReturn('vision:ocrMetrics:characterErrorRate');
                printComma = true;
            end
            if this.WantWordErrorRate
                if printComma
                    this.Printer.print(', ');
                end
                this.Printer.printMessageNoReturn('vision:ocrMetrics:wordErrorRate');
            end

            this.Printer.print('.');
            this.Printer.linebreak();
        end

        %------------------------------------------------------------------
        function this = printFooter(this)
            this.Printer.printMessage('vision:ocrMetrics:finalizingResults');
        end
        
        %------------------------------------------------------------------
        function this = printDone(this)
            this.Printer.print('\b ');
            this.Printer.printMessage('vision:ocrMetrics:done');
            this.Printer.printMessage('vision:ocrMetrics:dataSetMetrics');
            this.Printer.linebreak();
            if this.Verbose
                disp(this.DataSetMetrics)
            end
        end

        %------------------------------------------------------------------
        function this = parseMetrics(this, metrics)
            
            switch metrics
                case "all"
                    this.WantCharacterErrorRate = true;
                    this.WantWordErrorRate = true;
                case "character-error-rate"
                    this.WantCharacterErrorRate = true;
                case "word-error-rate"
                    this.WantWordErrorRate = true;
            end
        end
    end

    methods(Static, Access=private)
        %------------------------------------------------------------------
        % load object
        %------------------------------------------------------------------
        function this = loadobj(that)
            
            this = ocrMetrics(that);
        end
    end
    
    methods(Access=private)
        %------------------------------------------------------------------
        % save object
        %------------------------------------------------------------------
        function that = saveobj(this)
            
            % save properties into struct
            that.DataSetMetrics = this.DataSetMetrics;
            that.ImageMetrics = this.ImageMetrics;
            that.NumImages = this.NumImages;
            that.Version = this.Version;
        end
    end
end

%--------------------------------------------------------------------------
function output = addConfidenceScoresToDs(data)
    
    score = 100*ones(size(data{2},1),1);
    output = {data{1}, data{2}, score, data{3}};
end


%--------------------------------------------------------------------------
function output = dealOCRText(ocrTxt)
    numDetections = length(ocrTxt);
    im = NaN;
    
    bbox = zeros(numDetections, 4);
    scores = zeros(numDetections, 1);
    txt = strings(numDetections, 1);
    for i = 1:numDetections
        if isempty(ocrTxt(i).Text)
            bbox(i, :) = nan(1, 4);
            scores(i) = NaN;
            txt(i) = "";
        else
            numTextLines = numel(ocrTxt(i).TextLines);
            if numTextLines > 1
                error(message('vision:ocr:invalidEvalResultsInput'));
            end

            bbox(i, :) = ocrTxt(i).TextLineBoundingBoxes;
            scores(i) = ocrTxt(i).TextLineConfidences;
            txt(i) = ocrTxt(i).TextLines;
        end
    end

    output = {im, bbox, scores, txt};
end

%--------------------------------------------------------------------------
function out = iForEach(fcn,data)
    out = fcn(data);
end

%--------------------------------------------------------------------------
function meanError = evaluateOCRString(resultTxt, groundTruthTxt, options)
    arguments
        resultTxt
        groundTruthTxt
        options.MetricUnit
    end

    numDetections = length(resultTxt);
    errorMetric = zeros(numDetections, 1);
    for i = 1:numDetections
        errorMetric(i) = ocrMetrics.computeErrorMetric(groundTruthTxt(i), resultTxt(i), options.MetricUnit);
    end

    if numDetections ~= 0
        meanError = mean(errorMetric);
    else
        meanError = 0;
    end
end

%--------------------------------------------------------------------------
function d = normLev(ocrStr, truthStr)
    
    ocrSize   = length(ocrStr);
    truthSize = length(truthStr);
    maxSize   = max(ocrSize, truthSize);
    d         = lev(ocrStr, truthStr)/maxSize;
end

%--------------------------------------------------------------------------
function d = lev(ocrStr, truthStr)
% Levenshtein distance between strings or char arrays.
% lev(ocrStr, truthStr) is the number of deletions, insertions,
% or substitutions required to transform ocrStr to truthStr.
% https://en.wikipedia.org/wiki/Levenshtein_distance

    ocrSize   = length(ocrStr);
    truthSize = length(truthStr);
    x = 0:truthSize;
    y = zeros(1,truthSize+1);   
    for ocrIdx = 1:ocrSize
        y(1) = ocrIdx;
        for truthIdx = 1:truthSize
            c = (ocrStr(ocrIdx) ~= truthStr(truthIdx)); % c = 0 if chars match, 1 if not.
            y(truthIdx+1) = min([y(truthIdx) + 1
                                 x(truthIdx+1) + 1
                                 x(truthIdx) + c]);
        end
        % swap
        [x,y] = deal(y,x);
    end
    d = x(truthSize+1);
end

%--------------------------------------------------------------------------
function updateMessage(printer, prevMessage, nextMessage)

    backspace = sprintf(repmat('\b',1,numel(prevMessage))); % figure how much to delete
    printer.print([backspace nextMessage]);
end

%--------------------------------------------------------------------------
function nextMessage = iPrintProgress(printer, prevMessage, k)

    nextMessage = getString(message('vision:semanticseg:verboseProgressTxt',k));
    updateMessage(printer, prevMessage, nextMessage);
end