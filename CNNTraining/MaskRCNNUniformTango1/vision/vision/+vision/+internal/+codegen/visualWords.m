%   This visualWords class contains code generation implementation of
%   vision.internal.visualWords

%   This class is for internal use only.

%   Copyright 2022 The MathWorks, Inc.

%#codegen

classdef visualWords < vision.internal.visualWordsImpl & ...
        matlab.mixin.internal.indexing.ParenAssign & ...
        matlab.mixin.internal.indexing.Paren

    properties(Access = protected) % For array of visualWords codegen
        Data
    end

    properties(Constant, Hidden)
        ClassName = 'visualWords';
    end

    methods
        %------------------------------------------------------------------
        % Constructor
        %------------------------------------------------------------------
        function this = visualWords(varargin)
            this = this@vision.internal.visualWordsImpl(varargin{:});
            coder.varsize('data');
            data = {vision.internal.visualWordsImpl(varargin{:})};
            this.Data = data;
        end
    end

    % ---------------------------------------------------------------------
    methods(Hidden, Static, Access=private)
        function params = parseSpatialMatchInput(words1, words2, varargin)

            vision.internal.codegen.visualWords.checkWords(words1,'words1');
            vision.internal.codegen.visualWords.checkWords(words2,'words2');

            coder.internal.errorIf(words1.VocabularySize ~= words2.VocabularySize, ...
                'vision:visualWords:vocabSizeNotEqual');

            defaults = struct( ...
                'NumNeighbors', 10, ...
                'MatchThreshold', 1);
            % Define parser mapping struct
            pvPairs = struct( ...
                'NumNeighbors', uint32(0), ...
                'MatchThreshold', uint32(0));
            % Specify parser options
            poptions = struct( ...
                'CaseSensitivity', false, ...
                'StructExpand',    true, ...
                'PartialMatching', true);
            % Parse PV pairs
            pstruct = coder.internal.parseParameterInputs(pvPairs, poptions, varargin{:});

            numNeighbors = coder.internal.getParameterValue(pstruct.NumNeighbors, ...
                defaults.NumNeighbors, varargin{:});
            matchThreshold = coder.internal.getParameterValue(pstruct.MatchThreshold, ...
                defaults.MatchThreshold, varargin{:});

            vision.internal.visualWordsImpl.checkScalar(numNeighbors,'NumNeighbors');
            vision.internal.visualWordsImpl.checkScalar(matchThreshold,'MatchThreshold');

            params.NumNeighbors = double(numNeighbors);
            params.MatchThreshold = double(matchThreshold);
            coder.internal.errorIf((params.MatchThreshold > params.NumNeighbors), ...
                'vision:visualWords:thresholdGTNumNeighbors');

        end

        % -----------------------------------------------------------------
        function checkWords(words,name)
            validateattributes(words, {'vision.internal.codegen.visualWords'},...
                {'scalar'}, mfilename, name);
        end
    end

    methods(Hidden)
        % -----------------------------------------------------------------
        function this = addROIOffset(this,roi)
            % Add offset to location to compensate for ROI.
            this.Location = vision.internal.detector.addOffsetForROI(this.Location, roi, true);
        end

        %------------------------------------------------------------------
        function [indexPairs, score] = spatialMatch(words1, words2, varargin)

            params = vision.internal.codegen.visualWords.parseSpatialMatchInput(words1, words2, varargin{:});

            hist1 = vision.internal.visualWordsImpl.createSparseHistogram(words1);
            hist2 = vision.internal.visualWordsImpl.createSparseHistogram(words2);

            matchingWords = uint32(find(hist1 & hist2));

            indexPairs = vision.internal.codegen.spatialMatchVisualWords(words1.WordIndex,...
                words2.WordIndex, words1.Location, words2.Location,...
                matchingWords, params.NumNeighbors, params.MatchThreshold);

            % The score is the number of spatially matched words over total
            % number of words. A perfect score is 1.
            numMatched = size(indexPairs,1);

            score = numMatched ./ (words1.Count + eps);

        end
    end
    methods(Hidden, Static)
        %------------------------------------------------------------------
        % Creating an empty visualWords object
        %------------------------------------------------------------------
        function e = makeEmpty()
            %makeEmpty Make an empty object

            % The empty() method cannot be overridden in code generation.
            % To overcome this, we use the make-empty method as an interface
            % to create empty objects.
            wordIndex  = zeros(coder.ignoreConst(0), coder.ignoreConst(1), 'uint32');
            location = zeros(coder.ignoreConst(0), coder.ignoreConst(2), 'single');
            vocabSize = double(1);

            % Create a dummy object
            vw = vision.internal.visualWords(wordIndex, location, vocabSize);

            % Use repmat to make an empty object
            e = repmat(vw, 0, 0);
        end
    end

    methods(Hidden, Static, Access=private)
        % -----------------------------------------------------------------
        function checkInputs(words, locations, sz)

            validateattributes(words, {'numeric'},...
                {'vector','nonsparse','real'},...
                mfilename, 'WordIndex', 1);

            validateattributes(locations, {'single'},...
                {'size', [NaN 2], 'nonsparse', 'real'},...
                mfilename, 'Location', 2);

            validateattributes(sz, {'numeric'}, ...
                {'scalar', 'positive', 'integer', 'real'}, ...
                mfilename, 'VocabularySize', 3);

            coder.internal.errorIf(numel(words) ~= size(locations,1), ...
                'vision:visualWords:invalidNumelWords');
        end

    end

    methods
        %------------------------------------------------------------------
        % Paren reference function for array of visualWords objects
        %------------------------------------------------------------------
        function this1 = parenReference(this, idx, varargin)

            coder.internal.errorIf(numel(varargin)>0, ...
                "vision:visualWords:oneDimIndexing");
            this1 = vision.internal.codegen.visualWords.makeEmpty();

            if ischar(idx) && strcmpi(idx, ':')
                return;
            elseif ischar(idx)
                coder.internal.errorIf(true, "vision:visualWords:oneDimIndexing");
            end

            % Maintain sizing
            if isrow(this)
                dataArray = coder.nullcopy( cell(1, numel(idx)) );
            else
                dataArray = coder.nullcopy( cell(numel(idx), 1) );
            end

            for n = 1 : numel(idx)
                dataArray{n} = this.Data{idx(n)};
            end

            this1.Data   = dataArray;
            this1.WordIndex                = dataArray{1}.WordIndex;
            this1.Location               = dataArray{1}.Location;
            this1.VocabularySize        = dataArray{1}.VocabularySize;
        end
        %------------------------------------------------------------------
        % Paren assign function for array of visualWords objects
        %------------------------------------------------------------------
        function this = parenAssign(this, rhs, idx, varargin)

            coder.internal.errorIf(numel(varargin)>0, ...
                "vision:visualWords:oneDimIndexing");
            this.checkTypes(rhs);
            if ischar(idx) && strcmp(idx, ':')
                idx = 1 : numel(this);
            elseif ischar(idx)
                coder.internal.errorIf(true, "vision:visualWords:oneDimIndexing");
            end

            farthestElement = max(idx);

            if farthestElement > numel(this)
                % Copy over current elements
                if isrow(this)
                    dataArray = coder.nullcopy( cell(1, farthestElement) );
                else
                    dataArray = coder.nullcopy( cell(farthestElement, 1) );
                end
                for n = 1 : numel(this)
                    dataArray{n} = this.Data{n};
                end

                % Replace/add new elements
                for n = 1 : numel(idx)
                    dataArray{idx(n)} = rhs.Data{n};
                end
                this.Data = dataArray;
            else
                % No need to grow cell array, just replace data
                for n = 1 : numel(idx)
                    this.Data{idx(n)} = rhs.Data{n};
                end
            end
        end

        %------------------------------------------------------------------
        % Overloading the vertcat functionality
        %------------------------------------------------------------------
        function this = vertcat(this, varargin)

            coder.internal.assert(...
                isa(this, class(this)), ...
                "vision:visualWords:invalidClass");
            coder.internal.errorIf(...
                (isrow(this) && numel(this)~=1), 'MATLAB:catenate:matrixDimensionMismatch');

            num = numel(varargin);
            for n = 1 : num
                this.checkTypes(varargin{n});
            end

            data = initializeArrayData(this);
            dataArray  = repmat({data}, coder.ignoreConst(0), 1);

            this = copyData(this, dataArray, num, varargin{:});
        end

        %------------------------------------------------------------------
        % Calculate the number of visualWords in visualWords array
        %------------------------------------------------------------------
        function n = numel(this)
            n = numel(this.Data);
        end

        %------------------------------------------------------------------
        % Overloading the isscalar functionality
        %------------------------------------------------------------------
        function n = isscalar(this)
            n = numel(this.Data)==1;
        end

        %------------------------------------------------------------------
        % Overloading the repmat functionality
        %------------------------------------------------------------------
        function this = repmat(this, varargin)
            coder.internal.assert( numel(varargin)<3, ...
                "vision:visualWords:oneDimIndexing");

            % Validate repmat(obj,[a, b, ...]) syntax
            if numel(varargin)==1 && ~isscalar(varargin{1})
                in = varargin{1};

                % Only indexing up to two dimensions
                coder.internal.assert( numel(in)<=2 && ...
                    (in(1)<=1 || in(2)<=1), ...
                    "vision:visualWords:oneDimIndexing");
            end

            if numel(varargin)==2 && isscalar(varargin{1}) && isscalar(varargin{2})
                coder.internal.assert( ((varargin{1}<=1) && (varargin{2}<=1)), ...
                    "vision:visualWords:oneDimIndexing");
            end
            this.Data = repmat(this.Data, varargin{:});
        end

        %------------------------------------------------------------------
        % Overloading the horzcat functionality
        %------------------------------------------------------------------
        function this = horzcat(this, varargin)

            coder.internal.assert(...
                isrow(this), 'MATLAB:catenate:matrixDimensionMismatch');
            num = numel(varargin);
            for n = 1 : num
                this.checkTypes(varargin{n});
            end

            data = initializeArrayData(this);
            dataArray  = repmat({data}, 1, coder.ignoreConst(0));

            this = copyData(this, dataArray, num, varargin{:});
        end

        %------------------------------------------------------------------
        % Overloading the transpose functionality
        %------------------------------------------------------------------
        function this = transpose(this)

            % Transpose is not supported for cell arrays in code
            % generation. Use reshape instead for transpose because these
            % are 1-D arrays.
            if isrow(this)
                this.Data = reshape(this.Data, numel(this), 1);
            else
                this.Data = reshape(this.Data, 1, numel(this));
            end
        end

        %------------------------------------------------------------------
        % Overloading the ctranspose functionality
        %------------------------------------------------------------------
        function this = ctranspose(this)

            this = transpose(this);
        end

        %------------------------------------------------------------------
        % Overloading the reshape functionality
        %------------------------------------------------------------------
        function this = reshape(this, varargin)

            coder.internal.assert( numel(varargin)<3, ...
                "vision:visualWords:oneDimIndexing");

            if numel(varargin)==2
                coder.internal.assert(...
                    ~(isempty(varargin{1}) || isempty(varargin{2})), ...
                    'driving:oneDimArrayBehavior:reshapeWithEmpties');
            end

            this.Data = reshape(this.Data, varargin{:});
        end

        %------------------------------------------------------------------
        % Overloading the isempty functionality
        %------------------------------------------------------------------
        function ie = isempty(this)

            ie = numel(this)== 0;
        end

        %------------------------------------------------------------------
        function n = end(this,varargin)
            % Only 1-D indexing is supported, so end is always numel.
            n = numel(this);
        end

        %------------------------------------------------------------------
        function l = length(this)
            % For a 1-D array, length is numel
            l = numel(this);
        end

        %------------------------------------------------------------------
        function y = isrow(this)
            y = isrow(this.Data);
        end
        %------------------------------------------------------------------
        function this = copyData(this, dataArray, num, varargin)

            % copy over current elements
            for n=1:numel(this)
                dataArray{end+1} = this.Data{n};
            end

            % copy over new elements
            for n=1:num
                for nn = 1:numel(varargin{n})
                    dataArray{end+1} = varargin{n}.Data{nn};
                end
            end

            % Assign dataArray to corresponding data property
            this.Data = dataArray;
        end

        %------------------------------------------------------------------
        function checkTypes(this, tObj)
            coder.internal.assert(isa(tObj, class(this)), ...
                "vision:visualWords:invalidClass");
            coder.internal.errorIf(~isa(this.WordIndex, class(tObj.WordIndex)),...
                "vision:visualWords:differentTypes");
            coder.internal.errorIf(~isa(this.Location, class(tObj.Location)),...
                "vision:visualWords:differentTypes");
            coder.internal.errorIf(~isa(this.VocabularySize, class(tObj.VocabularySize)),...
                "vision:visualWords:differentTypes");
        end

        %------------------------------------------------------------------
        % initialize the array objects for visualWords object
        %------------------------------------------------------------------
        function data = initializeArrayData(this)
            % initialize visual words array data
            data = vision.internal.visualWordsImpl(this.WordIndex, ...
                this.Location, this.VocabularySize);
        end
    end
end
