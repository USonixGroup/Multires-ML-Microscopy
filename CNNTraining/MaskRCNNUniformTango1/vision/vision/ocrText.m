classdef ocrText

% Copyright 2013-2024 The MathWorks, Inc.

%#codegen
%#ok<*EMCA>

    properties(GetAccess = public,SetAccess = private)

        Text        
        
        CharacterBoundingBoxes
        
        CharacterConfidences
        
        Words
        
        WordBoundingBoxes
        
        WordConfidences

        TextLines
        
        TextLineBoundingBoxes
        
        TextLineConfidences
    end    
    
    properties(Hidden, Access = private)
        pTextInfo

        LayoutAnalysis
    end

    properties(Access = protected)
        
        Version = 2.0;
    end
    
    methods
        function words = get.Words(obj)
            % Words access is not supported in codegen.
            coder.internal.assert(isempty(coder.target), ...
            'vision:ocr:codegenInvalidWordsAccess');
            
            words = obj.Words;            
        end
    end
    
    methods (Access = private)
        
        % -----------------------------------------------------------------
        function this = ocrText(varargin)
            narginchk(0,1)

            if nargin == 1
                
                that = varargin{1};

                this.Text = that.Text;
                this.pTextInfo = that.pTextInfo;
                this.LayoutAnalysis = that.LayoutAnalysis;
                this.Version = that.Version;

                this.CharacterBoundingBoxes = that.CharacterBoundingBoxes;
                this.CharacterConfidences = that.CharacterConfidences;
                this.Words = that.Words;
                this.WordBoundingBoxes = that.WordBoundingBoxes;
                this.WordConfidences = that.WordConfidences;
                this.TextLines = that.TextLines;
                this.TextLineBoundingBoxes = that.TextLineBoundingBoxes;
                this.TextLineConfidences = that.TextLineConfidences;
            else
                
                defaultBBox = double(zeros(0,4));
                defaultIndexVector = int32(zeros(0,1));
                defaultCharacterVector = '';
                defaultConfidenceVector = single(zeros(0,1));
                defaultCharacterIndexVector= int32(zeros(0,2));
                coder.varsize('defaultBBox',[Inf 4], [1 0] );
                coder.varsize('defaultIndexVector',[Inf 1], [1 0] );
                coder.varsize('defaultCharacterVector',[1 Inf], [0 1] );
                coder.varsize('defaultConfidenceVector',[Inf 1], [1 0] );
                coder.varsize('defaultCharacterIndexVector',[Inf 2], [1 0] );
                
                defaultPTextInfo = struct('CharacterBBox', defaultBBox,'CharacterWordIndex', defaultIndexVector, ...
                    'CharacterConfidence', defaultConfidenceVector, ...
                    'WordBBox', defaultBBox, 'WordTextLineIndex', defaultIndexVector, 'WordConfidence', defaultConfidenceVector,...
                    'WordCharacterIndex', defaultCharacterIndexVector, ...
                    'TextLineBBox', defaultBBox, 'TextLineParagraphIndex', defaultIndexVector, ...
                    'TextLineConfidence', defaultConfidenceVector, 'TextLineCharacterIndex', defaultCharacterIndexVector,...
                    'ParagraphBBox', defaultBBox, 'ParagraphBlockIndex', defaultIndexVector, ...
                    'ParagraphConfidence', defaultConfidenceVector, 'ParagraphCharacterIndex', defaultCharacterIndexVector,...
                    'BlockBBox', defaultBBox, 'BlockPageIndex', defaultIndexVector, ...
                    'BlockConfidence', defaultConfidenceVector, 'BlockCharacterIndex', defaultCharacterIndexVector,...
                    'Characters', defaultCharacterVector);
                
                this.pTextInfo = defaultPTextInfo;
                this.LayoutAnalysis = defaultCharacterVector;

                this.Text = defaultCharacterVector;
                this.CharacterBoundingBoxes = defaultBBox;
                this.CharacterConfidences = defaultConfidenceVector;
                this.Words = defaultCharacterVector;
                this.WordBoundingBoxes = defaultBBox;
                this.WordConfidences = defaultConfidenceVector;
                this.TextLines = defaultCharacterVector;
                this.TextLineBoundingBoxes = defaultBBox;
                this.TextLineConfidences = defaultConfidenceVector;
            end
        end

        % -----------------------------------------------------------------
        function this = populateProperties(this, txt, metadata, layoutAnalysis)
                
            this.pTextInfo = metadata;
            this.Text      = txt;

            if numel(this.Text) ~= size(this.pTextInfo.CharacterBBox,1)
                % Text and metadata are inconsistent. Use the text
                % associated with the metadata instead.
                this.Text = this.pTextInfo.Characters;
            end

            if strcmpi(layoutAnalysis, 'Character') && ~isempty(this.Text)
                % Remove trailing newlines inserted by tesseract. This
                % produces single character results for the character
                % text layout mode.
                newlines = this.Text == newline;
                this.Text(newlines) = [];
                
                if ~isempty(this.pTextInfo.Characters)
                    % This field is populated only in the latest engine. In
                    % legacy engine, this field is empty.
                    this.pTextInfo.Characters(:,newlines(:)) = [];
                end

                this.pTextInfo.CharacterBBox(newlines(:),:) = [];
                this.pTextInfo.CharacterConfidence(newlines(:)) = [];
                this.pTextInfo.CharacterWordIndex(newlines(:)) = [];

            else
                % Get bounding boxes for new line characters.
                newlines      = strfind(this.Text,newline);
                newlineBBoxes = getNewlineBBoxes(this, newlines);
                this.pTextInfo.CharacterBBox(newlines(:),:) = newlineBBoxes;
            end
            
            % Get metadata for text lines.
            [this.TextLineConfidences, this.TextLineBoundingBoxes, this.TextLines] ...
                = getTextLineMetadata(this);

            % Get metadata for words.
            [this.WordConfidences, this.WordBoundingBoxes, this.Words] ...
                = getWordMetadata(this);
            
            % Get bounding boxes for spaces.
            spaces      = findSpaces(this);
            spaceBBoxes = getSpaceBBoxes(this, spaces);
            this.pTextInfo.CharacterBBox(spaces(:),:) = spaceBBoxes;
            
            % Fill metadata for Characters
            this.CharacterConfidences = this.pTextInfo.CharacterConfidence * single(0.01);
            
            % Confidence values less than zero indicate new line or
            % space characters. The negative confidence values are
            % converted to NaN in order to indicate they are not valid
            % confidences.
            this.CharacterConfidences(this.CharacterConfidences < 0) = NaN;
            this.CharacterBoundingBoxes = this.pTextInfo.CharacterBBox;
    
            % Store layoutAnalyis and engine inputs to use in loadobj method.
            this.LayoutAnalysis = layoutAnalysis;
        end
    end

    methods
        
        % -----------------------------------------------------------------
        function bboxes = locateText(this, pattern, varargin)
            %
              
            pattern = convertStringsToChars(pattern);
            params = checkInputs(pattern, varargin{:});

            if isscalar(this)
                bboxes = locateTextScalar(this, this.Text, pattern, ...
                    params.IgnoreCase, params.UseRegexp);
            else
                validateattributes(this,{class(this)},{'vector'});
                numROI = numel(this);
                bboxes = cell(numROI,1);
                for n = 1:numROI
                    bboxes{n} = locateTextScalar(this(n), this(n).Text, ...
                        pattern, params.IgnoreCase, params.UseRegexp);
                end
            end
            
        end               
    end
    
    methods(Access = private)
        % -----------------------------------------------------------------
        % Returns bounding boxes around Text from startIndex to endIndex
        % -----------------------------------------------------------------
        function bbox = ind2bbox(this, startIndex, endIndex)                               
            
            startIndex = int32(startIndex);
            endIndex   = int32(endIndex);
            
            % Find the text line where Text(startIndex) is located
            widx   = this.pTextInfo.CharacterWordIndex(startIndex);
            tlidx  = this.pTextInfo.WordTextLineIndex(widx);
                                      
            % Define the end of all the text lines            
            endOfTextLines = this.pTextInfo.TextLineCharacterIndex(tlidx:end,:);
            
            % adjust for new lines at the end of the lines
            endOfTextLines = endOfTextLines(:,2) - 1;
            
            % clip last text line, which is a empty new line
            endOfTextLines(end) = length(this.Text);
            
            % Determine the number of line wraps that are present 
            numLineWraps = int32(sum(endIndex > endOfTextLines));
            tl = tlidx:tlidx+numLineWraps;
                        
            % Check if number of line wraps exceed number of textlines
            numTextLines = size(this.pTextInfo.TextLineBBox,1);
            if tlidx+numLineWraps > numTextLines
                numLineWraps = numTextLines - tlidx;
                tl = tlidx:tlidx+numLineWraps;
            end
            
            if numLineWraps
                % The set of indices, startIndex:endIndex, cover multiple
                % lines of text. We must partition this set and create
                % sub-sets that span only 1 line at a time so that we can
                % create bounding boxes for each line of text.
                
                % create array to hold sets of indices
                i1 = zeros(numLineWraps+1,1);
                i2 = zeros(numLineWraps+1,1);
                
                % store the starting and ending indices for each text line.
                tlIndexes = this.pTextInfo.TextLineCharacterIndex(tl,:);
                
                % partition startIndex:endIndex
                i1(1)       = startIndex;                                              
                i1(2:end)   = tlIndexes(2:end,1)';
                
                i2(1:end-1) = tlIndexes(1:end-1,2)'-2; % -2 for new line and one-past-the-last index value
                i2(end)     = endIndex;
                
                % Create a bounding box for each text line.
                n = numel(tl);
                bbox = zeros(n,4);
                for j = 1:n                                        
                    % get the bounding boxes of all the characters in the
                    % line of text between i1(j):i2(j)
                    charBBoxes = this.pTextInfo.CharacterBBox(i1(j):i2(j),:);
                    
                    if isempty(charBBoxes), continue, end;
                                                                              
                    bbox(j,:) = bboxUnion(charBBoxes);

                end
            else                   
                charBBoxes = this.pTextInfo.CharacterBBox(startIndex:endIndex,:);
                                                
                bbox = bboxUnion(charBBoxes);                
            end
            
        end
        
        % -----------------------------------------------------------------
        % Scalar version of locateText. Calls either strfind or regexp,
        % based on user selection.
        % -----------------------------------------------------------------
        function bboxes = locateTextScalar(this, txt, expr, ...
                ignoreCase, useRegexp)
            if logical(useRegexp)
                bboxes = regexpScalar(this,txt,expr,ignoreCase);
            else
                bboxes = strfindScalar(this,txt,expr,ignoreCase);
            end
        end
        
        % -----------------------------------------------------------------
        % Scalar version of strfind. Invokes strfind to locate text.
        % -----------------------------------------------------------------
        function bboxes = strfindScalar(this,txt, str, ignoreCase)
            if logical(ignoreCase)
                str = lower(str);
                txt = lower(txt);
            end
            if iscell(str)
                startIndex = cell(1,numel(str));
                endIndex   = cell(1,numel(str));
                for i = 1:numel(str)
                    startIndex{i} = strfind(txt,str{i});
                    endIndex{i}   = startIndex{i} + numel(str{i}) - 1;
                end
                startIndex = cell2mat(startIndex);
                endIndex   = cell2mat(endIndex);
            else
                startIndex = strfind(txt,str);
                endIndex   = startIndex + numel(str) - 1;
            end
            
            bboxes = populateBBox(this,startIndex, endIndex);
            
        end
        
        % -----------------------------------------------------------------
        % Populates bounding boxes that are generated for text between
        % startIndex and endIndex. 
        % -----------------------------------------------------------------
        function bboxes = populateBBox(this,startIndex, endIndex)
            bboxes = zeros(0,4);
            for i = 1:numel(startIndex)
                bboxes = [bboxes; ind2bbox(this, startIndex(i),endIndex(i))]; %#ok<AGROW>
            end
        end
        
        % -----------------------------------------------------------------
        % Scalar version of regexp for ocrText. Invokes regexp to locate
        % text.
        % -----------------------------------------------------------------
        function bboxes = regexpScalar(this, txt, expr, ignoreCase)
            if isempty(coder.target)
                if ignoreCase
                    [startIndex, endIndex] = regexpi(txt, expr, 'start','end');
                else
                    [startIndex, endIndex] = regexp(txt, expr, 'start','end');
                end
                
                if iscell(expr)
                    startIndex = cell2mat(startIndex);
                    endIndex   = cell2mat(endIndex);
                end
                bboxes = populateBBox(this, startIndex, endIndex);
            else
                bboxes = zeros(0,4);
            end
        end
        
        % -----------------------------------------------------------------
        % Get bounding boxes for new lines.
        % -----------------------------------------------------------------
        function bboxes = getNewlineBBoxes(this, newlines)
            % Get bboxes for newline characters. Newline characters are
            % "virtual" characters. They don't have a real bounding boxes,
            % but we need to make sure they have something that makes sense
            % for computations later on. Here a newline is given a bounding
            % box located at the end of the text line with a width and
            % height of zero.
            if isempty(newlines)
                bboxes = zeros(0, 4);
            else
                wordIdx = this.pTextInfo.CharacterWordIndex(newlines);
                tind = this.pTextInfo.WordTextLineIndex(wordIdx);
                bboxes = zeros(numel(tind), 4);
                for i = 1:numel(tind)
                    
                    textLineBBox = this.pTextInfo.TextLineBBox(tind(i),:);
                    % xy is at end of textline, width and height are zero
                    x = textLineBBox(1) + textLineBBox(3);
                    y = textLineBBox(2);
                    w = 0;
                    h = 0;
                    
                    bboxes(i,:) = [x y w h];
                    
                end
            end
        end
        
        % -----------------------------------------------------------------
        % Get bounding boxes for spaces.
        % -----------------------------------------------------------------
        function bboxes = getSpaceBBoxes(this,spaceIdx)
            % Spaces are not assigned bounding boxes by Tesseract.
            % Therefore, a bounding box for spaces is manually created to
            % ensure consistent processing later on. The bounding box for
            % a space between two words, w1 and w2, spans the distance
            % between w1 and w2. The x y location of the bounding
            % box for a space starts at the top right corner of the
            % bounding box for w1. The width equals the distance between w1
            % and w2, and the height is the set equal to the height of w1.
                                       
            wordIdx  = this.pTextInfo.CharacterWordIndex(spaceIdx);
            bboxes   = zeros(numel(wordIdx), 4);
            
            for i = 1:numel(wordIdx)
                wordBBox = this.pTextInfo.WordBBox(wordIdx(i),:);
                x = wordBBox(1) + wordBBox(3);
                y = wordBBox(2);
                
                w = max(this.pTextInfo.WordBBox(wordIdx(i)+1,1) - x, 1);
                h = wordBBox(4);
                
                bboxes(i,:) = [x y w h];
            end
        end

        % -----------------------------------------------------------------
        % Get text line related metadata.
        % -----------------------------------------------------------------
        function [conf, bbox, textLines] = getTextLineMetadata(this)
        % getTextLineMetadata and getWordMetadata works on the following mental model:
        % A typical OCR result: txt = 'CVT~~' where ~ is a newline. All OCR metadata 
        % results contain two newlines in the end. For such results, the 
        % "<*>CharacterIndex" properties of metadata will be: [startIndex endIndex]
        %   WordCharacterIndex      - [1 4] -> endIndex = numCharacters + 1 newline
        %   TextLineCharacterIndex  - [1 5] -> endIndex = numCharacters + 2 newlines
        %   ParagraphCharacterIndex - [1 6] -> endIndex = numCharacters + 2 newlines + 1
        %   BlockCharacterIndex     - [1 6] -> endIndex = numCharacters + 2 newlines + 1

            indices = this.pTextInfo.TextLineCharacterIndex;
            numTextLines = size(indices,1);
            
            conf = this.pTextInfo.TextLineConfidence .* single(0.01);
            bbox = this.pTextInfo.TextLineBBox;
            if isempty(coder.target)
                textLines = cell(numTextLines,1);
                for i = 1:size(indices,1)
                    textLines{i} = this.Text(indices(i,1):indices(i,2)-2);
                end
            else
                textLines = '';
            end
        end

        % -----------------------------------------------------------------
        % Get word related metadata.
        % -----------------------------------------------------------------
        function [conf, bbox, words] = getWordMetadata(this)
            indices = this.pTextInfo.WordCharacterIndex;
            numWords = size(indices,1);
            
            conf = this.pTextInfo.WordConfidence .* single(0.01);
            bbox = this.pTextInfo.WordBBox;
            if isempty(coder.target)
                words = cell(numWords,1);
                for i = 1:size(indices,1)
                    words{i} = this.Text(indices(i,1):indices(i,2)-1);
                end
            else
                words = '';
            end
        end
        
        % -----------------------------------------------------------------
        % Return the indices to all the space characters between words.
        % -----------------------------------------------------------------
        function idx = findSpaces(this)                                                     
          
            % find all the spaces in Text
            allSpaces = this.Text' == char(32);                       
            
            % The spaces in Text might occur in the middle of actual words
            % when a small CharacterSet is used. These spaces must be
            % removed because they are not real spaces between words.
            %
            % Spaces in the middle of the word are removed by finding the
            % intersection between all the spaces and the real spaces
            % between words, which is encoded by CharacterConfidence values
            % of -1. Newlines locations are also encoded using
            % CharacterConfidences of -1, so the real space locations
            % cannot be determined using only the CharacterConfidences.
            
            isRealSpaceOrNewLine = this.pTextInfo.CharacterConfidence < 0;
                        
            idx = find(isRealSpaceOrNewLine(:) & allSpaces(:));
        end
    end

    methods(Hidden, Static)
        % -----------------------------------------------------------------
        % Create an ocrText object given text and metadata information.
        % -----------------------------------------------------------------
        function textInfo = create(txt,metadata,layoutAnalysis)
            
            if iscell(txt)
                n = size(txt,1);
                if n > 0
                    textInfo(n,1) = createImpl(txt{end},metadata(end),layoutAnalysis); %#ok<*EMVDF>
                    for i = 1:n-1
                        textInfo(i) = createImpl(txt{i},metadata(i),layoutAnalysis);
                    end
                else
                    % create empty object array
                    textInfo = ocrText.empty(0,1);
                end
            else
                textInfo = createImpl(txt,metadata,layoutAnalysis);
            end
            
            % -------------------------------------------------------------
            % Helper to create one ocrText object and populate its
            % properties.
            % -------------------------------------------------------------
            function this = createImpl(txt,metadata,layoutAnalysis)
                this = ocrText;
                this = this.populateProperties(txt,metadata,layoutAnalysis);
            end
        end
    end

    methods(Static, Access=private)
        %------------------------------------------------------------------
        % load object
        %------------------------------------------------------------------
        function this = loadobj(in)
            
            if isstruct(in) % Object was saved after R2021b.
               that = in; 
            else % Object was saved in or before R2021b.
                
                that.Text = in.Text;
                that.pTextInfo = in.pTextInfo;
                that.LayoutAnalysis = in.LayoutAnalysis;
                
                that.CharacterBoundingBoxes = in.CharacterBoundingBoxes;
                that.CharacterConfidences = in.CharacterConfidences;
                that.Words = in.Words;
                that.WordBoundingBoxes = in.WordBoundingBoxes;
                that.WordConfidences = in.WordConfidences;
                that.TextLines = in.TextLines;
                that.TextLineBoundingBoxes = in.TextLineBoundingBoxes;
                that.TextLineConfidences = in.TextLineConfidences;
                
                that.Version = 1.0;
            end

            this = ocrText(that);
        end
    end
    
    methods(Access=private)
        %------------------------------------------------------------------
        % save object
        %------------------------------------------------------------------
        function that = saveobj(this)
            
            % save properties into struct
            that.Text = this.Text;
            that.pTextInfo = this.pTextInfo;
            that.LayoutAnalysis = this.LayoutAnalysis;
            that.Version = this.Version;            
            that.CharacterBoundingBoxes = this.CharacterBoundingBoxes;
            that.CharacterConfidences = this.CharacterConfidences;
            that.Words = this.Words;
            that.WordBoundingBoxes = this.WordBoundingBoxes;
            that.WordConfidences = this.WordConfidences;
            that.TextLines = this.TextLines;
            that.TextLineBoundingBoxes = this.TextLineBoundingBoxes;
            that.TextLineConfidences = this.TextLineConfidences;
        end
    end
end

% -------------------------------------------------------------------------
function params = checkInputs(pattern, varargin)

if isempty(coder.target)
    
    parser = inputParser();
    
    parser.addParameter('UseRegexp',  false);
    parser.addParameter('IgnoreCase', false);
    
    parse(parser, varargin{:});
    params = parser.Results;
    
else
    params = checkInputsCodegen(varargin{:});
end

if iscell(pattern)
    validateattributes(pattern,{'cell'},{'vector','row','nonempty'},'locateText');
    if ~iscellstr(pattern)
        error(message('vision:ocr:notAllStrings'));
    end
else    
    validateattributes(pattern, {'char'},{'vector','row','nonempty'},'locateText');
end

checkLogical(params.UseRegexp,  'UseRegexp');
checkLogical(params.IgnoreCase, 'IgnoreCase');

end

% -------------------------------------------------------------------------
function results = checkInputsCodegen(varargin)

pvPairs = struct( ...
    'UseRegexp',  uint32(0), ...
    'IgnoreCase', uint32(0));

popt = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand'   , true, ...
    'PartialMatching', true);

optarg = eml_parse_parameter_inputs(pvPairs, popt, varargin{:});

results.UseRegexp  = eml_get_parameter_value(optarg.UseRegexp, ...
    false, varargin{:});

results.IgnoreCase = eml_get_parameter_value(optarg.IgnoreCase, ...
    false, varargin{:});

% UseRegexp is not supported in codegen.
coder.internal.errorIf(logical(results.UseRegexp),...
    'vision:ocr:codegenRegexpUnsupported');

end

% -------------------------------------------------------------------------
function checkLogical(tf,name)
vision.internal.errorIfNotFixedSize(tf, name);
validateattributes(tf, {'logical','numeric'},...
    {'nonnan', 'scalar', 'real','nonsparse'},...
    'locateText',name);
end

% -------------------------------------------------------------------------
% Merges multiple bboxes into one encompassing bbox
% -------------------------------------------------------------------------
function bbox = bboxUnion(bboxes)
x1 = bboxes(:,1);
y1 = bboxes(:,2);
x2 = bboxes(:,1) + bboxes(:,3) - 1;
y2 = bboxes(:,2) + bboxes(:,4) - 1;

x = min(x1);
y = min(y1);

w = max(x2) - x + 1;
h = max(y2) - y + 1;

bbox = [x y w h];
end