classdef ProgressDisplayTableHeader
    % ProgressDisplayTableHeader   Class to create progress display table header
    %
    % Note: This is a CVT equivalent of DLT's nnet.internal.cnn.util.ProgressDisplayTableHeader
    % with the following differences.
    %
    %   - This utility can support column group headers like used in trainOCR.
    %   - The entries of each row are center justified.  

    %   Copyright 2022 The MathWorks, Inc.
    
    properties (Constant)
        % PreferredMaxLineLength   Maximum preferred line length. This can
        % be bigger if the first word is bigger. Sentences that are bigger
        % than this quantity will be split in multiple lines.
        PreferredMaxLineLength = 15
        
        % ColumnPadding   How much padding to put around each header.
        ColumnPadding = 1
    end
    
    properties
        % Header   The header for the table (does not include horizontal
        % borders).
        Header
        
        % ColumnWidths   Width of each of the N columns, expressed as a 1xN
        % vector.
        ColumnWidths
        
        % HorizontalBorder   An horizontal border with the same length as
        % the header.
        HorizontalBorder
    end

    properties (Access=private)

        % RawHeaders  Cell array of column headers.
        RawHeaders

        % ColumnGroupIDs Array of column group ids corresponding to each
        % raw header.
        ColumnGroupIDs

        % ColumnGroupHeaders Cell array of column group headers
        % corresponding to each unique column group ID.
        ColumnGroupHeaders

        % HeaderLength Length of the constructed header. 
        HeaderLength

        % ColumnPaddingString Blank string of ColumnPadding length.
        ColumnPaddingString
    end
    
    methods
        %------------------------------------------------------------------
        function this = ProgressDisplayTableHeader(rawHeaders, columnGroupIDs, columnGroupHeaders)
            % ProgressDisplayTableHeader   Construct a
            % ProgressDisplayTableHeader from a cell array of headers
            % rawHeaders, group ids and group headers.
            
            % The number of unique column group ids must match the number of group headers.
            assert(length(unique(columnGroupIDs)) == length(columnGroupHeaders))

            % Initialize properties.
            this.Header = "";
            this.RawHeaders = rawHeaders;
            this.ColumnGroupIDs = columnGroupIDs;
            this.ColumnGroupHeaders = columnGroupHeaders;
            this.ColumnPaddingString = iCreateBlankString(this.ColumnPadding);

            % Construct the header for the provided column strategy.
            this = this.constructHeader();

            % Get the horizontal border for the formulated header length.
            this.HorizontalBorder = iCreateHorizontalBorder(this.HeaderLength);
        end
    end

    methods (Access = private)

        %------------------------------------------------------------------
        function this = constructHeader(this)
            % constructHeader   Transform a char array of raw headings saparated by | into
            % display-ready headings, well spaced and wrapped into multiple lines when
            % needed along with their group headers.
            
            % Wrap column headers on multiple lines.
            [this, wrappedHeaders] = this.wrapColumnHeaders();
            
            % Compute column group widths.
            [this, columnGroupWidths] = this.computeColumnGroupWidths(wrappedHeaders);

            % Wrap column group headers on multiple lines.
            [this, wrappedGroupHeaders] = this.wrapColumnGroupHeaders();

            % Construct the column headers.
            this = this.constructColumnHeaders(wrappedHeaders, wrappedGroupHeaders, columnGroupWidths);
            
            % Compute column widths.
            this.ColumnWidths = cellfun(@(s)strlength(s(1)),wrappedHeaders);
            this.ColumnWidths = this.ColumnWidths'+strlength(this.ColumnPaddingString)*2;
        end

        %------------------------------------------------------------------
        function [this, wrappedHeaders] = wrapColumnHeaders(this)
            % Wrap column headers on multiple lines.

            wrappedHeaders = cellfun(@(s)iWrapColumnHeader(s, this.PreferredMaxLineLength), ...
                this.RawHeaders, 'UniformOutput', false);
        end

        %------------------------------------------------------------------
        function [this, columnGroupWidths] = computeColumnGroupWidths(this, wrappedHeaders)
            % Compute length of each column header after wrapping them on multiple lines.
            wrappedHeadersLengths = cellfun(@(s)s(1).strlength, wrappedHeaders);

            % Compute the width of each column group by adding up the widths of the
            % column headers in its group.
            columnGroupWidths = zeros(size(this.ColumnGroupHeaders));
            for i = 1:length(this.ColumnGroupIDs)
                columnGroupID = this.ColumnGroupIDs(i);
                columnGroupWidths(columnGroupID) = columnGroupWidths(columnGroupID) + ...
                    wrappedHeadersLengths(i) + 2*this.ColumnPadding; % Account for 2 padding spaces
            end

            % Account for the pipelines between each column.
            groupFrequency = histcounts(this.ColumnGroupIDs);
            columnGroupWidths = columnGroupWidths + groupFrequency -1; 
        end

        %------------------------------------------------------------------
        function [this, wrappedGroupHeaders] = wrapColumnGroupHeaders(this)
            % Wrap column headers on multiple lines.

            groupFrequency = histcounts(this.ColumnGroupIDs);
            wrappedGroupHeaders = cell(size(this.ColumnGroupHeaders));
            for i = 1:length(this.ColumnGroupHeaders)
                minimumGroupHeaderWordLength = this.PreferredMaxLineLength*groupFrequency(i);
                wrappedGroupHeaders{i} = iWrapColumnHeader(this.ColumnGroupHeaders{i}, minimumGroupHeaderWordLength);
            end
        end

        %------------------------------------------------------------------
        function this = constructColumnHeaders(this, wrappedHeaders, wrappedGroupHeaders, columnGroupWidths)
            % Create the final header by merging the single headers with separators and
            % padding and by adding blank strings when needed.

            % Get the maximum number of rows for the header. If an header does not have
            % as many rows, fill them with blank strings
            headerRows = iCountHeaderRows(wrappedHeaders);
            groupHeaderRows = iCountHeaderRows(wrappedGroupHeaders);
            
            numRows = headerRows+groupHeaderRows;
            numHeaders = numel(wrappedHeaders);
            
            % Assumption: All columns in the same group are adjacent to each other.
            for currentRow = 1:numRows

                isGroupHeaderPrintedInCurrentRow = false(size(this.ColumnGroupIDs));
                for currentColumn = 1:numHeaders

                    columnGroupID = this.ColumnGroupIDs(currentColumn);
                    currentColumnGroupHeader = wrappedGroupHeaders{columnGroupID};
                    if strcmp(currentColumnGroupHeader, '')
                        % Print any column header that does not have any group header.

                        % Print.
                        columnHeader = wrappedHeaders{currentColumn};
                        this.Header = iPrintColumnHeader(this.Header, currentRow, ...
                            this.ColumnPaddingString, columnHeader, this.ColumnPaddingString);
                    else % Process the column header that has a group header.
                        
                        if currentRow <= groupHeaderRows 
                            % Process group header if the current row is within the
                            % first few rows alloted for group header.
            
                            if ~isGroupHeaderPrintedInCurrentRow(currentColumn)
                                % Print only if the group header was previously not
                                % printed in the current row.

                                % Print.
                                columnHeader = wrappedGroupHeaders{columnGroupID};
                                maxWidth = columnGroupWidths(columnGroupID);
                                this.Header = iPrintColumnHeaderCenterJustified(this.Header, currentRow, columnHeader, maxWidth);
                
                                % Update the status of group header printing in all columns.
                                isGroupHeaderPrintedInCurrentRow(this.ColumnGroupIDs==columnGroupID) = true;
                            else
                                % Do not print any thing if the group header has already
                                % been printed in the current row.
                            end
                        else 
                            % Print column headers if the current row is beyond the
                            % rows alloted for group header.

                            % Print.
                            columnHeader = wrappedHeaders{currentColumn};
                            adjustedRow = currentRow - groupHeaderRows;
                            this.Header = iPrintColumnHeader(this.Header, adjustedRow, ...
                                this.ColumnPaddingString, columnHeader, this.ColumnPaddingString);
                        end
                    end
                end
                
                if isempty(this.HeaderLength)
                    this.HeaderLength = strlength(this.Header) - 1;
                end
                this.Header = sprintf("%s|\n",this.Header);
            end
            
            % Strip out last \n
            this.Header = this.Header.strip;

            % Strip of any empty lines.
            this = this.stripEmptyLines;
        end

        %------------------------------------------------------------------
        function this = stripEmptyLines(this)

            headerlines = cellstr(strsplit(this.Header, newline));
            doStrip = cellfun(@(line) isequal(unique(line), ' |'), headerlines);
            if any(doStrip)
                nonEmptyHeaderlines = string(headerlines(~doStrip));
                this.Header = strjoin(nonEmptyHeaderlines, newline);
            end
        end
    end
end

%--------------------------------------------------------------------------
% Local helper functions.
%--------------------------------------------------------------------------
function header = iPrintColumnHeaderCenterJustified(header, row, columnHeader, maxWidth)
    
    % Center justify the text.
    currentColumnWidth = strlength(columnHeader(1));
    l1 = floor((maxWidth - currentColumnWidth)/2);
    l2 = ceil((maxWidth - currentColumnWidth)/2);
    leftPadding = iCreateBlankString(l1);
    rightPadding = iCreateBlankString(l2);

    header = iPrintColumnHeader(header, row, leftPadding, columnHeader, rightPadding);
end

%--------------------------------------------------------------------------
function header = iPrintColumnHeader(header, row, leftPadding, columnHeader, rightPadding)
    
    currentHeaderDepth = numel(columnHeader);
    if row > currentHeaderDepth
        currentColumnWidth = columnHeader(1).strlength;
        blankString = iCreateBlankString(currentColumnWidth);
        currentHeading = blankString;
    else
        currentHeading = columnHeader(row);
    end
    header = sprintf("%s|%s%s%s",header,leftPadding,currentHeading,rightPadding);
end

%--------------------------------------------------------------------------
function wrappedTextOutput = iWrapColumnHeader(inputText, minimumStringLength)
    % iWrapColumnHeader   Split text into multiple lines. Return an array of
    % strings (one string per line), where all strings are of the same length
    % (at least minimumStringLength). Text is centrally aligned.
    
    wrappedText = matlab.internal.display.printWrapped(inputText, minimumStringLength);
    wrappedText = string(wrappedText);
    wrappedTextSplit = split(wrappedText, newline);
    
    % Delete the last line, being only composed of a newline symbol.
    wrappedTextSplit(end) = [];
    
    % Centrally align text.
    wrappedTextOutput = wrappedTextSplit.pad('both');
end

%--------------------------------------------------------------------------
function blankString = iCreateBlankString(strLength)
    % iCreateBlankString   Create a blank string of length strLength.

    blankString = string(repmat(' ',1,strLength));
end

%--------------------------------------------------------------------------
function horizontalBorder = iCreateHorizontalBorder(headerLength)
    % iCreateHorizontalBorder   Create an horizontal border of length headerLength.

    horizontalBorder = "|" + string(repmat('=',1,headerLength)) + "|";
end

%--------------------------------------------------------------------------
function numRows = iCountHeaderRows(wrappedHeaders)

    numRows = max(cellfun(@numel,wrappedHeaders));
end