classdef RecognitionColumnsWithValidationStatistics < vision.internal.ocr.ColumnStrategy
    % RecognitionColumnsWithValidationStatistics   OCR training column strategy with 
    %                                              validation statistics.
    
    %   Copyright 2022 The MathWorks, Inc.
    
    properties (Access = protected)
        % TableHeaderManager   An object of type vision.internal.ocr.ProgressDisplayTableHeader.
        TableHeaderManager
    end
    
    properties (SetAccess=protected)
        % Column (struct)   Struct array of table columns.
        Column
    end
    
    properties (SetAccess=protected, Dependent)
        % HorizontalBorder (char array)   Horizontal border of the table to
        %                                 print.
        HorizontalBorder
        
        % Headings (char array)   Table headings.
        Headings
    end
    
    methods
        %------------------------------------------------------------------
        function this = RecognitionColumnsWithValidationStatistics()
            % Formulate the column structure.
            this.Column(1).Name = "Epoch";
            this.Column(1).Type = "d";
            this.Column(1).Header = getString(message('vision:trainOCR:epoch'));
            this.Column(1).GroupID = 1;
            this.Column(2).Name = "Iteration";
            this.Column(2).Type = "d";
            this.Column(2).Header = getString(message('vision:trainOCR:iteration'));
            this.Column(2).GroupID = 1;
            this.Column(3).Name = "Time";
            this.Column(3).Type = "s";
            this.Column(3).Header = getString(message('vision:trainOCR:time'));
            this.Column(3).GroupID = 1;
            this.Column(4).Name = "TrainingRMSE";
            this.Column(4).Type = ".2f%";
            this.Column(4).Header = getString(message('vision:trainOCR:rmse'));
            this.Column(4).GroupID = 2;
            this.Column(5).Name = "TrainingCharacterError";
            this.Column(5).Type = ".2f";
            this.Column(5).Header = getString(message('vision:trainOCR:characterError'));
            this.Column(5).GroupID = 2;
            this.Column(6).Name = "TrainingWordError";
            this.Column(6).Type = ".2f";
            this.Column(6).Header = getString(message('vision:trainOCR:wordError'));
            this.Column(6).GroupID = 2;
            this.Column(7).Name = "ValidationRMSE";
            this.Column(7).Type = ".2f%";
            this.Column(7).Header = getString(message('vision:trainOCR:rmse'));
            this.Column(7).GroupID = 3;
            this.Column(8).Name = "ValidationCharacterError";
            this.Column(8).Type = ".2f";
            this.Column(8).Header = getString(message('vision:trainOCR:characterError'));
            this.Column(8).GroupID = 3;
            this.Column(9).Name = "ValidationWordError";
            this.Column(9).Type = ".2f";
            this.Column(9).Header = getString(message('vision:trainOCR:wordError'));
            this.Column(9).GroupID = 3;
            this.Column(10).Name = "BaseLearnRate";
            this.Column(10).Type = ".4f";
            this.Column(10).Header = getString(message('vision:trainOCR:learnRate'));
            this.Column(10).GroupID = 4;
            
            % Pad RMSE column header with additional spaces to increase column 
            % width. This makes space for larger RMSE values in the column.
            numPadSpaces = 4; % must be an even number to equally pad in left and right side.
            this.Column(4).Header = iPadSpacesBothSide(this.Column(4).Header, numPadSpaces);
            this.Column(7).Header = iPadSpacesBothSide(this.Column(7).Header, numPadSpaces);

            % Label column groups.
            columnGroupHeaders = {'', getString(message('vision:trainOCR:trainingStatistics')), ...
                            getString(message('vision:trainOCR:validationStatistics')), ''};

            % Get the table header with the specified column formulation.
            rawHeaders = {this.Column.Header};
            columnGroupIDs = [this.Column.GroupID];
            this.TableHeaderManager = vision.internal.ocr.ProgressDisplayTableHeader( ...
                rawHeaders, columnGroupIDs, columnGroupHeaders );
            
            % Format the columns of the table based on the column width of
            % their headers.
            columnWidths = this.TableHeaderManager.ColumnWidths;
            this.Column = this.formatColumns( this.Column, columnWidths );
        end
    end
    
    methods
        %------------------------------------------------------------------
        function horizontalBorder = get.HorizontalBorder(this)

            horizontalBorder = this.TableHeaderManager.HorizontalBorder;
        end
        
        %------------------------------------------------------------------
        function headings = get.Headings(this)
            
            headings = this.TableHeaderManager.Header;
        end
    end
end

%--------------------------------------------------------------------------
function out = iPadSpacesBothSide(in, numPadSpaces)

    currentLength = strlength(in);
    outputLength = currentLength + numPadSpaces;
    out = pad(in, outputLength, "both");
end