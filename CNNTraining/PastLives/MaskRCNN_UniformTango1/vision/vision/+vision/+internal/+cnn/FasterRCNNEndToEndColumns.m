classdef FasterRCNNEndToEndColumns < nnet.internal.cnn.util.ColumnStrategy
    % FasterRCNNEndToEndColumns Columns for end-to-end training which shows
    % RPN accuracy and loss and Fast R-CNN accuracy and loss.
    
    %   Copyright 2018 The MathWorks, Inc.
    
    properties (Access = protected)
        % TableHeaderManager   An object of type nnet.internal.cnn.util.ProgressDisplayTableHeader
        TableHeaderManager
    end
    
    properties (SetAccess=protected)
        % Column (struct)   Struct array of table columns
        Column
    end
    
    properties (SetAccess=protected, Dependent)
        % HorizontalBorder (char array)   Horizontal border of the table to
        %                                 print
        HorizontalBorder
        
        % Headings (char array)   Table headings
        Headings
    end
    
    methods
        function this = FasterRCNNEndToEndColumns(isValidationDataSpecified)
            this.Column(1).Name = "Epoch";
            this.Column(1).Type = "d";
            this.Column(1).Header = getString(message('nnet_cnn:internal:cnn:RegressionColumns:Epoch'));
            this.Column(2).Name = "Iteration";
            this.Column(2).Type = "d";
            this.Column(2).Header = getString(message('nnet_cnn:internal:cnn:RegressionColumns:Iteration'));
            this.Column(3).Name = "Time";
            this.Column(3).Type = "s";
            this.Column(3).Header = getString(message('nnet_cnn:internal:cnn:RegressionColumns:Time'));
            this.Column(4).Name = "Loss";
            this.Column(4).Type = ".4f";
            this.Column(4).Header = getString(message('nnet_cnn:internal:cnn:RegressionColumns:Loss'));
            this.Column(5).Name = "Accuracy";
            this.Column(5).Type = ".2f%%";
            this.Column(5).Header = getString(message('nnet_cnn:internal:cnn:ClassificationColumns:Accuracy'));
            this.Column(6).Name = "RMSE";
            this.Column(6).Type = ".2f%";
            this.Column(6).Header = getString(message('nnet_cnn:internal:cnn:RegressionColumns:RMSE'));
            this.Column(7).Name = "RPNAccuracy";
            this.Column(7).Type = ".2f%%";
            this.Column(7).Header = getString(message('vision:rcnn:verboseColRPNAcc'));
            this.Column(8).Name = "RPNRMSE";
            this.Column(8).Type = ".2f%";
            this.Column(8).Header = getString(message('vision:rcnn:verboseColRPNRmse'));
            currentColIdx = 9;
            if isValidationDataSpecified
                this.Column(currentColIdx).Name = "ValidationLoss";
                this.Column(currentColIdx).Type = ".4f";
                this.Column(currentColIdx).Header = getString(message('nnet_cnn:internal:cnn:RegressionValidationColumns:ValidationLoss'));
                currentColIdx = currentColIdx + 1;

                this.Column(currentColIdx).Name = "ValidationAccuracy";
                this.Column(currentColIdx).Type = ".2f%%";
                this.Column(currentColIdx).Header = getString(message('nnet_cnn:internal:cnn:ClassificationValidationColumns:ValidationAccuracy'));
                currentColIdx = currentColIdx + 1;

                this.Column(currentColIdx).Name = "ValidationRMSE";
                this.Column(currentColIdx).Type = ".2f%";
                this.Column(currentColIdx).Header = getString(message('nnet_cnn:internal:cnn:RegressionValidationColumns:ValidationRMSE'));
                currentColIdx = currentColIdx + 1;
            end

            this.Column(currentColIdx).Name = "LearnRate";
            this.Column(currentColIdx).Type = ".4f";
            this.Column(currentColIdx).Header = getString(message('nnet_cnn:internal:cnn:RegressionColumns:LearnRate'));
            
            rawHeaders = {this.Column.Header};
            this.TableHeaderManager = nnet.internal.cnn.util.ProgressDisplayTableHeader( ...
                rawHeaders );
            
            columnWidths = this.TableHeaderManager.ColumnWidths;
            
            % Adjust accuracy width to take the extra % into account
            columnWidths(5) = columnWidths(5) - 1;
            columnWidths(7) = columnWidths(7) - 1;
            
            % Format columns
            this.Column = this.formatColumns( this.Column, columnWidths );
            
            % Undo the width adjustment, so that the real width is stored
            this.Column(5).Width = this.Column(5).Width + 1;
            this.Column(7).Width = this.Column(7).Width + 1;

            if isValidationDataSpecified
                % Adjust accuracy width to take the extra % into account
                columnWidths(10) = columnWidths(10) - 1;
                % Format columns
                this.Column = this.formatColumns( this.Column, columnWidths );
                % Undo the width adjustment, so that the real width is stored
                this.Column(10).Width = this.Column(10).Width + 1;
            end
        end
    end
    
    methods
        function horizontalBorder = get.HorizontalBorder(this)
            horizontalBorder = this.TableHeaderManager.HorizontalBorder;
        end
        
        function headings = get.Headings(this)
            headings = this.TableHeaderManager.Header;
        end
    end
end
