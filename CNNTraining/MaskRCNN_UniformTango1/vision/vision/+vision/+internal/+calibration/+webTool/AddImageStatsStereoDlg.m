classdef AddImageStatsStereoDlg < vision.internal.calibration.webTool.AddImageStatsDlg
    
    %   Copyright 2014-2024 The MathWorks, Inc.
    
    methods
        function this = AddImageStatsStereoDlg(stats, rejectedFileNames, location)
            this = this@vision.internal.calibration.webTool.AddImageStatsDlg(...
                stats, rejectedFileNames, location);
        end
    end
    
    methods(Access=protected)
        %------------------------------------------------------------------
        function setHeadingStringNoDuplicates(this)
            this.HeadingString = vision.getMessage(...
                'vision:caltool:NumDetectedBoardsStereo');
        end
        
        %------------------------------------------------------------------
        function setHeadingStringWithDuplicates(this)
            this.HeadingString = vision.getMessage(...
                'vision:caltool:AddBoardStatisticsStereo');
        end
    end
end
