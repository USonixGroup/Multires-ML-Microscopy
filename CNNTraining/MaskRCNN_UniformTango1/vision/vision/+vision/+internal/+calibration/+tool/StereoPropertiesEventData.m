classdef (ConstructOnLoad) StereoPropertiesEventData < vision.internal.calibration.tool.PatterDetectorEventData
    % The StereoPropertiesEventData class encapsulates data needed for
    % stereo camera files and detector selection event.
    
    % Copyright 2021 The MathWorks, Inc.
    
    properties
        FileNames;
        Dir1;
        Dir2;
    end
    
    methods
        
        function data = StereoPropertiesEventData(propertiesDlgObj)
            data = data@vision.internal.calibration.tool.PatterDetectorEventData(propertiesDlgObj.CurrentDetector, ...
                propertiesDlgObj.CurrentDetectorFileName);
            
            data.FileNames = propertiesDlgObj.FileNames;
            data.Dir1      = propertiesDlgObj.Dir1;
            data.Dir2      = propertiesDlgObj.Dir2;
        end
        
    end
    
end