classdef (ConstructOnLoad) PatterDetectorEventData < event.EventData
    % The PatterDetectorEventData class encapsulates data needed for
    % pattern detector selection event.
    
    % Copyright 2021 The MathWorks, Inc.
    
    properties
        PatternDetector        
        PatternDetectorFile
    end
    
    methods
        
        function data = PatterDetectorEventData(detector, fileName)
            data.PatternDetector = detector;            
            data.PatternDetectorFile = fileName;
        end
        
    end
    
end