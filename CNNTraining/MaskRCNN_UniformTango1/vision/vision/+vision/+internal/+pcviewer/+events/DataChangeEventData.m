classdef (ConstructOnLoad) DataChangeEventData < event.EventData
% Event data for any toolstrip events in pcviewer

%   Copyright 2022 The MathWorks, Inc.
    
    properties
        Data
    end
    
    methods 
        %------------------------------------------------------------------
        function this = DataChangeEventData(data)
            this.Data = data; 
        end
    end
end
