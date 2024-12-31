classdef ToHostStrategy
    % Move data to host.
    
    % Copyright 2019 The Mathworks, Inc.
    methods
        function data = toDevice(~,data)
            data = gather(data);
        end
    end
    
end