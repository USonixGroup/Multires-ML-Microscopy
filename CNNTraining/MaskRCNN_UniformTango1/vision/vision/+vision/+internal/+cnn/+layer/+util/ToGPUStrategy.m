classdef ToGPUStrategy
    % Move data to GPU.
    
    % Copyright 2019 The Mathworks, Inc.
    methods
        function data = toDevice(~,data)
            data = gpuArray(data);
        end
    end
    
end