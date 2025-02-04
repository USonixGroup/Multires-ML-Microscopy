function sequence = gpuOrCPUSequence(sequence,exe)
%gpuOrCPUSequence Based on execution environment create a gpuArray
% or gather the gpuArray.

%   Copyright 2021-2023 The MathWorks, Inc.
    switch exe
        case "gpu"
            if ~isgpuarray(sequence)
                sequence = gpuArray(sequence);
            end
        case "cpu"
            if isgpuarray(sequence)
                sequence = gather(sequence);
            end
    end
end
