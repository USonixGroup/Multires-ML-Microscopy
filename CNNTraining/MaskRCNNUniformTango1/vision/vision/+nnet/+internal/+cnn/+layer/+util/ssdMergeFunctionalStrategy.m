classdef ssdMergeFunctionalStrategy < nnet.internal.cnn.layer.util.FunctionalStrategy
    % ssdMergeFunctionalStrategy  Execution strategy for SSD in dlnetwork.
    %
    % X is organized as a cell array, each element corresponding to
    % one feature map. The contents of X are in the shape of
    % [M-by-N-by-C-by-B] where M is the width of the feature map, N
    % is the height, C is the depth, and B is the number of
    % batches. The output is of size F-by-1-by-C-by-B, where F is
    % the number of anchor boxes for all the feature maps (in order
    % of their appearance in X).

    %   Copyright 2023-2024 The MathWorks, Inc.

    methods
        function Z = forward(this, X)
    
            fmt = dims(X);
            X = stripdimes(X);
            sizeOfZ = 0;

            if ~iscell(X)
                X = {X};
            end

            for idx = 1:this.NumInputs
                [h, w, c, b] = size(X{idx});
                numBoxesPerGrid = c/this.NumChannels;
                sizeOfZ = sizeOfZ + h*w*numBoxesPerGrid;
            end
            Z = zeros(sizeOfZ, 1, this.NumChannels, b, 'like', X{1});

            idxZ = 1;
            for idx = 1:this.NumInputs
                [h, w, c, b] = size(X{idx});

                % Make row order instead of column order.
                thisX = permute(X{idx}, [2 1 3 4]);

                numBoxesPerGrid = c/this.NumChannels;
                thisX = reshape(thisX, [h*w*numBoxesPerGrid, 1, this.NumChannels, b]);

                % Convert Z to gpuArray to handle cpu and gpuArray data.
                if isgpuarray(thisX)
                    Z = gpuArray(Z);
                end

                Z(idxZ:idxZ + h*w*numBoxesPerGrid - 1, :, :, :) = thisX;
                idxZ = idxZ + h*w*numBoxesPerGrid;
            end
            Z = dlarray(Z,fmt);
        end
    end
end
