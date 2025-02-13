classdef CustomReshapeLayer < nnet.layer.Layer
    properties
        OutputSize
    end
    
    methods
        function layer = CustomReshapeLayer(outputSize, name)
            layer.Name = name;
            layer.OutputSize = outputSize;
            layer.Description = "Reshape layer to output size " + mat2str(outputSize);
        end
        
        function Z = predict(layer, X)
            % Reshape correctly: ensure batch size remains at the last dimension
            Z = reshape(X, [layer.OutputSize size(X, 4)]);
        end
        
        function Z = forward(layer, X)
            % Ensure reshape is correct during training
            Z = reshape(X, [layer.OutputSize size(X, 4)]);
        end
    end
end
