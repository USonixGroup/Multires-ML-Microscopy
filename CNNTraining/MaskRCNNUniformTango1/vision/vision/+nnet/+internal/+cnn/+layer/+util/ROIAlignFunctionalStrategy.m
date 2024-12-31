classdef ROIAlignFunctionalStrategy < ...
        nnet.internal.cnn.layer.util.FunctionalStrategy
    % ROIAlignFunctionalStrategy - Strategy definition for ROIAlign for
    % dlnetworks
    
    %   Copyright 2020-2021 The MathWorks, Inc.
    
    methods
        function [Z, memory] = forward(~, X, roi, gridSize, samplingRatio)
            
            assert( isa(X, 'dlarray') && isa(roi, 'dlarray') );
            
            roi = permute(roi, [2 1 3 4]);
            
            % Initialize dlarray extension method
            func = vision.internal.dlarray.method.roiAlignMethod(gridSize, samplingRatio, size(X));
            
            Z = deep.internal.dlarray.extension.applyExtensionMethod(func, X, gather(roi));
            
            memory = [];
        end
    end
end