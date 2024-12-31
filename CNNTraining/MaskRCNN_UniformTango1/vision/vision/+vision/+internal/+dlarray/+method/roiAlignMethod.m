classdef roiAlignMethod < deep.internal.dlarray.extension.DlarrayMethod
    % dlarray extension method that implements roiAlign
    
    %   Copyright 2020-2021 The MathWorks, Inc.
    
    properties
        GridSize
        SamplingRatio
        SizeX
    end
    
    methods
        
        function obj = roiAlignMethod(gridSize, samplingRatio, sizeX)
            obj.GridSize = double(gridSize);
            obj.SamplingRatio = single(samplingRatio);
            obj.SizeX = sizeX;
        end
        
        function [inFormats, outFormats] = forwardFormats(~, inFormats, outFormats)
           
            % Output format will match the input
            inFormats{1} = 'SSCB';
            outFormats{1} = 'SSCB';
        end
        
        function [inNeeded, outNeeded] = backwardRequirements(~, inNeeded, outNeeded)
            inNeeded(1) = false;  % X
            inNeeded(2) = true;  % roi
            outNeeded(1) = false; % Z
        end
        
        function Z = forward(obj, X, roi)
            
            % Handle complex inputs
            if ~isreal(X)
                error(message('deep:dlarray:ComplexNotSupported'));
            end

            % Call the appropriate builtin based on type of input (GPU/Host)
            if(isa(X,'gpuArray'))
                
                % This is a temporary fix for mxIsDouble failing for
                % gpuArrays in the GPU builtins.Since Bdeepleaf doesn't
                % build vision, we are making this fix till vision and
                % Bdeepleaf sync up.
                if(isgpuarray(roi))
                    roi = gather(roi);
                end
                Z = visiongpuROIAlignForward(single(X),double(roi'),...
                                             obj.GridSize(1),obj.GridSize(2),...
                                             obj.SamplingRatio);
            else
                Z = visionROIAlignForward(single(X),single(roi'),...
                                          obj.GridSize(1),obj.GridSize(2),...
                                          obj.SamplingRatio);
            end
                        
        end
        
        function [dX, dROI] = backward(obj,dZ, roi)
            
            % Handle complex gradients - cast complex to real as roialign
            % does not support complex inputs.
            dZ = real(dZ);

            % Call the appropriate builtin based on type of input (GPU/Host)
            if(isa(dZ,'gpuArray'))
                % This is a temporary fix for mxIsDouble failing for
                % gpuArrays in the GPU builtins.Since Bdeepleaf doesn't
                % build vision, we are making this fix till vision and
                % Bdeepleaf sync up.
                if(isgpuarray(roi))
                    roi = gather(roi);
                end
                dX = visiongpuROIAlignBackward(single(obj.SizeX),double(roi'),...
                                               single(dZ), ...
                                               obj.SamplingRatio);
            else
                
                dX = visionROIAlignBackward(single(obj.SizeX), single(roi'),...
                                            single(dZ),...
                                            single(obj.SamplingRatio));
            end
            dROI = zeros(size(roi),'like',roi); % no learnable parameters.
            
        end
        
        function higherOrderError(~)
            %higherOrderError Throw an error for higher-order gradients
            %
            %   higherOrderError(meth) is called when a user attempts to perform a
            %   double-backward on a DlarrayMethod-based operation.  
            error(message('deep:operations:UnsupportedDoubleBackward', 'roialign'));
        end
    end
end
