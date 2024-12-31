classdef scanContextDescriptorBuildable < coder.ExternalDependency %#codegen
    % scanContextDescriptorBuildable.m - encapsulate scanContextDescriptorBuildable.m implementation library
    
    % Copyright 2020-2021 The MathWorks, Inc.
    
    methods (Static)
        
        function name = getDescriptiveName(~)
            name = 'scanContextDescriptorBuildable';
        end
        
        function b = isSupportedContext(~) % supports non-host target
            b = true;
        end
        
        function updateBuildInfo(buildInfo, ~)
            buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision','include')});
            
            srcPaths = fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision');
            buildInfo.addSourceFiles({'scanContextDescriptorUtilsCore.cpp',...
                }, srcPaths);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');
            
            buildInfo.addIncludeFiles({'vision_defines.h', ...
                'scanContextDescriptorUtils.hpp',...
                'scanContextDescriptorUtilsCore_api.hpp'});
        end
        
        
        %------------------------------------------------------------------
        % Write all supported data-type specific function calls
        %------------------------------------------------------------------
        function [descriptor] = scanContextDescriptor(location, params)
            
            coder.inline('always');
            coder.cinclude('scanContextDescriptorUtilsCore_api.hpp');
            
            numRadialBins    = uint16(params.NumBins(1));
            numAzimuthalBins = uint16(params.NumBins(2));
            minPointsPerBin  = uint16(params.MinPointsPerBin);
            
            if ismatrix(location)
                numPoints = uint32(size(location, 1));
            else
                numPoints = uint32(size(location, 1) * size(location, 2));
            end
            
            dataType = class(location);
            
            % Getting the function name w.r.t dataType
            fcnName = ['scanContextImageDescriptor_', dataType];
            
            % Calling the core C++ function to calculate the descriptor
            if coder.isColumnMajor
                % Allocate memory for output
                descriptor = zeros(numRadialBins, numAzimuthalBins, dataType);
                coder.ceval('-col', fcnName, coder.ref(location), numPoints,...
                    numRadialBins, numAzimuthalBins, minPointsPerBin, coder.ref(params.SensorOrigin),...
                    coder.ref(params.RadialRange), coder.ref(descriptor));
            else
                tempLocation = reshape(location, [], 1);
                tempDescriptor = zeros(numRadialBins*numAzimuthalBins, 1, dataType);
                coder.ceval('-row', fcnName, coder.ref(tempLocation), numPoints,...
                    numRadialBins, numAzimuthalBins, minPointsPerBin, coder.ref(params.SensorOrigin),...
                    coder.ref(params.RadialRange), coder.ref(tempDescriptor));
                descriptor = reshape(tempDescriptor, [numRadialBins numAzimuthalBins]);
            end
            
        end
    end
end