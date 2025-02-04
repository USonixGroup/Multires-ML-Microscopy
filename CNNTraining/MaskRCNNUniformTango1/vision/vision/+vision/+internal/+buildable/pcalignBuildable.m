classdef pcalignBuildable < coder.ExternalDependency %#codegen
    % pcalignBuildable - encapsulate pcalignBuildable implementation library
    
    % Copyright 2021-2022 The MathWorks, Inc.
    
    methods (Static)
        
        function name = getDescriptiveName(~)
            name = 'pcalignBuildable';
        end
        
        function b = isSupportedContext(~) % supports non-host target
            b = true;
        end
        
        function updateBuildInfo(buildInfo, ~)
            buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision','include')});
            
            srcPaths = fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision');
            buildInfo.addSourceFiles({'pcalignUtilsCore.cpp'}...
                , srcPaths);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');
            
            buildInfo.addIncludeFiles({'vision_defines.h', ...
                'pcalignUtils.hpp',...
                'pcalignUtilsCore_api.hpp'});
        end

        %------------------------------------------------------------------
        % Write all supported data-type specific function calls
        %------------------------------------------------------------------
        function [filteredLocation, filteredColor, filteredIntensity, filteredNormal]...
                = voxelGridFilter(tformedClouds, total, sortIndexVecIdx,...
                hasColor, hasNormal, hasIntensity, sizeArray)
            
            coder.inline('always');
            coder.cinclude('pcalignUtilsCore_api.hpp');
            
            numClouds = numel(tformedClouds);
            numPoints = sum(sizeArray);            
            
            locationDataType  = class(tformedClouds(1).Location);
            intensityDataType = class(tformedClouds(1).Intensity);
            colorDataType     = class(tformedClouds(1).Color);
            
            % Arrays to hold the pointCloud data
            location  = cast([], locationDataType);
            color     = cast([], colorDataType);
            normal    = cast([], locationDataType);
            intensity = cast([], intensityDataType);
            
            for i = 1:numClouds
                location  = [location;tformedClouds(i).Location];   %#ok
                color     = [color;tformedClouds(i).Color];         %#ok
                normal    = [normal;tformedClouds(i).Normal];       %#ok
                intensity = [intensity;tformedClouds(i).Intensity]; %#ok
            end
            
            if hasColor
                filteredColor = zeros(total, 3, colorDataType);
            else
                filteredColor = cast(zeros(0,0), colorDataType);
            end
            
            if hasNormal
                filteredNormal = zeros(total, 3, locationDataType);
            else
                filteredNormal = cast(zeros(0,0), locationDataType);
            end
            
            if hasIntensity
                filteredIntensity = zeros(total, 1, intensityDataType);
            else
                filteredIntensity = cast(zeros(0,0), intensityDataType);
            end
            
            indexVecSize     = uint32(size(sortIndexVecIdx, 1));
            filteredLocation = zeros(total, 3, locationDataType);
            
            functionName = getCppFunctionName(location, color, intensity);

            if coder.isColumnMajor
                
                coder.ceval('-col', functionName, ...
                        coder.rref(location), ...
                        coder.rref(color), ...
                        coder.rref(normal), ...
                        coder.rref(intensity), ...
                        coder.ref(filteredLocation), ...
                        coder.ref(filteredColor), ...
                        coder.ref(filteredNormal), ...
                        coder.ref(filteredIntensity), ...
                        coder.rref(sortIndexVecIdx), ...
                        uint32(numPoints), ...
                        hasColor, ...
                        hasNormal, ...
                        hasIntensity,...
                        indexVecSize,...
                        coder.rref(sizeArray),...
                        uint32(total));
            else
                
                filteredLocationRow = zeros(total*3, 1, locationDataType);
                
                if hasColor
                    filteredColorRow = zeros(total*3, 1, colorDataType);
                else
                    filteredColorRow = cast(zeros(0,0), colorDataType);
                end
                
                if hasNormal
                    filteredNormalRow = zeros(total*3, 1, locationDataType);
                else
                    filteredNormalRow = cast(zeros(0,0), locationDataType);
                end
                
                if hasIntensity
                    filteredIntensityRow = zeros(total, 1, intensityDataType);
                else
                    filteredIntensityRow = cast(zeros(0,0), intensityDataType);
                end

                coder.ceval('-row', functionName, ...
                    location(:), ...
                    color(:), ...
                    normal(:), ...
                    intensity(:), ...
                    coder.ref(filteredLocationRow), ...
                    coder.ref(filteredColorRow), ...
                    coder.ref(filteredNormalRow), ...
                    coder.ref(filteredIntensityRow), ...
                    sortIndexVecIdx(:), ...
                    uint32(numPoints), ...
                    hasColor, ...
                    hasNormal, ...
                    hasIntensity,...
                    indexVecSize,...
                    sizeArray(:),...
                    uint32(total));
                
                tempLoc             = filteredLocationRow';
                filteredLocation(:) = tempLoc(:);
                
                filteredColor     = reshape(filteredColorRow', size(filteredColor));
                filteredNormal    = reshape(filteredNormalRow', size(filteredNormal));
                filteredIntensity = reshape(filteredIntensityRow', size(filteredIntensity));
            end
        end
    end
end


function name = getCppFunctionName1(location, color, intensity)
    baseName = 'voxelGridAlgImpl';
    suffix1 = class(location);

    colorClass = 'uint8';
    if ~isempty(color) || isa(color, 'uint16')
        colorClass = class(color);
    end
    suffix2 = [colorClass 'Color'];
    name = strjoin({baseName, suffix1, suffix2},'_');

    suffix3 = [class(intensity) 'Intensity'];
    if isa(intensity,'uint8') || isa(intensity,'uint16')
        name = strjoin({name, suffix3},'_');
    end
end

function name = getCppFunctionName(location, color, intensity)

    if isa(intensity, 'uint8')
        if isa(color, 'uint16')
            name = ['voxelGridAlgImpl_' class(location) '_uint16Color_uint8Intensity'];
        else
            name = ['voxelGridAlgImpl_' class(location) '_uint8Color_uint8Intensity'];
        end
    elseif isa(intensity, 'uint16')
        if isa(color, 'uint16')
            name = ['voxelGridAlgImpl_' class(location) '_uint16Color_uint16Intensity'];
        else
            name = ['voxelGridAlgImpl_' class(location) '_uint8Color_uint16Intensity'];
        end
    else
        if isa(color, 'uint16')
            name = ['voxelGridAlgImpl_' class(location) '_uint16Color'];
        else
            name = ['voxelGridAlgImpl_' class(location) '_uint8Color'];
        end
    end
end