classdef insertMarkerBuildable < coder.ExternalDependency %#codegen
    % insertMarkerBuildable.m - encapsulate insertMarkerBuildable.m implementation library
    
    % Copyright 2019-2022 The MathWorks, Inc.
    
    methods (Static)
        
        function name = getDescriptiveName(~)
            name = 'insertMarkerBuildable';
        end
        
        function b = isSupportedContext(~) % supports non-host target
            b = true;
        end
        
        function updateBuildInfo(buildInfo, ~)
            buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision','include'),...
                fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision','export','include','vision')});
            
            srcPaths = fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision');
            buildInfo.addSourceFiles({'insertMarkerUtilsCore.cpp', 'cvstDraw.cpp', ...
                'insertShapeAndMarkerUtils.cpp'}, srcPaths);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');
            
            buildInfo.addIncludeFiles({'vision_defines.h', ...
                'insertShapeAndMarkerUtils.hpp', 'insertMarkerUtilsCore_api.hpp',...
                'cvstDraw.hpp'});
        end
        
        %------------------------------------------------------------------
        %                      Constructor
        %------------------------------------------------------------------
        function ptrObj = drawBaseConstruct()
            
            coder.inline('always');
            coder.cinclude('insertMarkerUtilsCore_api.hpp');
            ptrObj = coder.opaquePtr('void', coder.internal.null);
            coder.ceval('constructDrawBaseObjectMarker', coder.ref(ptrObj));
            
        end
        
        %------------------------------------------------------------------
        % Write all supported data-type specific function calls
        %------------------------------------------------------------------
        function [RGB] = insertMarker(markerShape, imgClass, markerSize, ...
                I, position, color)
            
            coder.inline('always');
            coder.cinclude('insertMarkerUtilsCore_api.hpp');
            
            % Obtain number of rows, columns and colors
            [numRow, numCol, numColor] = size(I);
            numRow = int16(numRow);
            numCol = int16(numCol);
            numColor = int16(numColor);
            opacity = 0.6; % Default Value taken
            
            % Allocate memory for output image
            RGB = coder.nullcopy(zeros(size(I), imgClass));
            
            % Obtain shape to be inserted and its required properties
            numDimens = int16(numel(size(position)));
            dimens1 = int16(size(position, 1));
            dimens2 = int16(size(position, 2));
            numFillColor = int16(size(color, 1));
            if( numRow > numCol )
                pixCountSize = numRow;
            else
                pixCountSize = numCol;
            end
             pixCount = zeros(1, pixCountSize, 'uint8');
            
            %Calling the constructor of Draw Base Class
            ptrObj = vision.internal.buildable.insertMarkerBuildable.drawBaseConstruct();
            
            % Getting the function name w.r.t dataType
            fcnName = ['instantiateDrawBaseMarker_', imgClass];
            
            for i = 1:2
                isInitialise = false;
                isInitialise = coder.ceval('initialiseDrawbaseMarker', ptrObj, int16(i-1));
                
                if(~isInitialise)
                    % Instantiate the drawBase Object
                    if coder.isColumnMajor
                        coder.ceval('-col',fcnName, ptrObj, coder.ref(RGB), coder.ref(I),...
                            coder.ref(position), coder.ref(color), opacity, int32(markerShape),...
                            int32(markerSize), numRow, numCol, numColor, numDimens, dimens1, dimens2,...
                            numFillColor, coder.ref(pixCount), int16(i-1));
                    else
                        % Data required for row major support
                        Irow = I(:);
                        RGB_row = coder.nullcopy(zeros(size(Irow,1), 1, 'like', Irow));
                        coder.ceval('-row',fcnName, ptrObj, coder.ref(RGB_row), Irow, position', color',...
                            opacity, int32(markerShape), int32(markerSize), numRow, numCol, numColor,...
                            numDimens, dimens1, dimens2, numFillColor, pixCount', int16(i-1));
                        RGB = reshape(RGB_row', size(I));
                    end
                end
            end           
            
            %Deallocating the memory
            coder.ceval('deallocateMemoryMarker', ptrObj);
        end
    end
end
