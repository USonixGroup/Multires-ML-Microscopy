classdef insertShapeBuildable < coder.ExternalDependency %#codegen
    % insertShapeBuildable - encapsulate insertShapeBuildable implementation library
    
    % Copyright 2019-2024 The MathWorks, Inc.
    
    methods (Static)
        
        function name = getDescriptiveName(~)
            name = 'insertShapeBuildable';
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
            buildInfo.addSourceFiles({'insertShapeUtilsCore.cpp', 'cvstDraw.cpp', ...
                'insertShapeAndMarkerUtils.cpp'},srcPaths);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');
            
            buildInfo.addIncludeFiles({'vision_defines.h', ...
                'insertShapeAndMarkerUtils.hpp', 'insertShapeUtilsCore_api.hpp',...
                'cvstDraw.hpp'});
        end
        
        %------------------------------------------------------------------
        %                      Constructor
        %------------------------------------------------------------------
        function ptrObj = drawBaseConstruct()
            
            coder.inline('always');
            coder.cinclude('insertShapeUtilsCore_api.hpp');
            ptrObj = coder.opaquePtr('void', coder.internal.null);
            coder.ceval('constructDrawBaseObjectShape', coder.ref(ptrObj));
            
        end
        
        %------------------------------------------------------------------
        %                    function to get position data pointer
        %------------------------------------------------------------------
        function posPtr = getPositionPointer(positionOut,dimens1, dimens2)
            
            coder.inline('always');
            coder.cinclude('insertShapeUtilsCore_api.hpp');
            posPtr = coder.opaquePtr('void', coder.internal.null);
            coder.ceval('getPositionDataPointer', coder.ref(posPtr), coder.ref(positionOut),...
                dimens1, dimens2 );
            
        end
        
        %------------------------------------------------------------------
        %                    function to get color data pointer
        %------------------------------------------------------------------
        function colPtr = getColorPointer(color,imgClass, dimensc1, dimensc2)
            
            coder.inline('always');
            coder.cinclude('insertShapeUtilsCore_api.hpp');
            colPtr = coder.opaquePtr('void', coder.internal.null);
            % Getting the function name w.r.t dataType
            fcnName = ['getColorDataPointer_', imgClass];
            coder.ceval(fcnName, coder.ref(colPtr), coder.ref(color),...
                dimensc1, dimensc2 );
            
        end
        
        %------------------------------------------------------------------
        %                    function to get ptsDW pointer
        %------------------------------------------------------------------
        function ptsDWPtr = getPtsDWPointer(shape,...
                numDimens, dimens1, dimens2)
            
            coder.inline('always');
            coder.cinclude('insertShapeUtilsCore_api.hpp');
            ptsDWPtr = coder.opaquePtr('void', coder.internal.null);
            coder.ceval('getPtsDWPointer', coder.ref(ptsDWPtr),...
                shape, numDimens, dimens1, dimens2);
            
        end
        
        %------------------------------------------------------------------
        % Write all supported data-type specific function calls
        %------------------------------------------------------------------
        function [RGB] = insertShape(shape, fillShape, lineWidth, opacity,...
                smoothEdges, imgClass, I, positionOut, color)
            
            coder.inline('always');
            coder.cinclude('insertShapeUtilsCore_api.hpp');
            
            % Obtain number of rows, columns and colors
            [numRow, numCol, numColor] = size(I);
            numRow   = uint32(numRow);
            numCol   = uint32(numCol);
            numColor = uint32(numColor);
            % Allocate memory for output image
            RGB = zeros(size(I), imgClass);
            
            % Obtain shape to be inserted and its required properties
            shape = int32(shape);
            lineWidthArr = int32(lineWidth);
            isAntiAlias = smoothEdges;
            isFillShape = fillShape;
            numDimens = uint32(numel(size(positionOut)));
            dimens1 = uint32(size(positionOut, 1));
            dimens2 = uint32(size(positionOut, 2));
            dimensc1 = uint32(size(color, 1));
            dimensc2 = uint32(size(color, 2));
            numFillColor = uint32(size(color, 1));
            mimicFillPoly = [false, true];
            if( numRow > numCol )
                pixCountSize = numRow;
            else
                pixCountSize = numCol;
            end
            pixCount = zeros(1, pixCountSize, 'uint8');
            
            %Calling the constructor of Draw Base Class
            ptrObj = vision.internal.buildable.insertShapeBuildable.drawBaseConstruct();
            
            %Initializing ploygon edge pointer
            polygonEdgePtr = coder.opaquePtr('void', coder.internal.null);
            
            %Calling the function to get position data pointer
            posPtr = vision.internal.buildable.insertShapeBuildable.getPositionPointer(positionOut,...
                dimens1, dimens2);
            
            %Calling the function to get ptsDW and polygonEdge pointer
            ptsDWPtr = vision.internal.buildable.insertShapeBuildable.getPtsDWPointer(shape,...
                numDimens, dimens1, dimens2);
            
            %Calling the function to get color data pointer
            colPtr = vision.internal.buildable.insertShapeBuildable.getColorPointer(color,imgClass,...
                dimensc1, dimensc2);
            
            % Getting the function name w.r.t dataType
            fcnName = ['instantiateDrawBaseShape_', imgClass];
            
            % Data required for row major support
            Irow = I(:);
            RGB_row = zeros(size(Irow,1), 1, 'like', Irow);
            
            for i = 1:2
                isInitialise = false;
                isInitialise = coder.ceval('initialiseDrawbaseShape', ptrObj, uint32(i-1), int32(shape));
               
                if(~isInitialise)
                    coder.ceval('allocatePolygonEdgePointer', coder.ref(polygonEdgePtr), int32(shape),...
                        numDimens, dimens1, dimens2, isFillShape, mimicFillPoly(i));
                   
                    % Instantiate the drawBase Object
                    if coder.isColumnMajor
                        coder.ceval('-col',fcnName, ptrObj, coder.ref(RGB), coder.ref(I),...
                            posPtr, colPtr, opacity, lineWidthArr, shape,...
                            isAntiAlias, numRow, numCol, numColor, numDimens, dimens1, dimens2, ...
                            numFillColor, isFillShape, mimicFillPoly(i), coder.ref(pixCount),...
                            uint32(i-1), ptsDWPtr, polygonEdgePtr);
                    else
                        coder.ceval('-row',fcnName, ptrObj, coder.ref(RGB_row), Irow, posPtr,...
                            colPtr, opacity, lineWidthArr, shape, isAntiAlias, numRow, numCol, ...
                            numColor, numDimens, dimens1, dimens2, numFillColor, isFillShape, ...
                            mimicFillPoly(i), pixCount', uint32(i-1), ptsDWPtr, polygonEdgePtr);
                    end
                end
            end
            
            %Calling the DrawShapes function
            coder.ceval('mDrawShapes', ptrObj, isFillShape, isAntiAlias, int32(shape), int32(lineWidth),...
                int32(numRow), int32(numCol));
            
            if ~(coder.isColumnMajor)
                RGB = reshape(RGB_row', size(I));
            end
          
            %Deallocating the memory
            coder.ceval('deallocateMemoryShape', ptrObj);
            coder.ceval('deallocatePolygonEdgePointer', polygonEdgePtr);
            coder.ceval('deletePositionDataPointer', posPtr);
            coder.ceval('deletePtsDWPointer', ptsDWPtr);
            % Getting the function name w.r.t dataType
            fcnName = ['deleteColorDataPointer_', imgClass];
            coder.ceval(fcnName, colPtr);
        end
    end
end
