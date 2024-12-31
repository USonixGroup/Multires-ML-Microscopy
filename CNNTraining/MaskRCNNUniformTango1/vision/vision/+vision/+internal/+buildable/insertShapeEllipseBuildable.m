classdef insertShapeEllipseBuildable < coder.ExternalDependency %#codegen
    % insertShapeEllipseBuildable - Buildable for insert ellipse shape
    
    % Copyright 2024 The MathWorks, Inc.
    
    methods (Static)
        
        function name = getDescriptiveName(~)
            name = 'insertShapeEllipseBuildable';
        end
        
        function b = isSupportedContext(~) % supports non-host target
            b = true;
        end
        
        %------------------------------------------------------------------
        % Update the buildinfo with required files
        %------------------------------------------------------------------
        function updateBuildInfo(buildInfo, context)
            buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','ocv', 'include'), fullfile(matlabroot,'toolbox', ...
                'images','opencv','opencvcg', 'include')});
            
            srcPaths = repmat({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','ocv')}, [1, 2]);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');
            buildInfo.addSourceFiles({'insertEllipseCore.cpp', 'cgCommon.cpp'},srcPaths);
            
            buildInfo.addIncludeFiles({'vision_defines.h','insertEllipseCore_api.hpp', 'cgCommon.hpp',});

            vision.internal.buildable.portableOpenCVBuildInfo(...
                buildInfo, context, 'insertEllipse');
        end

        %------------------------------------------------------------------
        % Invoke the insert ellipse algorithm
        %------------------------------------------------------------------
        function RGB = insertEllipse(image, positionOut, axesLength, angle,...
                fillShape, lineWidth, color, opacity, smoothEdges, shift)
            coder.inline('always');
            coder.cinclude('insertEllipseCore_api.hpp');
            imgClass = class(image);
            fcnName = ['insertEllipseCg_' imgClass];
            nRowsImg = int32(size(image, 1));
            nColsImg = int32(size(image, 2));
            numRowsPosition = int16(size(positionOut, 1));
            % Allocate memory for output image
            RGB = coder.nullcopy(zeros(size(image), imgClass));
            if coder.isColumnMajor
                coder.ceval('-col', fcnName, coder.ref(image), nRowsImg, nColsImg, ...
                    coder.ref(positionOut), coder.ref(axesLength), coder.ref(angle), ...
                    fillShape, lineWidth, coder.ref(color), ...
                    opacity, smoothEdges, shift, numRowsPosition, false, coder.ref(RGB));
            else
                positionTranspose = positionOut';
                axesLengthTransposed = axesLength';
                angleTranspose = angle';
                colorTranspose = color';
                coder.ceval('-row', fcnName, coder.ref(image), nRowsImg, nColsImg, ...
                    coder.ref(positionTranspose), coder.ref(axesLengthTransposed), coder.ref(angleTranspose),...
                    fillShape, lineWidth, coder.ref(colorTranspose), ...
                    opacity, smoothEdges, shift, numRowsPosition, true, coder.ref(RGB));
            end
        end
    end
end
