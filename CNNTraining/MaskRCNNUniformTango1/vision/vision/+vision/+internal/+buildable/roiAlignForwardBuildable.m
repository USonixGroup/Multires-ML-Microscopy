classdef roiAlignForwardBuildable < coder.ExternalDependency %#codegen
    % roiAlignForwardBuildable.m - encapsulate  roiAlignForwardBuildable.m implementation library

    % Copyright 2024 The MathWorks, Inc.

    methods (Static)

        function name = getDescriptiveName(~)
            name = 'roiAlignForwardBuildable';
        end

        function b = isSupportedContext(~) % supports non-host target
            b = true;
        end

        function updateBuildInfo(buildInfo, ~)

            srcPaths = fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision');
            buildInfo.addSourceFiles({'roiAlignForwardCore.cpp',...
                }, srcPaths);
            incPaths = fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision','include');
            buildInfo.addSourcePaths(srcPaths);
            buildInfo.addIncludePaths(incPaths);

            buildInfo.addIncludeFiles({'vision_defines.h', ...
                'roiAlignForwardCore_api.hpp','roiAlignForward.hpp'});
        end

        %---------------------------------------------------------------------
        % Function to visionROIAlignForward
        %---------------------------------------------------------------------
        function Z = roiAlignForward(X, roi, outputSize, samplingRatio)

            coder.inline('always');
            coder.cinclude('roiAlignForwardCore_api.hpp');

            gridHeight = outputSize(1);
            gridWidth = outputSize(2);

            if samplingRatio == -1
                samplingRatioNew = [-1,-1];
            else
                samplingRatioNew = samplingRatio;
            end

            xDim = uint32(size(X, 1));
            yDim = uint32(size(X, 2));
            numChannels= uint32(size(X, 3));

            fifthRow = roi(5,:);
            columnIndices = find(fifthRow == 0);

            if ~isempty(columnIndices)
                numROI = uint32(columnIndices(1)-1);
            else
                numROI = uint32(size(roi, 2));
            end

            xData = single(X);
            roiData = single(roi);
            samplingRatioX = cast(samplingRatioNew(1), 'int32');
            samplingRatioY = cast(samplingRatioNew(2), 'int32');

            % Copy pooled data to output
            if coder.isColumnMajor
                Z = zeros(gridHeight, gridWidth, size(X, 3), size(roi, 2), 'single');
                coder.ceval('-col','roiAlignForward', coder.ref(xData), coder.ref(roiData), ...
                    uint32(gridHeight), uint32(gridWidth), samplingRatioX, ...
                    samplingRatioY, xDim, yDim, numChannels, ...
                    numROI, coder.ref(Z));
            else
                tempZ = zeros(gridHeight*gridWidth*size(X, 3)*size(roi, 2), 1, 'single');
                xTranspose = xData(:);
                roiTranspose = roiData';
                coder.ceval('-row','roiAlignForward', coder.ref(xTranspose), coder.ref(roiTranspose), ...
                    uint32(gridHeight), uint32(gridWidth), samplingRatioX, ...
                    samplingRatioY, xDim, yDim, numChannels, ...
                    numROI, coder.ref(tempZ));
                Z = reshape(tempZ, [gridHeight, gridWidth, size(X, 3), size(roi, 2)]);
            end
        end
    end
end