classdef disparitySGMBuildable < coder.ExternalDependency %#codegen
    % disparitySGMBuildable - encapsulate Semi-Global Block Matching
    % algorithm
    
    % Copyright 2018-2019 The MathWorks, Inc.
    
    methods (Static)
        
        function name = getDescriptiveName(~)
            name = 'disparitySGMBuildable';
        end
        
        function b = isSupportedContext(context)
            b = context.isMatlabHostTarget();
        end
        
        function updateBuildInfo(buildInfo, context)
            
            vision.internal.buildable.cvstBuildInfo(buildInfo, context, ...
                'disparitySGM', {'use_tbb'});
        end
        
        %------------------------------------------------------------------
        % write all supported data-type specific function calls
        %------------------------------------------------------------------
        function outDisparity = disparitySGMCompute(image1, image2, opt)
            
            coder.inline('always');
            coder.cinclude('cvstCG_disparitySGM.h');
            
            nRows = int32(size(image1, 1)); % original(before transpose)
            nCols = int32(size(image1, 2)); % original(before transpose)
            outSize = [nRows nCols];
            
            maxD = opt.NumberOfDisparities + opt.MinDisparity;
            D = opt.NumberOfDisparities;
            
            % Width after subtracting Disparity Levels as we don't calculate
            % disparity for max disparity columns
            if(opt.MinDisparity < 0)
                if(maxD < 0)
                    costImgWidth = (nCols + opt.MinDisparity);
                else
                    costImgWidth = (nCols - opt.NumberOfDisparities);
                end
            else
                costImgWidth = (nCols - maxD);
            end
            
            % Memory required for Census Transform images
            leftCT = coder.nullcopy(zeros(outSize, 'uint32'));
            rightCT = coder.nullcopy(zeros(outSize, 'uint32'));
            
            % Size required for Matching Cost and Directional cost sum
            if(opt.MinDisparity >= 0)
                matchCostDims = [nRows (nCols - maxD) * opt.NumberOfDisparities];
            else
                matchCostDims = [nRows (nCols - opt.NumberOfDisparities) * opt.NumberOfDisparities];
                if(maxD < 0)
                    matchCostDims = [nRows (nCols + opt.MinDisparity) * opt.NumberOfDisparities];
                end
            end
            
            % Memory for Matching cost and Directional cost sum
            matchingCost = coder.nullcopy(zeros(matchCostDims, 'int16'));
            dirCostSum = coder.nullcopy(zeros(matchCostDims, 'int16'));
            
            NR = 3; % 3 - Top three directions
            % We shift the pointers by 8(8 * sizeof(short) == 16 -ideal alignment)
            % for SSE instruction so added a border of 16 bytes
            D2 = D + 16;
            LrBorder = 1;
            minLrSize = (costImgWidth + 2 * LrBorder) * NR;
            minBufDirCostDims = [1 minLrSize];
            % Intermediate memory for one row of minimum directional cost across all
            % disparity levels
            oneRowMinDirCostBuf1 = coder.nullcopy(zeros(minBufDirCostDims, 'int16'));
            oneRowMinDirCostBuf2 = coder.nullcopy(zeros(minBufDirCostDims, 'int16'));
            
            LrSize = minLrSize * D2 + LrBorder;
            rowDirCostBufDims = [1 LrSize];
            % Intermediate memory to store One row of directional costs
            oneRowDirCostBuf1 = coder.nullcopy(zeros(rowDirCostBufDims, 'int16'));
            oneRowDirCostBuf2 = coder.nullcopy(zeros(rowDirCostBufDims, 'int16'));
            
            % Intermediate memory required for 10 threads while calculating
            % directional costs sum
            tenRowDirCostBufDims = [1 LrSize*10];
            tenRowDirCostBuf1 = coder.nullcopy(zeros(tenRowDirCostBufDims, 'int16'));
            tenRowDirCostBuf2 = coder.nullcopy(zeros(tenRowDirCostBufDims, 'int16'));
            
            % Parameters required for
            paramStruct = struct( ...
                'MinDisparity', int32(opt.MinDisparity), ...
                'NumberOfDisparities', int32(opt.NumberOfDisparities), ...
                'UniquenessThreshold', int32(opt.UniquenessThreshold), ...
                'Directions', int32(opt.Directions), ...
                'Penalty1', int32(opt.Penalty1), ...
                'Penalty2', int32(opt.Penalty2) );
            
            coder.cstructname(paramStruct, 'cvstDisparitySGMStruct_T', 'extern');
            
            if coder.isColumnMajor
                
                % Transpose the input images as MATLAB takes column major
                % and C-Code takes row major
                I1Transpose = transpose(image1);
                I2Transpose = transpose(image2);
                sizeTranspose = [nCols nRows];
                outDispTranspose = coder.nullcopy(zeros(sizeTranspose, 'single'));
                
                coder.ceval('-col', 'disparitySGMCompute',...
                    coder.ref(I1Transpose), ...
                    coder.ref(I2Transpose), ...
                    coder.ref(leftCT), ...
                    coder.ref(rightCT), ...
                    coder.ref(matchingCost), ...
                    coder.ref(oneRowMinDirCostBuf1), ...
                    coder.ref(oneRowMinDirCostBuf2), ...
                    coder.ref(oneRowDirCostBuf1), ...
                    coder.ref(oneRowDirCostBuf2), ...
                    coder.ref(tenRowDirCostBuf1), ...
                    coder.ref(tenRowDirCostBuf2), ...
                    coder.ref(dirCostSum), ...
                    nRows, nCols, ...
                    coder.ref(outDispTranspose), ...
                    coder.ref(paramStruct));
                
                % Transpose the output
                outDisparity = transpose(outDispTranspose);
                
            else
                
                outDisparity= coder.nullcopy(zeros(outSize, 'single'));
                
                coder.ceval('-row', 'disparitySGMCompute',...
                    coder.ref(image1), ...
                    coder.ref(image2), ...
                    coder.ref(leftCT), ...
                    coder.ref(rightCT), ...
                    coder.ref(matchingCost), ...
                    coder.ref(oneRowMinDirCostBuf1), ...
                    coder.ref(oneRowMinDirCostBuf2), ...
                    coder.ref(oneRowDirCostBuf1), ...
                    coder.ref(oneRowDirCostBuf2), ...
                    coder.ref(tenRowDirCostBuf1), ...
                    coder.ref(tenRowDirCostBuf2), ...
                    coder.ref(dirCostSum), ...
                    nRows, nCols, ...
                    coder.ref(outDisparity), ...
                    coder.ref(paramStruct));
            end
        end
    end
end
