%   Copyright 2019 The MathWorks, Inc.

%% GPU coder implementation of disparitySGM
% The inputs are rectified stereo pair images and parameters required to
% process. The output is a disparity map.
%#codegen
function outDisparityMap = gpudisparitySGMCompute(I1, I2, opt)

coder.allowpcode('plain');

[numRows, numCols] = size(I1);
outputSize = size(I1); % Output size is same as input

numDisparity = opt.NumberOfDisparities;
maxDisparity = opt.MinDisparity + numDisparity;
paths = opt.Directions;

% Get the memory mode
coder.extrinsic('gpufeature');
memoryModeString = coder.const(@gpufeature,'gpuMemoryMode');

if( strcmp(memoryModeString,'low') )
    memoryMode = 0; % Low GPU memory
else
    memoryMode = 1; % Default:High GPU memory
end

% Transpose inputs as the GPU code is Row-major
I3 = transpose(I1);
I4 = transpose(I2);

% Output memory
disparityMap = coder.nullcopy(zeros(outputSize, 'single'));

costImgWidth = numCols - maxDisparity;
if (opt.MinDisparity < 0)
    costImgWidth = numCols - numDisparity;
    if (maxDisparity < 0)
        costImgWidth = numCols + opt.MinDisparity;
    end
end

censusTransformSize = numCols * numRows;
costImgSize = costImgWidth * numRows * numDisparity;

% Amount of intermediate memory for algorithm processing
if(memoryMode)
    totalMemRequired = (double(costImgSize)) * ((double(paths)) + 1) + censusTransformSize * 4 * 2;
else
    totalMemRequired = (double(costImgSize)) * 3 + censusTransformSize * 4 * 2;
end

% Error out if required memory for both low and high memory mode are
% greater than intmax.
if(totalMemRequired <= double(intmax))
    gpuWorkspace = coder.nullcopy(zeros([1 totalMemRequired],'uint8'));
else
    % Required memory for Low memory mode
    totalMemRequired = (double(costImgSize)) * 3 + censusTransformSize * 4 * 2;
    if(totalMemRequired <= double(intmax))
        gpuWorkspace = coder.nullcopy(zeros([1 totalMemRequired],'uint8'));
        memoryMode = 0;
    else
        coder.internal.errorIf(totalMemRequired > double(intmax), 'vision:disparity:allowedMemoryExceededSGM');
        % Initialize gpuWorkspace with one element so that it is defined in
        % all the code paths.
        gpuWorkspace = coder.nullcopy(zeros(1,'uint8'));
    end
end

% Specify PTX files needed
coder.gpu.internal.includePtx(...
    'toolbox/vision/builtins/src/visiongpudevice/wrapper/disparitySGMWrapperCuda.hpp', ...
    'toolbox/vision/builtins/src/visiongpudevice/wrapper/disparitySGMWrapperCuda.cpp', ...
    'toolbox/vision/builtins/src/visiongpudevice/wrapper/gpudisparitySGMCuda.hpp', ...
    'toolbox/vision/builtins/src/visiongpudevice/wrapper/gpudisparitySGMCostCuda.hpp', ...
    'toolbox/vision/builtins/src/visiongpudevice/export/include/visiongpudevice/gpudisparitySGMConfig.hpp', ...
    'toolbox/vision/builtins/src/visiongpudevice/ptxfiles/gpudisparitySGM_mw_ptx.cu', ...
    'toolbox/vision/builtins/src/visiongpudevice/ptxfiles/gpudisparitySGMCost_mw_ptx.cu');

coder.ceval('computeDisparity',int32(memoryMode), uint8(opt.Penalty1), uint8(opt.Penalty2), ...
    int32(numRows), int32(numCols), int32(paths), int32(opt.MinDisparity), int32(numDisparity), ...
    int32(opt.UniquenessThreshold), coder.ref(I3(1,1), 'gpu'), coder.ref(I4(1,1), 'gpu'), ...
    coder.ref(disparityMap(1,1), 'gpu'), coder.ref(gpuWorkspace(1,1), 'gpu') );

% Reshape the data as the output in GPU code is computed in Row-Major
outDisparityMap = transpose(reshape(disparityMap,[outputSize(2) outputSize(1)]));

end
