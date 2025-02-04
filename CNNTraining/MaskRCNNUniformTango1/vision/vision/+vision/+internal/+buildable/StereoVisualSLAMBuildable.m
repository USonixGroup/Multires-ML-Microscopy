classdef StereoVisualSLAMBuildable < vision.internal.buildable.BaseVisualSLAMBuildable

% Copyright 2023-2024 The MathWorks, Inc.

%#codegen
    properties (Constant, Access = private)
        APIHeaderFile = "stereovslam_core_api.hpp";
    end

    properties (Access = protected)
        SlamInternal
    end

    methods (Static)
        function name = getDescriptiveName(~)
            %getDescriptiveName
            name = "StereoVisualSLAMBuildable";
        end
    end

    methods
        function obj = StereoVisualSLAMBuildable(vslamObj, vocabFilePath)
            %isSupportedContext
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);

            baseParams = obj.getBaseParamsCStruct(vslamObj);
            stereoParams = obj.getStereoParamsCStruct(vslamObj);

            obj.SlamInternal = coder.opaquePtr('void', coder.internal.null);
            obj.SlamInternal = coder.ceval("StereoVisualSLAM_constructor", ...
                vslamObj.Intrinsics.FocalLength(1), vslamObj.Intrinsics.FocalLength(2), ...
                vslamObj.Intrinsics.PrincipalPoint(1)-1, vslamObj.Intrinsics.PrincipalPoint(2)-1, ...
                vslamObj.Baseline, coder.rref(baseParams), coder.rref(stereoParams), ...
                coder.rref(vocabFilePath), int32(vslamObj.ThreadLevel));
        end

        function addFrame(obj, I1u8_gray, I2u8_gray, disparity)
            %addFrame
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);

            nRows = int32(size(I1u8_gray, 1));
            nCols = int32(size(I1u8_gray, 2));

            coder.ceval("-row", "StereoVisualSLAM_addFrame", obj.SlamInternal, ...
                coder.rref(I1u8_gray), coder.rref(I2u8_gray), ...
                coder.rref(disparity), nRows, nCols);
        end
    end

    %----------------------------------------------------------------------
    % Setup C-structs
    %----------------------------------------------------------------------
    methods (Access = protected)
        function stereoParams = getStereoParamsCStruct(obj, vslamObj)
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);

            % Create MATLAB struct that will be represented as a
            % StereoParams struct in generated code. See note in equavalent
            % line in getBaseParamsCStruct.
            stereoParams = struct('minDisparity', int32(0), ...
                                  'maxDisparity', int32(0), ...
                                  'maxDepthFactor', single(0), ...
                                  'uniquenessThreshold', int32(0));

            % Tell MATLAB Coder to use the StereoParams struct from
            % ParameterStruct.hpp in the generated code.
            coder.cstructname(stereoParams, 'StereoParams', ...
                'extern', 'HeaderFile', 'ParameterStruct.hpp');

            % Set the default StereoParams field values.
            stereoParams = coder.ceval("StereoVisualSLAM_defaultStereoParams");

            % Set the StereoParams field values using vslamObj properties.
            stereoParams.minDisparity = int32(vslamObj.DisparityRange(1));
            stereoParams.maxDisparity = int32(vslamObj.DisparityRange(2));
            stereoParams.uniquenessThreshold = int32(vslamObj.UniquenessThreshold);
        end
    end
end
