classdef MonoVisualSLAMBuildable < vision.internal.buildable.BaseVisualSLAMBuildable

% Copyright 2022-2024 The MathWorks, Inc.

%#codegen
    properties (Constant, Access = private)
        APIHeaderFile = "vslam_core_api.hpp";
    end

    properties (Access = protected)
        SlamInternal
    end

    methods (Static)
        function name = getDescriptiveName(~)
            %getDescriptiveName
            name = "MonoVisualSLAMBuildable";
        end
    end

    methods
        function obj = MonoVisualSLAMBuildable(vslamObj, vocabFilePath)
            %isSupportedContext
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);

            baseParams = obj.getBaseParamsCStruct(vslamObj);
            imuParams = obj.getIMUParamsCStruct(vslamObj);
            camToIMU = vslamObj.CameraToIMUTransform.A;

            obj.SlamInternal = coder.opaquePtr('void', coder.internal.null);
            obj.SlamInternal = coder.ceval("-row", "MonoVisualSLAM_constructor", ...
                vslamObj.Intrinsics.FocalLength(1), vslamObj.Intrinsics.FocalLength(2), ...
                vslamObj.Intrinsics.PrincipalPoint(1)-1, vslamObj.Intrinsics.PrincipalPoint(2)-1, ...
                coder.rref(baseParams), coder.rref(imuParams), ...
                coder.rref(vocabFilePath), int32(vslamObj.ThreadLevel), ...
                coder.rref(camToIMU));
        end

        function addFrame(obj, Iu8_gray, imuG, imuA)
            %addFrame
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);

            nRows = int32(size(Iu8_gray, 1));
            nCols = int32(size(Iu8_gray, 2));

            imuSize = int32(size(imuG, 1));

            coder.ceval("-row", "MonoVisualSLAM_addFrame", obj.SlamInternal, ...
                coder.rref(Iu8_gray), nRows, nCols, coder.rref(imuG), ...
                coder.rref(imuA), imuSize);
        end

        function storeGravityRotationAndScale(obj, gRot, scale)
            %storeGravityRotationAndScale
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);

            coder.ceval("-row", "MonoVisualSLAM_storeGravityRotationAndScale", obj.SlamInternal, ...
                coder.rref(gRot), scale(1)); % Enforce scale input is scalar so codegen does not try to pass an array. This is needed because estimateGravityRotationAndPoseScale does not specify the output size properly for scale.
        end

    end
end

