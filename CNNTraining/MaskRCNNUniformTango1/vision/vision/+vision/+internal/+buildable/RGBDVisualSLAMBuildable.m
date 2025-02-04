classdef RGBDVisualSLAMBuildable < vision.internal.buildable.BaseVisualSLAMBuildable

% Copyright 2023-2024 The MathWorks, Inc.

%#codegen
    properties (Constant, Access = private)
        APIHeaderFile = "rgbdvslam_core_api.hpp";
    end

    properties (Access = protected)
        SlamInternal
    end

    methods (Static)
        function name = getDescriptiveName(~)
            %getDescriptiveName
            name = "RGBDVisualSLAMBuildable";
        end
    end

    methods
        function obj = RGBDVisualSLAMBuildable(vslamObj, vocabFilePath)
            %isSupportedContext
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);

            baseParams = obj.getBaseParamsCStruct(vslamObj);
            rgbdParams = obj.getRGBDParamsCStruct(vslamObj);

            obj.SlamInternal = coder.opaquePtr('void', coder.internal.null);
            obj.SlamInternal = coder.ceval("RGBDVisualSLAM_constructor", ...
                vslamObj.Intrinsics.FocalLength(1), vslamObj.Intrinsics.FocalLength(2), ...
                vslamObj.Intrinsics.PrincipalPoint(1)-1, vslamObj.Intrinsics.PrincipalPoint(2)-1, ...
                coder.rref(baseParams), coder.rref(rgbdParams), ...
                coder.rref(vocabFilePath), int32(vslamObj.ThreadLevel));
        end

        function addFrame(obj, Iu8_gray, Isingle_depth)
            %addFrame
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);

            nRows = int32(size(Iu8_gray, 1));
            nCols = int32(size(Iu8_gray, 2));

            coder.ceval("-row", "RGBDVisualSLAM_addFrame", ...
                obj.SlamInternal, ...
                coder.rref(Iu8_gray), coder.rref(Isingle_depth), nRows, nCols);
        end
    end

    %----------------------------------------------------------------------
    % Setup C-structs
    %----------------------------------------------------------------------
    methods (Access = protected)
        function rgbdParams = getRGBDParamsCStruct(~, vslamObj)
            coder.inline('always');

            % Create MATLAB struct that will be represented as a
            % RGBDParams struct in generated code and assign values to
            % members.
            rgbdParams = struct('depthScaleFactor', vslamObj.DepthScaleFactor, ...
                                'depthRange', single([vslamObj.DepthRange(1), vslamObj.DepthRange(2)]));

            % Tell MATLAB Coder to use the StereoParams struct from
            % ParameterStruct.hpp in the generated code.
            coder.cstructname(rgbdParams, 'RGBDParams', ...
                'extern', 'HeaderFile', 'ParameterStruct.hpp');
        end
    end
end

