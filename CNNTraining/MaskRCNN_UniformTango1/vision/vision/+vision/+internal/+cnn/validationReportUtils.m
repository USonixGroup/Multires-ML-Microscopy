classdef validationReportUtils 
    % A collection of box conversion and scaling utilities.

    % A collection of validation report utilities.

    % Copyright 2019 The MathWorks, Inc.

    methods(Static)
        %--------------------------------------------------------------------------
        function opts = updateValidationDataTransform(opts, applyTransformFcn)
            if vision.internal.cnn.validationReportUtils.isValidationSpecified(opts)
                for ii = 1:numel(opts)
                    % Copy and reset the validation datastore
                    validationData = copy(opts.ValidationData);
                    reset(validationData);
                    opts(ii).ValidationData = applyTransformFcn(validationData);
                end
            end
        end

        %--------------------------------------------------------------------------
        function tf = isValidationSpecified(opts)
            tf = ~isempty(opts(1).ValidationData);
        end
    end
end
