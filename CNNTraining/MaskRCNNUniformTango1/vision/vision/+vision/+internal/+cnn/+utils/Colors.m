classdef Colors
    % Colors   Provides commonly used colors

    %   Copyright 2020-2023 The MathWorks, Inc.
    
    properties(Constant)
        % Classification
        ClassificationAccuracySmoothedLineColor     = '--mw-graphics-colorOrder-1-primary';    % color1
        ClassificationAccuracyLineColor             = '--mw-graphics-colorOrder-1-tertiary';
        ClassificationValidationAccuracyLineColor   = '--mw-graphics-colorNeutral-line-tertiary';

        ClassificationLossSmoothedLineColor         = '--mw-graphics-colorOrder-2-primary';  % color2
        ClassificationLossLineColor                 = '--mw-graphics-colorOrder-2-tertiary';
        ClassificationValidationLossLineColor       = '--mw-graphics-colorNeutral-line-tertiary';

        % Regression
        RegressionRMSESmoothedLineColor             = '--mw-graphics-colorOrder-3-primary';   % color3
        RegressionRMSELineColor                     = '--mw-graphics-colorOrder-3-tertiary';
        RegressionValidationRMSELineColor           = '--mw-graphics-colorNeutral-line-tertiary';

        RegressionLossSmoothedLineColor             = '--mw-graphics-colorOrder-2-primary';  % color2
        RegressionLossLineColor                     = '--mw-graphics-colorOrder-2-tertiary';
        RegressionValidationLossLineColor           = '--mw-graphics-colorNeutral-line-tertiary';
    end
end
