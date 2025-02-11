function [outData] = visionVariance2DBlockTransform(inData)
% visionVariance2DBlockTransform
%
%   This is an internal function called by Simulink(R) during model load.

% Copyright 2019 The MathWorks, Inc.

[outData] = visionSlopeBiasToBinPtScalingTransform(inData);
end

function [outData] = visionSlopeBiasToBinPtScalingTransform(inData)
% visionSlopeBiasToBinPtScalingTransform flip 'Slope and bias scaling'
% parameter settings to 'Binary point scaling' for new block instances.

InstanceData            = inData.InstanceData;
outData.NewInstanceData = InstanceData;
outData.NewBlockPath    = inData.ForwardingTableEntry.('__slOldName__');

% Find old-style data type "mode" popup parameter names.
% There may be multiple "mode" popup parameter settings
% that apply here, so do not break early from the loop
% (i.e., go through full mask parameter list to hit all)
for i=1:length(InstanceData)
    
    thisMaskParamName = InstanceData(i).Name;
    
    % Set any existing 'Slope and bias scaling' "mode" mask param values
    % to Binary point scaling instead. The serialized WL, FL should
    % already have been handled in the saved param values, so only the
    % "mode" needs adjustment here for correct mask param value support.
    if ( strcmp(thisMaskParamName, 'firstCoeffMode')  || ...
            strcmp(thisMaskParamName, 'secondCoeffMode') || ...
            strcmp(thisMaskParamName, 'thirdCoeffMode')  || ...
            strcmp(thisMaskParamName, 'prodOutputMode')  || ...
            strcmp(thisMaskParamName, 'accumMode')       || ...
            strcmp(thisMaskParamName, 'memoryMode')      || ...
            strcmp(thisMaskParamName, 'outputMode') )
        
        if ( ~isempty(strfind(InstanceData(i).Value, ...
                'Slope and bias scaling')) || ...
                ~isempty(strfind(InstanceData(i).Value, 'User-defined')) )
            
            outData.NewInstanceData(i).Value = 'Binary point scaling';
            
        end
    end
end
end