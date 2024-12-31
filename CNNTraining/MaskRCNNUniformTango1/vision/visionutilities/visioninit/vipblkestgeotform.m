function vipblkestgeotform
%

%   Copyright 2017-2021 The MathWorks, Inc.

    blk = gcbh;   % Cache handle to block
    dynamicBlockUpdate(blk)
end

function dynamicBlockUpdate(blk)

blkMask = Simulink.Mask.get(blk);

isRandomSamplingValue = get_param(blk, 'isRandomSampling');
tformTypeValue        = get_param(blk, 'tformType');


algMethodValueMaskParam  = blkMask.getParameter('algMethod');
isRefineTFormParam       = blkMask.getParameter('isRefineTForm');
isOutputInlierParam      = blkMask.getParameter('isOutputInlier');
numMethodParam           = blkMask.getParameter('numMethod');
isStopSamplingParam      = blkMask.getParameter('isStopSampling');
distanceAlgParam         = blkMask.getParameter('distanceAlg');
distanceGeoParam         = blkMask.getParameter('distanceGeo');
sampleNumParam           = blkMask.getParameter('sampleNum');
maxSampleNumParam        = blkMask.getParameter('maxSampleNum');
desiredConfidenceParam   = blkMask.getParameter('desiredConfidence');
percentStopParam         = blkMask.getParameter('percentStop');
numMethodValueMaskParam  = blkMask.getParameter('numMethod');
isStopSamplingValueMaskParam = blkMask.getParameter('isStopSampling');

if strcmp(isRandomSamplingValue, 'on')
    algMethodValueMaskParam.Visible      = 'on';
    isRefineTFormParam.Visible  = 'on';
    isOutputInlierParam.Visible = 'on';
    
    if strcmp(algMethodValueMaskParam.Value, 'Random Sample Consensus (RANSAC)')
        numMethodParam.Visible      = 'on';
        isStopSamplingParam.Visible = 'on';

        if strcmp(tformTypeValue, 'Projective')
            distanceAlgParam.Visible = 'on';
            distanceGeoParam.Visible = 'off';
        else
            distanceGeoParam.Visible = 'on';
            distanceAlgParam.Visible = 'off';
        end

        if strcmp(numMethodValueMaskParam.Value, 'Specified value')
            sampleNumParam.Visible         = 'on';
            maxSampleNumParam.Visible      = 'off';
            desiredConfidenceParam.Visible = 'off';            
        else
            maxSampleNumParam.Visible      = 'on';
            desiredConfidenceParam.Visible = 'on';
            sampleNumParam.Visible         = 'off';
        end

        if strcmp(isStopSamplingValueMaskParam.Value, 'on')
            percentStopParam.Visible   = 'on';
        else
            percentStopParam.Visible   = 'off';
        end
        
    else
        numMethodParam.Visible         = 'off';
        isStopSamplingParam.Visible    = 'off';
        distanceAlgParam.Visible       = 'off';
        distanceGeoParam.Visible       = 'off';
        maxSampleNumParam.Visible      = 'off';
        desiredConfidenceParam.Visible = 'off';
        sampleNumParam.Visible         = 'on';
    end
else
    algMethodValueMaskParam.Visible         = 'off';
    isRefineTFormParam.Visible     = 'off';
    isOutputInlierParam.Visible    = 'off';
    numMethodParam.Visible         = 'off';
    isStopSamplingParam.Visible    = 'off';
    distanceAlgParam.Visible       = 'off';
    distanceGeoParam.Visible       = 'off';
    sampleNumParam.Visible         = 'off';
    desiredConfidenceParam.Visible = 'off';
    percentStopParam.Visible       = 'off';
    maxSampleNumParam.Visible = 'off';
end

end
