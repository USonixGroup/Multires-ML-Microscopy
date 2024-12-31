function vipblkdeinterlacing(action)
%VIPBLKDEINTERLACING Summary of this function goes here
%   Detailed explanation goes here

 % Copyright 1995-2019 The MathWorks, Inc.
    if nargin==0, action = 'dynamic'; end
    blk = gcbh;   % Cache handle to block

    switch action
        case 'dynamic'
             dynamicBlockUpdate(blk);
        otherwise
             dynamicBlockUpdate(blk);

    end
end

function dynamicBlockUpdate(blk)
    oldVisibilities = get_param(blk, 'MaskVisibilities');
    newVisibilities = oldVisibilities;
    maskNames    = get_param(blk,'MaskNames');
    blkMask = Simulink.Mask.get(gcb);
    
    methodValue = get_param(blk, 'method');

    
    % =====================================================
    % Fixpt tab visibility
    % =====================================================

    
    accumMode = get_param(blk, 'accumMode');
    outputMode = get_param(blk, 'outputMode');
    prodOutputMode = get_param(blk, 'prodOutputMode');

    accumWLengthIdx = find(strcmp(maskNames(:),'accumWordLength'));
    accumFLengthIdx = find(strcmp(maskNames(:),'accumFracLength'));
    
    outputWLengthIdx = find(strcmp(maskNames(:),'outputWordLength'));
    outputFLengthIdx = find(strcmp(maskNames(:),'outputFracLength'));
    prodOutputWLengthIdx = find(strcmp(maskNames(:),'prodOutputWordLength'));
    prodOutputFLengthIdx = find(strcmp(maskNames(:),'prodOutputFracLength'));
    accumSignedText = blkMask.getDialogControl('accumSignedText');
    outputSignedText = blkMask.getDialogControl('outputSignedText');
    prodOutputSignedText = blkMask.getDialogControl('prodOutputSignedText');
    SignedColumnLabel = blkMask.getDialogControl('SignedColumnLabel');
    WordLengthColumnLabel =  blkMask.getDialogControl('WordLengthColumnLabel');
    FracLengthColumnLabel =  blkMask.getDialogControl('FractionLengthColumnLabel');
    
    prodOutputModeIdx = find(strcmp(maskNames(:),'prodOutputMode'));
    prodOutputLabel = blkMask.getDialogControl('prodOutputLabel');
    
    
    signedColumnLabelVisibility = 'off';
    wordLengthColumnLabelVisibility = 'off';
    fracLengthColumnLabelVisibility = 'off';
    
    prodOutputLabel.Visible = 'off';
    newVisibilities{prodOutputModeIdx} = 'off';
    prodOutputSignedText.Visible = 'off';
    newVisibilities{prodOutputWLengthIdx} = 'off';
    newVisibilities{prodOutputFLengthIdx} = 'off';
    

    if strcmp(accumMode, 'Binary point scaling')
        accumSignedText.Visible = 'on';
        newVisibilities{accumWLengthIdx} = 'on';
        newVisibilities{accumFLengthIdx} = 'on';
        signedColumnLabelVisibility = 'on';
        wordLengthColumnLabelVisibility = 'on';
        fracLengthColumnLabelVisibility = 'on';
    else
        accumSignedText.Visible = 'off';
        newVisibilities{accumWLengthIdx} = 'off';
        newVisibilities{accumFLengthIdx} = 'off';
    end

    if strcmp(outputMode, 'Binary point scaling')
        outputSignedText.Visible = 'on';
        newVisibilities{outputWLengthIdx} = 'on';
        newVisibilities{outputFLengthIdx} = 'on';
        signedColumnLabelVisibility = 'on';
        wordLengthColumnLabelVisibility = 'on';
        fracLengthColumnLabelVisibility = 'on';   
    else
        outputSignedText.Visible = 'off';
        newVisibilities{outputWLengthIdx} = 'off';
        newVisibilities{outputFLengthIdx} = 'off';
    end

    SignedColumnLabel.Visible = signedColumnLabelVisibility;
    WordLengthColumnLabel.Visible =  wordLengthColumnLabelVisibility;
    FracLengthColumnLabel.Visible =  fracLengthColumnLabelVisibility;
    
  
    emptyFixptLabel = blkMask.getDialogControl('emptyFixpt');
    FixPtOpParamsGroupBox = blkMask.getDialogControl('FixPtOpParamsGroupBox');
    FixptDatatypesGroupBox = blkMask.getDialogControl('FixptDatatypesGroupBox');
    FloatingPointTrumpRuleText = blkMask.getDialogControl('FloatingPointTrumpRule');
    
    if(strncmp(methodValue,'Linear ...',6))
        emptyFixptLabel.Visible = 'off';
        FixPtOpParamsGroupBox.Visible = 'on';
        FixptDatatypesGroupBox.Visible = 'on';
        FloatingPointTrumpRuleText.Visible = 'on';
    else
        emptyFixptLabel.Visible = 'on';
        FixPtOpParamsGroupBox.Visible = 'off';
        FixptDatatypesGroupBox.Visible = 'off';
        FloatingPointTrumpRuleText.Visible = 'off';
    end

    if ~isequal(newVisibilities,oldVisibilities)
        set_param(blk, 'MaskVisibilities', newVisibilities);
    end
end