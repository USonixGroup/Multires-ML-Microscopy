function vipblk2difft

% VIPBLK2DIFFT is a dynamic block update function of 2-D IFFT block

% Copyright 2018 The MathWorks, Inc.

blk = gcbh;

blkMask = Simulink.Mask.get(gcb);
FFTImp = get_param(blk, 'FFTImplementation');

BitRevOrder      = blkMask.getParameter('BitRevOrder');
outputMin      = blkMask.getParameter('outputMin');
outputMax      = blkMask.getParameter('outputMax');
noDataTypesCase    = blkMask.getDialogControl('noDataTypes');
fixedptOpParams    = blkMask.getDialogControl('fixedptOpParams');
fixedptDataTypes   = blkMask.getDialogControl('fixedptDataTypes');
floatingPtTrumpRule= blkMask.getDialogControl('floatingPtTrumpRule');
inAddition         = blkMask.getDialogControl('inAddition');

if strcmp(FFTImp,'FFTW')
    BitRevOrder.Visible         = 'off';
    noDataTypesCase.Visible     = 'on';
    fixedptOpParams.Visible     = 'off';
    fixedptDataTypes.Visible    = 'off';
    floatingPtTrumpRule.Visible = 'off';
    inAddition.Visible          = 'off';
    outputMin.Visible             = 'off';
    outputMax.Visible             = 'off';
else
    BitRevOrder.Visible         = 'on';
    noDataTypesCase.Visible     = 'off';
    fixedptOpParams.Visible     = 'on';
    fixedptDataTypes.Visible    = 'on';
    floatingPtTrumpRule.Visible = 'on';
    inAddition.Visible          = 'on';
    outputMin.Visible             = 'on';
    outputMax.Visible             = 'on';
end
