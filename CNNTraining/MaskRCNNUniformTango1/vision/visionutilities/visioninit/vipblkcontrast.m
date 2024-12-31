function vipblkcontrast

% VIPBLKCONTRAST Mask dynamic dialog function for Contrast Adjustment Block

% Copyright 2018-2020 The MathWorks, Inc.

blk = gcbh;   % Cache handle to block

blkMask = Simulink.Mask.get(gcb);

methodInputRange  = get_param(blk, 'methodInputRange');
isBinNum          = get_param(blk, 'isBinNum');
methodOutputRange = get_param(blk, 'methodOutputRange');
fracSaturate      = blkMask.getParameter('fracSaturate');
rangeIn           = blkMask.getParameter('rangeIn');
isBinNumMaskParam = blkMask.getParameter('isBinNum');
binNum            = blkMask.getParameter('binNum');
rangeOut          = blkMask.getParameter('rangeOut');
binNum.Visible = 'off';
if strcmp(methodInputRange, 'User-defined range')
    rangeIn.Visible           = 'on';
    fracSaturate.Visible      = 'off';
    isBinNumMaskParam.Visible = 'off';
elseif strcmp(methodInputRange,'Full input data range [min max]')
    rangeIn.Visible           = 'off';
    fracSaturate.Visible      = 'off';
    isBinNumMaskParam.Visible = 'off';
else
    rangeIn.Visible           = 'off';
    fracSaturate.Visible      = 'on';
    isBinNumMaskParam.Visible = 'on';
    if strcmp(isBinNum,'on')
        binNum.Visible = 'on';
        binNum.Enabled = 'on';
    else
        binNum.Visible = 'on';
        binNum.Enabled = 'off';
    end
end

if strcmp(methodOutputRange,'User-defined range')
    rangeOut.Visible = 'on';
else
    rangeOut.Visible = 'off';
end

% =====================================================
% Fixpt tab visibility
% =====================================================
prod1Mode      = get_param(blk, 'prod1Mode');
prod2Mode      = get_param(blk, 'prod2Mode');
prod1WordLength      = blkMask.getParameter('prod1WordLength');
prod1FracLength      = blkMask.getParameter('prod1FracLength');
prod2ModeMaskParam   = blkMask.getParameter('prod2Mode');
prod2WordLength      = blkMask.getParameter('prod2WordLength');
prod2FracLength      = blkMask.getParameter('prod2FracLength');
product2Label        = blkMask.getDialogControl('product2Label');
fracLengthLabel      = blkMask.getDialogControl('fracLengthColumnLabel');
prod2SignedText      = blkMask.getDialogControl('prod2SignedText');

fracLengthLabel.Visible        = 'on';
prod2SignedText.Visible        = 'on';
if strcmp(methodInputRange, 'Range determined by saturating outlier pixels')
    product2Label.Visible      = 'on';
    prod2ModeMaskParam.Visible = 'on';
    if strcmp(prod1Mode, 'Binary point scaling')
        prod1WordLength.Visible = 'on';
        prod1FracLength.Visible = 'on';
    else
        prod1WordLength.Visible = 'on';
        prod1FracLength.Visible = 'off';
    end
    if strcmp(prod2Mode, 'Binary point scaling')
        prod2WordLength.Visible = 'on';
        prod2FracLength.Visible = 'on';
    else
        prod2WordLength.Visible = 'on';
        prod2FracLength.Visible = 'off';
    end
else
    product2Label.Visible      = 'off';
    prod2ModeMaskParam.Visible = 'off';
    prod2WordLength.Visible    = 'off';
    prod2FracLength.Visible    = 'off';
    prod2SignedText.Visible    = 'off';
    if strcmp(prod1Mode, 'Binary point scaling')
        prod1WordLength.Visible = 'on';
        prod1FracLength.Visible = 'on';
    else
        prod1WordLength.Visible = 'on';
        prod1FracLength.Visible = 'off';
        fracLengthLabel.Visible = 'off';
    end
end
