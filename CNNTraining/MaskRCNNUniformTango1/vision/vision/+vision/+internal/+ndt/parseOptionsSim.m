%vision.internal.ndt.parseOptionsSim Parse NDT options for simulation

% Copyright 2020 The MathWorks, Inc.
function args = parseOptionsSim(args)
arguments
    args.StepSize           double      {vision.internal.ndt.validateStepSize}         = 0.1;
    args.OutlierRatio                   {vision.internal.ndt.validateOutlierRatio}     = 0.55;
    args.MaxIterations                  {vision.internal.ndt.validateMaxIterations}    = 30;
    args.Tolerance                      {vision.internal.ndt.validateTolerance}        = [0.01 0.5];
    args.InitialTransform               {vision.internal.ndt.validateTform}            = [];
    args.Verbose            logical     {vision.internal.ndt.validateLogical}          = false;
end
end
