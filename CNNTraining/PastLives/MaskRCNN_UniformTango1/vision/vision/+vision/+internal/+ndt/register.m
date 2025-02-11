%vision.internal.ndt.register Registration using NDT
%
%   tform = register(movingPoints, fixedVoxelMeans, fixedVoxelICov,
%   fixedSearhTree, initTform, voxelSize, outlierRatio, maxIterations,
%   tolerance, verboseFlag, stepSize)
%
%   Inputs
%   ------
%   movingPoints    - M-by-3 [x,y,z] points from the moving scan
%   fixedVoxelMeans - V-by-3 [x,y,z] voxel means from the fixed scan
%   fixedVoxelICov  - 3-by-3-by-V voxel inverse covariances from fixed scan
%   fixedSearchTree - Kdtree indexed with fixed voxel means
%   initTform       - rigidtform3d initial transformation estimate
%   voxelSize       - Scalar voxel size
%   outlierRatio    - Scalar outlier ratio
%   maxIterations   - Scalar maximum number of iterations
%   tolerance       - 1-by-2 vector of translation and rotation tolerances
%   verboseFlag     - Scalar logical flag to display progress information
%   stepSize        - Scalar step size
%   tformType       - Type of transform to return ('single' or 'double')
%
%   Output
%   ------
%   tform   - rigidtform3d registration transformation
%
%   Notes
%   -----
%   - All numeric inputs are expected to be in double-precison
%
%   See also pcregisterndt, pcmapndt.

% Copyright 2020-2022 The MathWorks, Inc.
%#codegen
function tform = register(movingPoints, fixedVoxelMeans, fixedVoxelICov, fixedSearchTree, ...
    initTform, voxelSize, outlierRatio, maxIterations, tolerance, verboseFlag, stepSize, tformType)

% Convert to Euler angle format
x0 = vision.internal.eul.tform2pose(initTform);

% Initialize gaussian fitting parameters for estimation of likelihood of
% each point w.r.t to a distribution of a voxel that encloses the point. Eq
% 6.8, Martin 2013.
c1 = max(10*(1 - outlierRatio), eps);      % Gaussian-Distribution parameter c1
c2 = max(outlierRatio/voxelSize^3, eps);   % Uniform distribution parameter c2
d3 = -log(c2);
d1 = -log(c1 + c2) - d3;
d2 = -2*log((-log(c1*exp(-0.5) + c2) - d3) / d1);

% Set up the NLP solver
args.ps                     = movingPoints;
args.mvals                  = fixedVoxelMeans;
args.iCov                   = fixedVoxelICov;
args.tree                   = fixedSearchTree;
args.d1                     = d1;
args.d2                     = d2;
args.radius                 = voxelSize;
args.SolutionEvaluationFcn  = @solutionEvaluationNDT;

solver = vision.internal.ndt.NLPSolverLineSearch(...
    @visionNDTComputeScoreDerivatives, @solutionEvaluationNDT,...
    @checkGradientInitializationNDT,args);

params = getSolverParams(solver);
params.MaxIterations            = maxIterations;
params.MaxStepSize              = stepSize;
params.ExtraSolutionEvalArgs    = tolerance;


if isempty(coder.target) && verboseFlag
    params.PrintFcn             = @printMessage;
    params.Verbose              = verboseFlag;
end

setSolverParams(solver, params);

% Search the best pose with line search algorithm
x = solver.solve(x0(:));

% Convert to rigidtform3d format
tform = vision.internal.eul.pose2tform(x.', tformType);
end

%--------------------------------------------------------------------------
function tf = solutionEvaluationNDT(x, xTrial, tolerance)
% Stop the NLP solver if pose change is below tolerance.

% Difference in translation
tdiff = x(1:3)-xTrial(1:3);

% Difference in rotation (radian)
rdiff = x(4:6)-xTrial(4:6);

tf = (tdiff(1)^2 + tdiff(2)^2 + tdiff(3)^2) < tolerance(1)^2 && ...
    (rdiff(1)^2 + rdiff(2)^2 + rdiff(3)^2) < tolerance(2)^2;
end

%--------------------------------------------------------------------------
function checkGradientInitializationNDT(~, gradient, ~)

% Check that gradient is valid at initialization
if all(gradient==0)
    coder.internal.error('vision:pointcloud:ndtNoGradientAtInitialization')
end
end

%--------------------------------------------------------------------------
function printMessage(msg, arg)
% Verbose message printer for NLP solver

persistent printer;
if isempty(printer)
    printer = vision.internal.MessagePrinter.configure(true);
end

if msg == vision.internal.ndt.NLPSolverPrintFlags.IterationStart
    printer.linebreak;
    printer.print('--------------------------------------------\n');
    printer.printMessage('vision:pointcloud:ndtIteration',arg);
    
elseif msg == vision.internal.ndt.NLPSolverPrintFlags.CurrentFcn
    printer.printMessage('vision:pointcloud:ndtCurrentFcn',num2str(arg));
    
elseif msg == vision.internal.ndt.NLPSolverPrintFlags.CurrentVar
    printer.printMessage('vision:pointcloud:ndtCurrentVarTranslation',...
        num2str(arg.x(1)), num2str(arg.x(2)), num2str(arg.x(3)));
    printer.printMessage('vision:pointcloud:ndtCurrentVarRotation', ...
        num2str(rad2deg(arg.x(4))), num2str(rad2deg(arg.x(5))), num2str(rad2deg(arg.x(6))));
    
elseif msg == vision.internal.ndt.NLPSolverPrintFlags.IterationExit
    printer.linebreak;
    printer.print('--------------------------------------------\n');
    switch arg
        case vision.internal.ndt.NLPSolverExitFlags.ChangeInErrorBelowMinimum
            printer.printMessage('vision:pointcloud:nlpStopCondSmallAbsFunVal');
        case vision.internal.ndt.NLPSolverExitFlags.LocalMinimumFound
            printer.printMessage('vision:pointcloud:nlpStopCondSmallGrad');
        case vision.internal.ndt.NLPSolverExitFlags.StepSizeBelowMinimum
            printer.printMessage('vision:pointcloud:nlpStopCondSmallChangeOfX');
        case vision.internal.ndt.NLPSolverExitFlags.SolutionCheckFailed
            printer.printMessage('vision:pointcloud:ndtStopCondSolutionCheck');
        case vision.internal.ndt.NLPSolverExitFlags.MaximumIterationReached
            printer.printMessage('vision:pointcloud:nlpStopCondMaxIteration');
    end
end
end