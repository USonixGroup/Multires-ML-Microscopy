classdef NLPSolverLineSearch < vision.internal.EnforceScalarHandle
    %This class is for internal use only. It may be removed in the future.

    %NLPSolverLineSearch Solver for unconstrained nonlinear problem. The
    %objective is to maximize a function f(x) w.r.t x
    %
    %   References:
    %
    %   J. Nocedal and S. Wright, Numerical Optimization (Second Ed.)
    %   Springer, New York, 2006

    %   Copyright 2017-2024 The MathWorks, Inc.
    %#codegen

    properties (Access = protected)

        %GradientTolerance The solver terminates if the infinite norm of
        %the gradient f'(x) falls below this value (a positive value)
        GradientTolerance

        %FunctionTolerance The solver terminates if the change in the
        %scalar cost function f(x) within a step falls below this value (a
        %positive value)
        FunctionTolerance

        %StepTolerance The solver terminates if the change in the
        %norm of input variable x within a step falls below this value (a
        %positive value)
        StepTolerance

        %MaxIterations The solver terminates if max number of iterations
        %has been reached (a positive integer)
        MaxIterations

        %MaxStepSize The max step size along the descent direction (a
        %positive value)
        MaxStepSize

        %CostFcn Function handle to compute cost. The function must return
        %the cost(score), gradient vector and hessian matrix. The first
        %input must be the variable of the objective function.
        CostFcn

        %ExtraArgs Additional input arguments for the cost function
        ExtraArgs

        %SolutionEvaluationFcn Function handle to evaluate found solution.
        %This is an optional stopping criterion, usually a customized
        %function by the user. The first two inputs must be the previous
        %estimates of variable x, and the current new estimates.
        SolutionEvaluationFcn

        %ExtraSolutionEvalArgs Additional input arguments for the SolutionEvaluationFcn
        ExtraSolutionEvalArgs

        %PrintFcn print out internal message if this function is provided
        PrintFcn

        %CheckInitializationFcn Function handle to check initialization.
        %   Function handle that accepts score, gradient and Hessian. Used
        %   to check that score, gradient and/or Hessian are valid at
        %   initialization.
        CheckInitializationFcn

        %Verbose
        Verbose
    end
    methods
        function this = NLPSolverLineSearch(costFcn, evaluateFcn, checkGradFcn, varargin)
            %NLPSolverLineSearch Constructor
            this.MaxIterations = 100;
            this.MaxStepSize = 1000;
            this.GradientTolerance = 1e-6;
            this.FunctionTolerance = 1e-6;
            this.StepTolerance = 1e-6;

            validateattributes(costFcn,{'function_handle'},{'scalar'});
            if isSimMode
                this.CostFcn = costFcn;
            else
                % Codegen supports NDT's cost function only , not any other cost
                %function given by function handle
                if ~isGPUCodegen
                    if coder.internal.preferMATLABHostCompiledLibraries()
                        this.CostFcn = @vision.internal.buildable.ndtComputeScoreDerivativesBuildable ...
                            .ndtComputeScoreDerivatives;
                    else
                        this.CostFcn = @vision.internal.codegen.pc.ndtCostFunctionCPU;
                    end
                else
                    this.CostFcn = @vision.internal.codegen.gpu.pcregisterndt.ndtCostFunctionGPUImpl;
                end
            end
            if ~isempty(varargin)
                this.ExtraArgs = varargin{1};
            end
            validateattributes(evaluateFcn,{'function_handle'},{'scalar'});
            this.SolutionEvaluationFcn = evaluateFcn;
            this.ExtraSolutionEvalArgs  = [0,0];
            validateattributes(checkGradFcn,{'function_handle'},{'scalar'});
            this.CheckInitializationFcn = checkGradFcn;
            this.PrintFcn = @printMessage;
            this.Verbose = false;
        end

        function params = getSolverParams(this)
            params.MaxIterations = this.MaxIterations;
            params.GradientTolerance        = this.GradientTolerance;
            params.MaxStepSize              = this.MaxStepSize;
            params.FunctionTolerance        = this.FunctionTolerance;
            params.SolutionEvaluationFcn    = this.SolutionEvaluationFcn;
            params.ExtraSolutionEvalArgs    = this.ExtraSolutionEvalArgs;
            params.StepTolerance            = this.StepTolerance;
            params.PrintFcn                 = this.PrintFcn;
            params.Verbose                  = this.Verbose;
            params.CheckInitializationFcn   = this.CheckInitializationFcn;
        end

        function setSolverParams(this, params)
            this.MaxIterations          = params.MaxIterations;
            this.GradientTolerance      = params.GradientTolerance;
            this.StepTolerance          = params.StepTolerance;
            this.FunctionTolerance      = params.FunctionTolerance;
            this.MaxStepSize            = params.MaxStepSize;
            this.SolutionEvaluationFcn  = params.SolutionEvaluationFcn;
            this.ExtraSolutionEvalArgs  = params.ExtraSolutionEvalArgs;
            this.PrintFcn               = params.PrintFcn;
            this.Verbose                = params.Verbose;
            this.CheckInitializationFcn = params.CheckInitializationFcn;
        end
    end

    methods
        %==================================================================
        % Main function to run line search algorithm. The objective
        % function is unconstrained, and the user must provide a way to
        % compute the cost/score, gradient and hessian. For now, an initial
        % value of variable x, x0, must be provided to the solver.
        %==================================================================
        function [xSol, solutionInfo] = solve(this, x0)

            x = x0;

            % Initialize score, gradient and hessian (or hessian approximate)
            [score, grad, Hessian] = this.CostFcn(x, this.ExtraArgs);

            if ~isempty(this.CheckInitializationFcn)
                this.CheckInitializationFcn(score, grad, Hessian);
            end

            % Disable the warnings about conditioning for singular and
            % nearly singular matrices
            if isSimMode
                warningstate1 = warning('off','MATLAB:nearlySingularMatrix');
                warningstate2 = warning('off','MATLAB:singularMatrix');
                warningstate3 = warning('off','MATLAB:rankDeficientMatrix');
            end

            % Main iteration loop
            maxScore = score;
            bestX = x;
            bestIter = 0;
            exitFlag = vision.internal.ndt.NLPSolverExitFlags.Success;

            coder.gpu.nokernel;
            for iter = 1:this.MaxIterations
                if isSimMode && this.Verbose
                    this.PrintFcn(vision.internal.ndt.NLPSolverPrintFlags.IterationStart, iter);
                end
                % Solve for H*dx = -g
                dx = Hessian \ (-grad);

                % Normalized descent direction
                if ~isGPUCodegen
                    xnorm = sqrt(dx' * dx);
                else
                    dxsqSum = dx.*dx;
                    if length(dx) <= 32
                        % For small sizes we can use single-threaded CUDA
                        % kernel to sum the values on GPU.
                        dxsqSumVal = 0;
                        coder.gpu.kernel([32,1,1],[1,1,1])
                        for k = 1:1 % Dummy kernel to keep data on GPU
                            for i = 1:length(dx)
                                dxsqSumVal = dxsqSumVal + dxsqSum(i);
                            end
                        end
                    else
                        dxsqSumVal = gpucoder.reduce(dxsqSum,@funcSum);
                    end
                    xnorm = sqrt(dxsqSumVal(1));
                end
                dx = dx / xnorm;

                % Set the value of phi and phi' in equation-1.3[More, Thuente 1994]
                phi_0 = -score;
                dphi_0 = -grad'*dx;

                [xTrial, scoreTrial, gradTrial, HessianTrial] = lineSearch(this, x, dx, ...
                    phi_0, dphi_0, this.GradientTolerance, this.MaxStepSize, xnorm);

                % EXIT condition: minimum change in function value
                if changeInScoreBelowMinimum(this, score, scoreTrial)
                    exitFlag = vision.internal.ndt.NLPSolverExitFlags.ChangeInErrorBelowMinimum;
                    % EXIT condition: local minimum found
                elseif atLocalMinimum(this, gradTrial)
                    exitFlag = vision.internal.ndt.NLPSolverExitFlags.LocalMinimumFound;
                    % EXIT condition: minimum step size
                elseif stepSizeBelowMinimum(this, x, xTrial)
                    exitFlag = vision.internal.ndt.NLPSolverExitFlags.StepSizeBelowMinimum;
                    % EXIT condition: check solution by user function
                elseif ~isempty(this.SolutionEvaluationFcn)
                    if this.SolutionEvaluationFcn(x, xTrial, this.ExtraSolutionEvalArgs)
                        exitFlag = vision.internal.ndt.NLPSolverExitFlags.SolutionCheckFailed;
                    end
                end
                if isSimMode &&  this.Verbose
                    % Display results at each iteration
                    this.PrintFcn(vision.internal.ndt.NLPSolverPrintFlags.CurrentFcn, scoreTrial);
                    args.x = xTrial;
                    args.grad = gradTrial;
                    this.PrintFcn(vision.internal.ndt.NLPSolverPrintFlags.CurrentVar, args);
                end
                if int32(exitFlag)
                    if maxScore < scoreTrial
                        bestX = xTrial;
                        maxScore = scoreTrial;
                    end
                    if maxScore < score
                        bestX = x;
                        maxScore = score;
                    end
                    break;
                else
                    x = xTrial;
                    score = scoreTrial;
                    grad = gradTrial;
                    Hessian = HessianTrial;
                    % Record the best solution so far
                    if maxScore < score
                        bestX = x;
                        maxScore = score;
                        bestIter = iter;
                    end
                end
            end
            if isSimMode
                % Restore the warning states to their original settings
                warning(warningstate1)
                warning(warningstate2)
                warning(warningstate3)
            end
            % EXIT condition: maximum iteration reached
            if (~int32(exitFlag))
                exitFlag = vision.internal.ndt.NLPSolverExitFlags.MaximumIterationReached;
            end

            xSol = bestX;
            solutionInfo.Iterations = bestIter;
            solutionInfo.Objective = maxScore;
            solutionInfo.ExitFlag = exitFlag;
            if isSimMode && this.Verbose
                this.PrintFcn(vision.internal.ndt.NLPSolverPrintFlags.IterationExit, exitFlag);
            end
        end

        function [x_t, score, grad, Hessian] = lineSearch(this, x0, dx, ...
                phi_0, dphi_0, a_min, a_max, a_init)
            % lineSearch compute the line search algorithm with guaranteed
            % sufficient decrease.
            %
            % [1] More and Thuente, Line Search Algorithm with Guaranteed
            % Sufficient Decrease, ACM Trans. on Mathematical Software, Vol 20,
            % No. 3, 1994
            if ~isSimMode
                if ~isGPUCodegen
                    if coder.internal.preferMATLABHostCompiledLibraries()
                        this.CostFcn = @vision.internal.buildable.ndtComputeScoreDerivativesBuildable ...
                            .ndtComputeScoreDerivatives;
                    else
                        this.CostFcn = @vision.internal.codegen.pc.ndtCostFunctionCPU;
                    end
                else
                    this.CostFcn = @vision.internal.codegen.gpu.pcregisterndt.ndtCostFunctionGPUImpl;
                end
            end

            % Check the decent direction.
            if dphi_0 >= 0
                % Not a decent direction.
                if dphi_0 == 0
                    x_t = x0 + dx * this.MaxStepSize;
                    [score, grad, Hessian] = this.CostFcn(x_t, this.ExtraArgs);
                    return
                else
                    % Reverse step direction and calculate optimal step.
                    dphi_0 = -dphi_0;
                    dx = -dx;
                end
            end

            % Sufficient decreace constant, Equation 1.1 [More, Thuete 1994]
            mu = 0.0001;
            % Curvature condition constant, Equation 1.2 [More, Thuete 1994]
            nu = 0.9;
            % Initial endpoints of Interval I
            a_l = 0;
            a_u = 0;

            f_l = 0;
            g_l = dphi_0 - dphi_0 * mu;

            f_u = 0;
            g_u = g_l;

            % Check used to allow More-Thuente step length calculation to
            % be skipped
            interval_converged = (a_max -a_min) < 0.0001;
            open_interval = true;

            a_t = a_init;
            a_t = min(a_t, a_max);
            a_t = max(a_t, a_min);

            x_t = x0 + dx * a_t;

            % Updates score, gradient and hessian.
            [score, grad, Hessian] = this.CostFcn(x_t, this.ExtraArgs);

            % Calculate phi(alpha_t) and phi'(alpha_t)
            phi_t = -score;
            dphi_t = -grad' * dx;

            % Calculate psi(alpha_t) and psi'(alpha_t)
            psi_t =  phi_t - phi_0 - a_t * dphi_0 * mu;
            dpsi_t = dphi_t - dphi_0 * mu;

            % The search algorithm for T(mu)
            maxIterations = 10;
            iter = 0;

            % Iterate until max number of iterations, interval convergence
            % or a value satisfies the sufficient decrease, Equation 1.1,
            % and curvature condition, Equation 1.2 [More, Thuente 1994]
            coder.gpu.nokernel;
            while (~interval_converged && iter < maxIterations && ...
                    ~(psi_t <= 0 && dphi_t <= -nu * dphi_0))
                % Use auxilary function if interval I is not closed
                if open_interval
                    a_t = vision.internal.ndt.selectTrialValue(a_l, f_l, g_l, ...
                        a_u, f_u, g_u, a_t, psi_t, dpsi_t);
                else
                    a_t = vision.internal.ndt.selectTrialValue(a_l, f_l, g_l, ...
                        a_u, f_u, g_u, a_t, phi_t, dphi_t);
                end

                a_t = min(a_t, a_max);
                a_t = max(a_t, a_min);

                x_t = x0 + dx * a_t;

                [score, grad] = this.CostFcn(x_t, this.ExtraArgs);

                % Calculate phi(alpha_t) and phi'(alpha_t)
                phi_t = -score;
                dphi_t = -grad' * dx;

                % Calculate psi(alpha_t) and psi'(alpha_t)
                psi_t = phi_t - phi_0 - a_t * dphi_0 * mu;
                dpsi_t = dphi_t - dphi_0 * mu;

                % Check if I is now a closed interval
                if open_interval && (psi_t <= 0 && dpsi_t >= 0)
                    open_interval = false;

                    % Converts f_l and g_l from psi to phi
                    f_l = f_l + phi_0 - mu * dphi_0 * a_l;
                    g_l = g_l + mu * dphi_0;

                    % Converts f_u and g_u from psi to phi
                    f_u = f_u + phi_0 - mu * dphi_0 * a_u;
                    g_u = g_u + mu * dphi_0;
                end

                if open_interval
                    % Update interval end points using Updating Algorithm
                    % [More, Thuente 1994]
                    [a_l, f_l, g_l, a_u, f_u, g_u, interval_converged] = ...
                        vision.internal.ndt.updateInterval(a_l, f_l, g_l, a_u, f_u, g_u, a_t, psi_t, dpsi_t);
                else
                    % Update interval end points using Modified Updating
                    % Algorithm [More, Thuente 1994]
                    [a_l, f_l, g_l, a_u, f_u, g_u, interval_converged] = ...
                        vision.internal.ndt.updateInterval(a_l, f_l, g_l, a_u, f_u, g_u, a_t, phi_t, dphi_t);
                end

                iter = iter + 1;
            end

            % If inner loop was run then hessian needs to be calculated.
            % Hessian is unnecessary for step length determination but
            % gradients are required so derivative and transform data is
            % stored for the next iteration.
            if (iter)
                if isSimMode
                    [score, grad, Hessian] = this.CostFcn(x_t, this.ExtraArgs);
                else
                    if ~isGPUCodegen
                        if coder.internal.preferMATLABHostCompiledLibraries()
                            [score, grad, Hessian] = vision.internal.buildable.ndtComputeScoreDerivativesBuildable....
                            .ndtComputeScoreDerivatives(x_t, this.ExtraArgs);
                        else
                            [score, grad, Hessian] = vision.internal.codegen.pc.ndtCostFunctionCPU(x_t, this.ExtraArgs);
                        end
                    else
                        [score, grad, Hessian] = vision.internal.codegen.gpu.pcregisterndt.ndtCostFunctionGPUImpl(x_t, this.ExtraArgs);
                    end
                end
            end
        end

        function flag = atLocalMinimum(this, grad)
            flag = max(abs(grad)) < this.GradientTolerance;
        end

        function flag = changeInScoreBelowMinimum(this, score, scoreTrial)
            flag = abs(score - scoreTrial) < this.FunctionTolerance;
        end

        function flag = stepSizeBelowMinimum(this, x, xTrial)
            d = x - xTrial;
            flag = d' * d < this.StepTolerance^2;
        end
    end

end

function printMessage(msg, arg)
persistent nlpPrinter;
if isempty(nlpPrinter)
    nlpPrinter = vision.internal.MessagePrinter.configure(true);
end

if msg == vision.internal.ndt.NLPSolverPrintFlags.IterationStart
    nlpPrinter.linebreak;
    nlpPrinter.printMessage('vision:pointcloud:nlpIteration',arg);

elseif msg == vision.internal.ndt.NLPSolverPrintFlags.CurrentFcn
    nlpPrinter.printMessage('vision:pointcloud:nlpCurrentFcn',num2str(arg));

elseif msg == vision.internal.ndt.NLPSolverPrintFlags.CurrentVar
    % Only display infinite norm of gradient to check first-order
    % optimality
    nlpPrinter.printMessage('vision:pointcloud:nlpCurrentVar',num2str(max(abs(arg.grad))));

elseif msg == vision.internal.ndt.NLPSolverPrintFlags.IterationExit
    nlpPrinter.linebreak;
    nlpPrinter.print('--------------------------------------------\n');
    switch arg
        case vision.internal.ndt.NLPSolverExitFlags.ChangeInErrorBelowMinimum
            nlpPrinter.printMessage('vision:pointcloud:nlpStopCondSmallAbsFunVal');
        case vision.internal.ndt.NLPSolverExitFlags.LocalMinimumFound
            nlpPrinter.printMessage('vision:pointcloud:nlpStopCondSmallGrad');
        case vision.internal.ndt.NLPSolverExitFlags.StepSizeBelowMinimum
            nlpPrinter.printMessage('vision:pointcloud:nlpStopCondSmallChangeOfX');
        case vision.internal.ndt.NLPSolverExitFlags.SolutionCheckFailed
            nlpPrinter.printMessage('vision:pointcloud:nlpStopCondSolutionCheck');
        case vision.internal.ndt.NLPSolverExitFlags.MaximumIterationReached
            nlpPrinter.printMessage('vision:pointcloud:nlpStopCondMaxIteration');
    end
end
end

function flag = isSimMode()

flag = isempty(coder.target);
end
%--------------------------------------------------------------------------
% GPU Codegen flag
function flag = isGPUCodegen()
    flag = coder.gpu.internal.isGpuEnabled;
end

function c = funcSum(a,b)
    c = a + b;
end
