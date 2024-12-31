classdef NLPSolverExitFlags < int32
    %This class is for internal use only. It may be removed in the future.
    
    %NLPSOLVEREXITFLAGS
    
    %   Copyright 2017 The MathWorks, Inc.
    
    enumeration
        Success(0);
        LocalMinimumFound(1)
        StepSizeBelowMinimum(2)
        ChangeInErrorBelowMinimum(3)
        SolutionCheckFailed(4)
        MaximumIterationReached(5)
    end
end