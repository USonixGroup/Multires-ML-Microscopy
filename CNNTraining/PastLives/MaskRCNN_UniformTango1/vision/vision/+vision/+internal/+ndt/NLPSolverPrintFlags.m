classdef NLPSolverPrintFlags < int32
    %This class is for internal use only. It may be removed in the future.
    
    %NLPSOLVERPRINTFLAGS
    
    %   Copyright 2017 The MathWorks, Inc.
    
    enumeration
        IterationStart(1)
        CurrentFcn(2)
        CurrentVar(3)
        IterationExit(4)
    end
end