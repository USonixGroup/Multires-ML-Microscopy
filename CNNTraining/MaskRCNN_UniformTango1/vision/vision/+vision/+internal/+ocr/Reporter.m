classdef(Abstract) Reporter < handle
    % Reporter   Deep neural networks training reporter interface
    %
    % Note: This is a CVT equivalent of DLT's nnet.internal.cnn.util.Reporter.
    
    %   Copyright 2022 The MathWorks, Inc.
    
    methods(Abstract)
        
        % start   Start the reporter.
        start( this )
        
        % reportIteration   Report an iteration.
        reportIteration( this, summary )
        
        % finish   End the reporter.
        finish( this, summary, trainingStopReason )
    end
end
