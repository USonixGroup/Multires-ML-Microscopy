classdef OutputFcnCallbackReporter < nnet.internal.cnn.util.Reporter
    % OutputFcnCallbackReporter   Reporter to output training info to user-defined callback
    
    %   Copyright 2020-2021 The MathWorks, Inc.

    properties( Constant )
        StandardInfoFieldsMapToSummaryAndStartValue = { ...
            'Epoch',                'Epoch',                0; ...
            'Iteration',            'Iteration',            0; ...
            'TimeSinceStart',       'Time',                 []; ...
           };
    end
    
    properties(Access = private)
        % Callbacks   Functions to call with the info from each iteration
        Callbacks
        
        % Info  Last info value sent to callbacks
        Info
        
        % Info field to summary field map. 
        InfoFieldsMapToSummaryAndStartValue
    end
    
    methods
        function this = OutputFcnCallbackReporter( callback, customInfoFieldsToSummaryMapAndStartValue )
            iAssertIsValidCallback(callback);
            if ~iscell(callback)
                this.Callbacks = { callback };
            else
                this.Callbacks = callback;
            end
            
            this.InfoFieldsMapToSummaryAndStartValue = [
                this.StandardInfoFieldsMapToSummaryAndStartValue
                customInfoFieldsToSummaryMapAndStartValue
                ];
            
            % Initialize Info struct with default values
            structArgs = this.InfoFieldsMapToSummaryAndStartValue(:,[1 3])';
            this.Info = struct( structArgs{:} );
        end
        
        function setup( ~, ~ ) 
        end
        
        function start( this )
            % Add state to info then call callbacks
            this.Info.State = "start";            
            this.callCallbacks();
        end
        
        function reportIteration( this, summary, ~ )
            % Edit the Info struct with fields from summary input, and call
            % callbacks
            updateInfo( this, summary, "iteration" );
            this.callCallbacks();
        end
        
        function reportEpoch( ~, ~, ~, ~ )
            % End of epoch does not trigger user callback
        end
        
        function finish( this, summary, ~ )
            % Incorporate any changes to summary into current info and call
            % callbacks
            updateInfo( this, summary, "done" );
            this.callCallbacks();
        end
        
        function computeFinalIteration(~, ~, ~)
            % computing final validation does not trigger user callback
        end
        
        function reportFinalIteration(~, ~)
            % reporting final validation does not trigger user callback
        end
    end
    
    methods (Access = private)
        function updateInfo( this, summary, state )
            fields = this.InfoFieldsMapToSummaryAndStartValue(:,1);
            for ii = 1:numel(fields)
                fieldToRecord = fields{ii};
                fieldOfSummary = this.InfoFieldsMapToSummaryAndStartValue{ii, 2};
                this.Info.(fieldToRecord) = iGatherAndConvert( summary.(fieldOfSummary) );
            end
            this.Info.State = state;
        end
        
        function callCallbacks( this )
            % Output to callbacks, retrieving stop output if implemented.
            % Don't call the callbacks directly, call via a wrapper which
            % ensures there is always an output.
            stop = cellfun( @(f) iCallbackWrapper(f, this.Info), this.Callbacks );
            if any( stop )
                stopReason = nnet.internal.cnn.util.TrainingStopReason.OutputFcn;
                evtData = nnet.internal.cnn.util.TrainingInterruptEventData( stopReason );
            
                notify( this, 'TrainingInterruptEvent', evtData);
            end
        end
    end
end

function iAssertIsValidCallback(callback, ~)
if iscell(callback) && nargin == 1 % Prevents nested cell arrays
    nested = true;
    cellfun(@(c)iAssertIsValidCallback(c, nested), callback);
else
    assert( isa( callback, 'function_handle' ) && nargin(callback) ~= 0 );
end
end

function tf = iCallbackWrapper(f, info)
% Converts any function that takes at least one argument to one that takes
% one and returns one logical output. Also wraps errors with CNNException.
try
    f(info);
    % If function returned a valid output, return it, otherwise return false
    if exist('ans', 'var') == 1 && iIsConvertibleToLogicalScalar(ans) %#ok<NOANS>
        tf = logical(ans); %#ok<NOANS>
    else
        tf = false;
    end
catch me
    err = nnet.internal.cnn.util.CNNException.hBuildUserException( me );
    throw(err);
end
end

function tf = iIsConvertibleToLogicalScalar(x)
tf = isscalar(x) && (isnumeric(x) || islogical(x));
end

function val = iGatherAndConvert(val)
% Gather if gpuArray and convert to double if numeric
if isnumeric(val)
    val = double( gather( val ) );
end
end
