classdef WarningLogger < handle
    % Utility class to log and issue warnings. Used to reissue warnings at
    % the end of training when the verbose log may have made it hard to see
    % the original warning.
    %
    % In training functions call
    % vision.internal.cnn.WarningLogger.initialize(). Then any warnings
    % should be issued using vision.internal.cnn.WarningLogger.warning().
    %
    % NB: warnings are displayed without backtrace to avoid polluting the
    % verbose log.
    
    % Copyright 2018 The MathWorks, Inc. 
    properties
        % Log holds message objects.
        Log = {};
    end
    
    methods(Static)

        %------------------------------------------------------------------
        function clearLog()
            w = vision.internal.cnn.WarningLogger.getSingleton();
            w.Log = {};
        end
        
        %------------------------------------------------------------------
        function reissueAllWarnings()
            w = vision.internal.cnn.WarningLogger.getSingleton();
            for i = 1:numel(w.Log)
                iWarnWithoutBacktrace(w.Log{i});
                fprintf('\n');
            end
        end
        
        %------------------------------------------------------------------
        function warning(msg)
            % issue message without backtrace and append to log.
            w = vision.internal.cnn.WarningLogger.getSingleton();
            iWarnWithoutBacktrace(msg)
            w.Log{end+1} = msg;
        end
        
        %------------------------------------------------------------------
        function deferredWarning(msg)
            % log warning instead of issuing it right away.
             w = vision.internal.cnn.WarningLogger.getSingleton();
             w.Log{end+1} = msg;
        end
        
        %------------------------------------------------------------------
        function tf = hasWarnings()
            w = vision.internal.cnn.WarningLogger.getSingleton();
            tf = ~isempty(w.Log);
        end
        
        %------------------------------------------------------------------
        function initialize()
            vision.internal.cnn.WarningLogger.clearLog();
        end
        
        %------------------------------------------------------------------
        function x = getSingleton()
            persistent w
            if isempty(w)
                w = vision.internal.cnn.WarningLogger();
            end
            x = w;
        end
    end
end
   
%--------------------------------------------------------------------------
function iWarnWithoutBacktrace(msg)
warning off backtrace
warning(msg)
warning on backtrace
end