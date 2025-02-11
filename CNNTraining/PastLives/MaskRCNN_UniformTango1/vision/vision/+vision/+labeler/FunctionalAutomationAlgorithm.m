classdef FunctionalAutomationAlgorithm < ...
        vision.labeler.AutomationAlgorithm
    % This automation algorithm class provides support for running
    % user-defined functions during automation.

    % Copyright 2022 The MathWorks, Inc.
   
    properties(Constant)
        
        Name = vision.getMessage('vision:labeler:FunctionalAlgName');

        Description = vision.getMessage('vision:labeler:FunctionalAlgDescription');

        UserDirections = {...
            vision.getMessage('vision:labeler:FunctionalAlgInstruction1'), ...
            vision.getMessage('vision:labeler:FunctionalAlgInstruction2'), ...
            vision.getMessage('vision:labeler:AcceptInstruction')};
    end
   
    properties(Access = private)
        % FunctionHandle The function handle to execute during automation.
        FunctionHandle

        % FunctionString 
        FunctionString = "";
    end

    %----------------------------------------------------------------------
    methods
        
        function isValid = checkLabelDefinition(~, ~)
            % The type of labels an automation function can support is not
            % known a priori. Therefore, we always return true and
            % check the automation algorithm output after it is run to
            % verify it is consistent with the current label definitions.
            isValid = true;
        end

        function isReady = checkSetup(this)
            % Return true if the user has specified a function. Otherwise,
            % the app will throw a message if the user has not specified a
            % function via the settings dialog.
            isReady = isFunctionHandleValid(this);
        end

        function settingsDialog(this)
            dlg = vision.internal.labeler.CustomAutomationFunctionSettingsDialog(this.FunctionString);
            createDialog(dlg);

            % Cache the string value entered by the user and the resulting
            % function handle.
            this.FunctionString = dlg.FunctionString;
            this.FunctionHandle = dlg.FunctionHandle;
        end
    end

    %----------------------------------------------------------------------
    methods

        %------------------------------------------------------------------
        function autoLabels = run(this, I, info)
            % Run automation function. Errors are caught and rethrown to be
            % displayed by the labeler app. The output of the function is
            % validated by the app.
            try
                if(nargin(this.FunctionHandle) == 2)
                    autoLabels = this.FunctionHandle(I, info);
                else
                    autoLabels = this.FunctionHandle(I);
                end

            catch causeOfError
          
                % Decorate the actual error with a custom message.
                msg = message('vision:labeler:FunctionalAlgUserFunctionError',...
                    causeOfError.message);
                exception = MException(msg.Identifier, getString(msg));
                throw(exception)
            end
        end

        %------------------------------------------------------------------
        function initialize(~, ~)
            % Nothing required.
        end
      
        %------------------------------------------------------------------
        function terminate(~)
            % Nothing required.
        end
    end

    %----------------------------------------------------------------------
    methods(Access = private)
        function tf = isFunctionHandleValid(this)
            tf = ~isempty(this.FunctionHandle);
        end
    end
end