classdef SessionManager < handle
%

%   Copyright 2014-2020 The MathWorks, Inc.

    properties
        AppName = 'Camera Calibrator';
        SessionField = 'calibrationSession';
        SessionClass = 'vision.internal.calibration.tool.Session';
        CustomErrorMsgId = 'vision:uitools:invalidSessionFile';
    end
    
    properties(Dependent)
        DefaultSessionFileName;
    end

    methods
        %------------------------------------------------------------------
        function fileName = get.DefaultSessionFileName(this)
            fileName = [this.SessionField, '.mat'];
        end
        
        %------------------------------------------------------------------
        function session = loadSession(this, pathname, filename, appHandle)
            session = [];
            
            try 
                % load the MAT file
                fullname = [pathname, filename];
                temp = load(fullname,'-mat');
               
                if isValidSessionFile(this, temp)
                    % image labeler session
                    session = temp.(this.SessionField);
                    if isempty(session.FileName)
                        session.FileName = fullname;
                    end
                    
                    session.checkImagePaths(pathname, session.FileName); 
                    session.FileName = fullname;
                    
                else
                    errorMsg = getString(message(this.CustomErrorMsgId, ...
                        fullname, this.AppName));
                    dlgTitle = getString(message('vision:uitools:LoadingSessionFailedTitle'));
                    if nargin > 3
                        uialert(appHandle, errorMsg, dlgTitle);
                    else
                        errordlg(errorMsg, dlgTitle, 'modal');
                    end
                end
                
            catch loadSessionEx
                session = [];
                if strcmp(loadSessionEx.identifier, 'MATLAB:load:notBinaryFile')
                    errorMsg = getString(message('vision:uitools:invalidSessionFile',...
                        fullname, this.AppName));
                else
                    errorMsg = loadSessionEx.message;
                end
                
                if nargin > 3
                    uialert(appHandle, ...
                        errorMsg, ...
                        getString(message('vision:uitools:LoadingSessionFailedTitle')));
                else
                    errordlg(errorMsg, ...
                        getString(message('vision:uitools:LoadingSessionFailedTitle')), ...
                        'modal');
                end
            end            
        end
        
        %------------------------------------------------------------------
        function success = saveSession(this, session, filename, appHandle)  
            session.FileName = filename;
            sessionVar = this.SessionField;
            assignSessionVar(sessionVar, session');
                        
            try 
                save(filename, sessionVar);                
                session.IsChanged = false;
                success = true;
            catch savingEx
                
                if nargin > 3
                    uialert(appHandle, ...
                        savingEx.message, ...
                        vision.getMessage('vision:uitools:SavingSessionFailedTitle'));
                else
                    errordlg(savingEx.message, ...
                        vision.getMessage('vision:uitools:SavingSessionFailedTitle'), ...
                        'modal');
                end
                success = false;
            end
        end
    
        %------------------------------------------------------------------
        function isValid = isValidSessionFile(this, sessionStruct)
            % verify that it's a valid session file
            isValid = isfield(sessionStruct, this.SessionField) && ...
                isa(sessionStruct.(this.SessionField), this.SessionClass);
        end    

    end
    
end

%--------------------------------------------------------------------------
function assignSessionVar(sessionVar, session)
% unfortunately there is no way to assign a variable in the current
% funciton's workspace, so we need another level of indirection.
assignin('caller', sessionVar, session');
end
